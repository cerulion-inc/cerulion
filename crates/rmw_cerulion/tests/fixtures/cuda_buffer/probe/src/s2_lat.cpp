// One-way latency of a CUDA-backed sensor_msgs/Image through a Lyrical rmw.
//
// The publisher fills a cuda_buffer_backend buffer on the GPU, stamps
// header.stamp with CLOCK_MONOTONIC just before publish(), and publishes.
// With --stamp enqueued (the default) the stamp is taken right after the fill
// is enqueued, so the figure includes the producer's GPU write. With
// --stamp ready the publisher synchronizes its stream first, so the figure
// covers publish, transport, import and the consumer-side wait only.
// The subscriber stops the clock when the payload is usable on its own
// stream: for a "cuda" buffer, after from_input_buffer (which enqueues the
// wait on the producer's event) and a stream synchronize; for a "cpu"
// buffer, on callback entry (the bytes are already in host memory). After
// the clock stops it checks the first payload byte against the sequence fill.
//
//   s2_lat --role pub|sub|both --size BYTES --count N --warmup W --rate HZ
//          [--accept cuda|cpu] [--intra] [--stamp enqueued|ready] [--out FILE.csv]
//
// --role both runs publisher and subscriber in one process, each node on its
// own executor thread; --intra turns on rclcpp intra-process comms there.
#include <cuda_runtime.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "probe_common.hpp"

using Image = sensor_msgs::msg::Image;

struct Args
{
  std::string role{"both"};
  size_t size{4096};
  int count{1000};
  int warmup{100};
  double rate{100.0};
  std::string accept{"cuda"};
  bool intra{false};
  bool stamp_ready{false};
  std::string out{"lat.csv"};
};

static Args parse(int argc, char ** argv)
{
  Args a;
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    auto v = [&]() {return std::string(i + 1 < argc ? argv[++i] : "");};
    if (k == "--role") {a.role = v();} else if (k == "--size") {
      a.size = std::stoull(v());
    } else if (k == "--count") {a.count = std::stoi(v());} else if (k == "--warmup") {
      a.warmup = std::stoi(v());
    } else if (k == "--rate") {a.rate = std::stod(v());} else if (k == "--accept") {
      a.accept = v();
    } else if (k == "--intra") {a.intra = true;} else if (k == "--out") {a.out = v();} else if (
      k == "--stamp")
    {
      a.stamp_ready = (v() == "ready");
    }
  }
  return a;
}

static rclcpp::QoS qos() {return rclcpp::QoS(rclcpp::KeepLast(10)).reliable();}

class Pub : public rclcpp::Node
{
public:
  Pub(const Args & a, const rclcpp::NodeOptions & o)
  : Node("s2_lat_pub", o), a_(a)
  {
    cudaStreamCreate(&stream_);
    pub_ = create_publisher<Image>("s2_lat", qos());
  }
  ~Pub() override {cudaStreamDestroy(stream_);}

  void run(const std::atomic<bool> & stop)
  {
    while (!stop && pub_->get_subscription_count() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // Let backend discovery settle before the first sample.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / a_.rate));
    auto next = std::chrono::steady_clock::now();
    const int total = a_.count + a_.warmup;
    for (int seq = 0; seq < total && !stop && rclcpp::ok(); ++seq) {
      next += period;
      auto msg = std::make_unique<Image>();
      msg->header.frame_id = std::to_string(seq);
      msg->height = 1;
      msg->width = static_cast<uint32_t>(a_.size);
      msg->encoding = "mono8";
      msg->step = static_cast<uint32_t>(a_.size);
      msg->data = cuda_buffer_backend::allocate_buffer(a_.size);
      {
        auto wh = cuda_buffer_backend::from_output_buffer(msg->data, stream_);
        cudaMemsetAsync(wh.get_ptr(), seq & 0xff, a_.size, stream_);
      }
      if (a_.stamp_ready) {cudaStreamSynchronize(stream_);}
      const uint64_t t = s2probe::mono_ns();
      msg->header.stamp.sec = static_cast<int32_t>(t / 1000000000ull);
      msg->header.stamp.nanosec = static_cast<uint32_t>(t % 1000000000ull);
      pub_->publish(std::move(msg));
      std::this_thread::sleep_until(next);
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

private:
  Args a_;
  cudaStream_t stream_{nullptr};
  rclcpp::Publisher<Image>::SharedPtr pub_;
};

class Sub : public rclcpp::Node
{
public:
  Sub(const Args & a, const rclcpp::NodeOptions & o)
  : Node("s2_lat_sub", o), a_(a)
  {
    cudaStreamCreate(&stream_);
    rows_.reserve(static_cast<size_t>(a.count + a.warmup));
    rclcpp::SubscriptionOptions so;
    so.acceptable_buffer_backends = a.accept;
    sub_ = create_subscription<Image>(
      "s2_lat", qos(), [this](Image::ConstSharedPtr m) {on_msg(*m);}, so);
  }
  ~Sub() override {cudaStreamDestroy(stream_);}

  bool done() const {return received_ >= a_.count + a_.warmup;}

  void write_csv() const
  {
    FILE * f = std::fopen(a_.out.c_str(), "w");
    std::fprintf(f, "seq,lat_ns,backend,size,ok\n");
    for (const auto & r : rows_) {
      std::fprintf(f, "%d,%lld,%s,%zu,%d\n", r.seq, static_cast<long long>(r.lat),
        r.backend.c_str(), r.size, r.ok ? 1 : 0);
    }
    std::fclose(f);
  }

private:
  struct Row
  {
    int seq;
    int64_t lat;
    std::string backend;
    size_t size;
    bool ok;
  };

  void on_msg(const Image & m)
  {
    const std::string backend = m.data.get_backend_type();
    const size_t n = m.data.size();
    uint8_t first = 0;
    uint64_t t1 = 0;
    if (backend == "cuda" && n > 0) {
      auto rh = cuda_buffer_backend::from_input_buffer(m.data, stream_);
      cudaStreamSynchronize(stream_);
      t1 = s2probe::mono_ns();
      cudaMemcpy(&first, rh.get_ptr(), 1, cudaMemcpyDeviceToHost);
    } else {
      t1 = s2probe::mono_ns();
      if (n > 0) {
        const std::vector<uint8_t> & host = m.data;
        first = host[0];
      }
    }
    const uint64_t t0 = static_cast<uint64_t>(m.header.stamp.sec) * 1000000000ull +
      m.header.stamp.nanosec;
    const int seq = std::stoi(m.header.frame_id);
    rows_.push_back({seq, static_cast<int64_t>(t1 - t0), backend, n,
        n == a_.size && first == static_cast<uint8_t>(seq & 0xff)});
    ++received_;
  }

  Args a_;
  cudaStream_t stream_{nullptr};
  rclcpp::Subscription<Image>::SharedPtr sub_;
  std::vector<Row> rows_;
  std::atomic<int> received_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const Args a = parse(argc, argv);
  rclcpp::NodeOptions o;
  o.use_intra_process_comms(a.intra);
  std::atomic<bool> stop{false};
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::seconds(30 + static_cast<int>((a.count + a.warmup) / a.rate));

  std::shared_ptr<Sub> sub;
  std::shared_ptr<Pub> pub;
  rclcpp::executors::SingleThreadedExecutor sub_exec, pub_exec;
  std::thread sub_thread, pub_spin;
  if (a.role == "sub" || a.role == "both") {
    sub = std::make_shared<Sub>(a, o);
    sub_exec.add_node(sub);
    sub_thread = std::thread([&]() {
          while (rclcpp::ok() && !sub->done() && std::chrono::steady_clock::now() < deadline) {
            sub_exec.spin_once(std::chrono::milliseconds(100));
          }
        });
  }
  if (a.role == "pub" || a.role == "both") {
    pub = std::make_shared<Pub>(a, o);
    pub_exec.add_node(pub);
    pub_spin = std::thread([&]() {
          while (rclcpp::ok() && !stop) {pub_exec.spin_once(std::chrono::milliseconds(100));}
        });
    pub->run(stop);
  }
  if (sub_thread.joinable()) {sub_thread.join();}
  stop = true;
  if (pub_spin.joinable()) {pub_spin.join();}
  int rc = 0;
  if (sub) {
    sub->write_csv();
    rc = sub->done() ? 0 : 3;
  }
  rclcpp::shutdown();
  return rc;
}
