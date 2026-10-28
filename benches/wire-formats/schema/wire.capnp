@0xd3c1f0e2a4b59687;
# The same logical content as the Cerulion / ROS 2 messages in the bench.

struct Time { sec @0 :Int32; nanosec @1 :UInt32; }
struct Header { stamp @0 :Time; frameId @1 :Text; }

struct ImuSample {
  # Cap'n Proto has no fixed-size arrays; scalars keep the sample inside the
  # data section (no pointers), which is the fast layout for this format.
  stampNs @0 :UInt64; seq @1 :UInt32; status @2 :UInt32;
  ox @3 :Float64; oy @4 :Float64; oz @5 :Float64; ow @6 :Float64;
  avx @7 :Float64; avy @8 :Float64; avz @9 :Float64;
  lax @10 :Float64; lay @11 :Float64; laz @12 :Float64;
}

struct Image {
  header @0 :Header; height @1 :UInt32; width @2 :UInt32; encoding @3 :Text;
  isBigendian @4 :UInt8; step @5 :UInt32; data @6 :Data;
}

struct PointField { name @0 :Text; offset @1 :UInt32; datatype @2 :UInt8; count @3 :UInt32; }
struct PointCloud2 {
  header @0 :Header; height @1 :UInt32; width @2 :UInt32; fields @3 :List(PointField);
  isBigendian @4 :Bool; pointStep @5 :UInt32; rowStep @6 :UInt32; data @7 :Data; isDense @8 :Bool;
}
