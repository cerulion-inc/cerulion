import re

import numpy as np
import pytest

import cerulion

from conftest import finish_proc, spawn_fixture, unique_topic, wait_ready


VECTOR3_MSG = "float64 x\nfloat64 y\nfloat64 z\n"
TIME_MSG = "int32 sec\nuint32 nanosec\n"
HEADER_MSG = "builtin_interfaces/Time stamp\nstring frame_id\n"
LASER_SCAN_MSG = """\
std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
"""


def ros_schemas():
    schemas = cerulion.SchemaSet()
    schemas.add_rosmsg(TIME_MSG, "builtin_interfaces/Time")
    schemas.add_rosmsg(HEADER_MSG, "std_msgs/Header")
    schemas.add_rosmsg(VECTOR3_MSG, "geometry_msgs/Vector3")
    schemas.add_rosmsg(LASER_SCAN_MSG, "sensor_msgs/LaserScan")
    return schemas


# Hand-written oracle for the LaserScan both Python writers publish, as the
# Rust fixture prints it after decoding the frame with generated types.
LASER_SCAN_VALUES = (
    "angle_min=-1.5 angle_max=1.5 angle_increment=0.25 time_increment=0.001 "
    "scan_time=0.1 range_min=0.2 range_max=30 ranges=[1.5, 2.25, 3.0, 0.5] "
    "intensities=[10.0, 20.0, 30.0, 40.0] header_stamp_sec=7 "
    "header_stamp_nanosec=9 frame_id=laser"
)


def frame_hex(frame):
    raw = bytearray(frame.raw)
    raw[20:32] = b"\0" * 12
    return raw.hex()


def output_frame_hex(output):
    match = re.search(r"^frame_hex=([0-9a-f]+)$", output, re.MULTILINE)
    assert match, output
    return match.group(1)


def test_rust_vector3_decodes_as_typed_message(session, fixture_bin):
    schemas = ros_schemas()
    topic = unique_topic("interop-rust-vector3")
    proc = spawn_fixture(
        fixture_bin,
        [
            "publish-typed",
            "--topic",
            topic,
            "--schema",
            "geometry_msgs/Vector3",
            "--count",
            "1",
        ],
    )
    wait_ready(proc)
    sub = session.subscriber(topic, schema="geometry_msgs/Vector3", schemas=schemas)
    frame = sub.receive(5000)
    assert frame is not None
    message = frame.view()
    assert (message.x, message.y, message.z) == (1.5, -2.25, 1e-3)
    finish_proc(proc)
    frame.release()


def test_rust_laserscan_decodes_as_typed_message(session, fixture_bin):
    schemas = ros_schemas()
    topic = unique_topic("interop-rust-laserscan")
    proc = spawn_fixture(
        fixture_bin,
        [
            "publish-typed",
            "--topic",
            topic,
            "--schema",
            "sensor_msgs/LaserScan",
            "--count",
            "1",
        ],
    )
    wait_ready(proc)
    sub = session.subscriber(topic, schema="sensor_msgs/LaserScan", schemas=schemas)
    frame = sub.receive(5000)
    assert frame is not None
    message = frame.view()
    assert message.angle_min == pytest.approx(-1.5)
    assert message.angle_max == pytest.approx(1.5)
    assert message.angle_increment == pytest.approx(0.25)
    assert message.time_increment == pytest.approx(0.001)
    assert message.scan_time == pytest.approx(0.1)
    assert message.range_min == pytest.approx(0.2)
    assert message.range_max == pytest.approx(30.0)
    np.testing.assert_allclose(message.ranges, [1.5, 2.25, 3.0, 0.5])
    np.testing.assert_allclose(message.intensities, [10.0, 20.0, 30.0, 40.0])
    assert message.header.stamp.sec == 7
    assert message.header.frame_id == "laser"
    finish_proc(proc)
    frame.release()


def _python_to_rust(schema, payload, session, fixture_bin, schemas, name, expected_values):
    """Publish ``payload`` from Python; the Rust fixture's decoded field
    values must match the hand-written ``expected_values`` oracle, and its
    frame bytes must match the frame a Python subscriber received."""
    topic = unique_topic(name)
    proc = spawn_fixture(
        fixture_bin,
        [
            "subscribe-typed",
            "--topic",
            topic,
            "--schema",
            schema,
            "--count",
            "1",
            "--timeout-ms",
            "10000",
        ],
    )
    wait_ready(proc)
    pub = session.publisher(topic, schema=schema, schemas=schemas)
    sub = session.subscriber(topic, schema=schema, schemas=schemas)
    pub.publish(payload)
    frame = sub.receive(5000)
    assert frame is not None
    expected_hex = frame_hex(frame)
    finish_proc(proc)
    output = proc.stdout.read() if proc.stdout else ""
    frame.release()
    assert f"schema={schema} values={expected_values}\n" in output, output
    assert output_frame_hex(output) == expected_hex


def test_python_typed_loan_pins_vector3_bytes(session, fixture_bin):
    schemas = ros_schemas()
    topic = unique_topic("interop-python-loan-vector3")
    proc = spawn_fixture(
        fixture_bin,
        [
            "subscribe-typed",
            "--topic",
            topic,
            "--schema",
            "geometry_msgs/Vector3",
            "--count",
            "1",
            "--timeout-ms",
            "10000",
        ],
    )
    wait_ready(proc)
    pub = session.publisher(topic, schema="geometry_msgs/Vector3", schemas=schemas)
    sub = session.subscriber(topic, schema="geometry_msgs/Vector3", schemas=schemas)
    with pub.loan() as message:
        message.x = 1.5
        message.y = -2.25
        message.z = 1e-3
    frame = sub.receive(5000)
    assert frame is not None
    expected_hex = frame_hex(frame)
    finish_proc(proc)
    output = proc.stdout.read()
    frame.release()
    assert output_frame_hex(output) == expected_hex


def test_python_dict_pins_vector3_bytes(session, fixture_bin):
    schemas = ros_schemas()
    _python_to_rust(
        "geometry_msgs/Vector3",
        {"x": 1.5, "y": -2.25, "z": 1e-3},
        session,
        fixture_bin,
        schemas,
        "interop-python-dict-vector3",
        "x=1.5 y=-2.25 z=0.001",
    )


def test_python_typed_loan_pins_laserscan_bytes(session, fixture_bin):
    schemas = ros_schemas()
    topic = unique_topic("interop-python-loan-laserscan")
    proc = spawn_fixture(
        fixture_bin,
        [
            "subscribe-typed",
            "--topic",
            topic,
            "--schema",
            "sensor_msgs/LaserScan",
            "--count",
            "1",
            "--timeout-ms",
            "10000",
        ],
    )
    wait_ready(proc)
    pub = session.publisher(topic, schema="sensor_msgs/LaserScan", schemas=schemas)
    sub = session.subscriber(topic, schema="sensor_msgs/LaserScan", schemas=schemas)
    with pub.loan(
        header=21,
        ranges=4,
        intensities=4,
    ) as message:
        message.header = bytes(
            b"\x07\0\0\0\x09\0\0\0\x10\0\0\0\x05\0\0\0laser"
        )
        message.angle_min = -1.5
        message.angle_max = 1.5
        message.angle_increment = 0.25
        message.time_increment = 0.001
        message.scan_time = 0.1
        message.range_min = 0.2
        message.range_max = 30.0
        message.ranges[:] = [1.5, 2.25, 3.0, 0.5]
        message.intensities[:] = [10.0, 20.0, 30.0, 40.0]
    frame = sub.receive(5000)
    assert frame is not None
    expected_hex = frame_hex(frame)
    finish_proc(proc)
    output = proc.stdout.read()
    frame.release()
    assert output_frame_hex(output) == expected_hex
    assert LASER_SCAN_VALUES in output, output


def test_python_dict_pins_laserscan_bytes(session, fixture_bin):
    schemas = ros_schemas()
    payload = {
        "header": {"stamp": {"sec": 7, "nanosec": 9}, "frame_id": "laser"},
        "angle_min": -1.5,
        "angle_max": 1.5,
        "angle_increment": 0.25,
        "time_increment": 0.001,
        "scan_time": 0.1,
        "range_min": 0.2,
        "range_max": 30.0,
        "ranges": [1.5, 2.25, 3.0, 0.5],
        "intensities": [10.0, 20.0, 30.0, 40.0],
    }
    _python_to_rust(
        "sensor_msgs/LaserScan",
        payload,
        session,
        fixture_bin,
        schemas,
        "interop-python-dict-laserscan",
        LASER_SCAN_VALUES,
    )


def test_rust_subscribe_typed_rejects_wrong_schema_hash(session, fixture_bin):
    topic = unique_topic("interop-bad-hash")
    proc = spawn_fixture(
        fixture_bin,
        [
            "subscribe-typed",
            "--topic",
            topic,
            "--schema",
            "geometry_msgs/Vector3",
            "--count",
            "1",
            "--timeout-ms",
            "10000",
        ],
    )
    wait_ready(proc)
    pub = session.publisher(topic, schema_hash=0xDEADBEEF)
    pub.publish(b"\0" * 24)
    proc.wait(timeout=15)
    assert proc.returncode != 0
    assert "schema hash mismatch" in proc.stderr.read()


def test_rust_subscribe_typed_rejects_truncated_body(session, fixture_bin):
    schemas = ros_schemas()
    topic = unique_topic("interop-truncated")
    proc = spawn_fixture(
        fixture_bin,
        [
            "subscribe-typed",
            "--topic",
            topic,
            "--schema",
            "geometry_msgs/Vector3",
            "--count",
            "1",
            "--timeout-ms",
            "10000",
        ],
    )
    wait_ready(proc)
    pub = session.publisher(
        topic, schema_hash=schemas.schema_hash("geometry_msgs/Vector3")
    )
    pub.publish(b"\0" * 8)
    proc.wait(timeout=15)
    assert proc.returncode != 0
    assert "shorter" in proc.stderr.read()
