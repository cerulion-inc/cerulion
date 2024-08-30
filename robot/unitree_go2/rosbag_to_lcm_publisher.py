from pathlib import Path
import os
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
import lcm
from unitree_lcm import joint_states_t
import time

# Create a typestore and get the string class.
typestore = get_typestore(Stores.ROS2_FOXY)

lc = lcm.LCM()

def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

add_types = {}

current_path = os.path.dirname(__file__)
for pathstr in ['unitree_go/msg/LowState.msg', 
                'unitree_go/msg/IMUState.msg',
                'unitree_go/msg/MotorState.msg',
                'unitree_go/msg/BmsState.msg']:
    msgpath = Path(os.path.join(current_path, pathstr))
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

typestore.register(add_types)

# Create reader instance and open for reading.
with Reader('/Users/lakshay/cerulion/bags') as reader:
    # Topic and msgtype information is available on .connections list.
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)

    # Iterate over messages.
    # for connection, timestamp, rawdata in reader.messages():
    #     if connection.topic == '/utlidar/imu':
    #         msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
    #         print(msg.header.frame_id)

    # The .messages() method accepts connection filters.
    connections = [x for x in reader.connections if x.topic == '/lowstate']
    last_timestamp = 0
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        
        if last_timestamp != 0:
            delay_ns = timestamp - last_timestamp
            time.sleep(delay_ns/1000000000.0)
        last_timestamp = timestamp

        # print([[msg.motor_state[i+3*j].q for i in range(3)] for j in range(4)])
        
        lcm_msg = joint_states_t()
        lcm_msg.tick = msg.tick
        lcm_msg.q_front_left = [msg.motor_state[i+3].q for i in range(3)]
        lcm_msg.q_front_right = [msg.motor_state[i].q for i in range(3)]
        lcm_msg.q_rear_left = [msg.motor_state[i+9].q for i in range(3)]
        lcm_msg.q_rear_right = [msg.motor_state[i+6].q for i in range(3)]
        
        lc.publish("JOINT_STATES", lcm_msg.encode())
