from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
import lcm
from unitree_lcm import joint_states_t

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

for pathstr in ['/mnt/c/Users/laksh/Desktop/Research/RenardOS/demo_bags/unitree_go/msg/LowState.msg', 
                '/mnt/c/Users/laksh/Desktop/Research/RenardOS/demo_bags/unitree_go/msg/IMUState.msg',
                '/mnt/c/Users/laksh/Desktop/Research/RenardOS/demo_bags/unitree_go/msg/MotorState.msg',
                '/mnt/c/Users/laksh/Desktop/Research/RenardOS/demo_bags/unitree_go/msg/BmsState.msg']:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

typestore.register(add_types)

# Create reader instance and open for reading.
with Reader('/mnt/c/Users/laksh/Desktop/Research/RenardOS/demo_bags/rosbag2_2024_08_07-16_47_08') as reader:
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
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        # print([[msg.motor_state[i+3*j].q for i in range(3)] for j in range(4)])
        
        lcm_msg = joint_states_t()
        lcm_msg.tick = msg.tick
        lcm_msg.q_front_left = [msg.motor_state[i].q for i in range(3)]
        lcm_msg.q_front_right = [msg.motor_state[i+3].q for i in range(3)]
        lcm_msg.q_rear_left = [msg.motor_state[i+6].q for i in range(3)]
        lcm_msg.q_rear_right = [msg.motor_state[i+9].q for i in range(3)]
        
        lc.publish("JOINT_STATES", lcm_msg.encode())
