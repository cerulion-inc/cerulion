# Admittance parameters
std_msgs/Float64MultiArray mass  # 6-vector of mass terms used in the admittance calculation
std_msgs/Float64MultiArray damping # 6-vector of damping terms used in the admittance calculation
std_msgs/Float64MultiArray stiffness # 6-vector of stiffness terms used in the admittance calculation

# Frame information
geometry_msgs/Quaternion rot_base_control # quaternion describing the orientation of the control frame
geometry_msgs/TransformStamped ref_trans_base_ft # force torque sensor transform at the reference joint configuration
std_msgs/Int8MultiArray selected_axes # 6-vector of 0/1 describing if admittance is enable in the corresponding control frame axis
std_msgs/String ft_sensor_frame # name of the force torque frame

# State information
geometry_msgs/TransformStamped admittance_position # calculated admittance position in cartesian space
geometry_msgs/TwistStamped admittance_acceleration # calculated admittance acceleration in cartesian space
geometry_msgs/TwistStamped admittance_velocity # calculated admittance velocity in cartesian space
geometry_msgs/WrenchStamped wrench_base # wrench used in the admittance calculation
sensor_msgs/JointState joint_state # calculated admittance offsets in joint space
