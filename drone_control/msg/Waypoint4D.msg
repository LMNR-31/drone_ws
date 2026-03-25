# 4D waypoint: position (x, y, z) plus optional absolute yaw around the Z axis.
# Convention: yaw = NaN means "no yaw provided – keep current heading".
# yaw is in radians; values are normalized to [-pi, pi] by the controller.
geometry_msgs/Pose pose
float32 yaw
