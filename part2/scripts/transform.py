import numpy as np
from part2.scripts.a_star_path import WIDTH, HEIGHT
from scipy.spatial.transform import Rotation as R


# def transform_map_to_gazebo(x_m: float, y_m: float, yaw_m: float):
#     x_gz: float = x_m
#     x_gz: float = y_m
#     yaw_gz: float = yaw_m

#     position_m = np.array([x_m, HEIGHT - y_m, 0.0])
#     orientation_m = R.from_euler("xyz", [0, 0, yaw_m], degrees=True).as_matrix()

#     # Homogeneous transformation matrix of pose in Map
#     T_pose_m = np.eye(4)
#     T_pose_m[:3, :3] = orientation_m
#     T_pose_m[:3, 3] = position_m

#     # ---- Transform from Frame A to Frame B ----
#     # Example: Frame B is rotated and translated w.r.t Frame A
#     rotation_AB = R.from_euler("xyz", [180, 0, 0], degrees=True).as_matrix()
#     translation_AB = np.array([0, 0, 0])

#     T_AB = np.eye(4)
#     T_AB[:3, :3] = rotation_AB
#     T_AB[:3, 3] = translation_AB

#     # Now transform the pose from frame A to frame B
#     T_pose_in_B = np.linalg.inv(T_AB) @ T_pose_in_A

#     # Extract the new position and orientation
#     position_B = T_pose_in_B[:3, 3]
#     orientation_B = R.from_matrix(T_pose_in_B[:3, :3]).as_euler("xyz", degrees=True)

#     return position_B[0], position_B[1], orientation_B[2]

#     return x_gz, yaw_gz, yaw_gz

