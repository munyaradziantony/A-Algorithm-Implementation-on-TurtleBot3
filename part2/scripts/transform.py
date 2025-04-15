from scipy.spatial.transform import Rotation as R
import numpy as np
# from search import get_search_canvas_width,get_search_canvas_height


def transform_map_to_robot(x, y, theta):
    """
    Transform pose in map to robot
    x y in mm
    yaw in degrees
    """
    position_A = np.array([x, y, 0.0])
    orientation_A = R.from_euler('xyz', [0, 0, theta], degrees=True).as_matrix()

    # Homogeneous transformation matrix of pose in A
    T_pose_in_A = np.eye(4)
    T_pose_in_A[:3, :3] = orientation_A
    T_pose_in_A[:3, 3] = position_A

    # ---- Transform from Frame A to Frame B ----
    # Example: Frame B is rotated and translated w.r.t Frame A
    rotation_AB = R.from_euler('X', -3.14, degrees=False).as_matrix()
    translation_AB = np.array([0, 3000/2, 0])

    T_AB = np.eye(4)
    T_AB[:3, :3] = rotation_AB
    T_AB[:3, 3] = translation_AB

    # Now transform the pose from frame A to frame B
    T_pose_in_B = np.linalg.inv(T_AB) @ T_pose_in_A

    # Extract the new position and orientation
    position_B = T_pose_in_B[:3, 3]
    orientation_B = R.from_matrix(T_pose_in_B[:3, :3]).as_euler('xyz', degrees=True)

    return position_B[0], position_B[1], orientation_B[2]


def transform_robot_to_map(x, y, yaw):
    """
    Transform a pose in robot to map(top left corner)
    x y in mm

    yaw in degrees
    """
    # ---- Pose in Frame A ----
    position_A = np.array([x, y, 0.0])
    orientation_A = R.from_euler('xyz', [0, 0, yaw], degrees=True).as_matrix()

    # Homogeneous transformation matrix of pose in A
    T_pose_in_A = np.eye(4)
    T_pose_in_A[:3, :3] = orientation_A
    T_pose_in_A[:3, 3] = position_A

    # ---- Transform from Frame A to Frame B ----
    # Example: Frame B is rotated and translated w.r.t Frame A
    rotation_AB = R.from_euler('X', -3.14, degrees=False).as_matrix()
    translation_AB = np.array([0, 3000/2, 0])

    T_AB = np.eye(4)
    T_AB[:3, :3] = rotation_AB
    T_AB[:3, 3] = translation_AB

    # Now transform the pose from frame A to frame B
    T_pose_in_B = np.linalg.inv(T_AB) @ T_pose_in_A

    # Extract the new position and orientation
    position_B = T_pose_in_B[:3, 3]
    orientation_B = R.from_matrix(T_pose_in_B[:3, :3]).as_euler('xyz', degrees=True)

    return position_B[0], position_B[1], orientation_B[2]