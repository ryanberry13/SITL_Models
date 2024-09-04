""""
Obtain the pose of the camera and omnirover

For testing the GSoC 2024 Camera Tracking project


Gazebo Topics
-------------

Static pose information.
/world/playpen/pose/info

The dynamic pose info does not provide any information about the model tree
and provides relative poses rather than world poses.
/world/playpen/dynamic_pose/info

/gui/camera/pose

Plugin: OdometryPublisher
pose: pose of model in the ENU world frame adjusted for optional offset.
odometry: pose and twist in the ENU world frame.
odometry_wkth_covariance: odometry with estimated covariance in ENU frame. 

/model/omni4rover/odometry
/model/omni4rover/odometry_with_covariance
/model/omni4rover/pose

Plugin: JointStatePublisher
joint_state: gz.msgs.Model
/world/playpen/model/omni4rover/joint_state


Camera Sensor
/world/playpen/model/mount/model/gimbal/link/pitch_link/sensor/camera/camera_info
/world/playpen/model/mount/model/gimbal/link/pitch_link/sensor/camera/image

Usage
-----

python ./src/ardupilot_sitl_models/Gazebo/scripts/omnirover_pose.py


"""

import numpy as np
import time
import unittest
import cv2

from threading import Lock
from threading import Thread

from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.pose_v_pb2 import Pose_V

from gz.transport13 import Node

from transforms3d import affines
from transforms3d import euler
from transforms3d import quaternions


class Pose:
    def __init__(self, world, body, position, orientation):
        self.world = world
        self.body = body
        self.position = position
        self.orientation = orientation

    def __str__(self):
        s = (
            "world: "
            + str(self.world)
            + "\n"
            + "body: "
            + str(self.body)
            + "\n"
            + "position: "
            + str(self.position)
            + "\n"
            + "orientation: "
            + str(self.orientation)
            + "\n"
            + "euler: "
            + str(euler.quat2euler(self.orientation))
        )
        return s


class CameraInformation:
    def __init__(self, msg: CameraInfo):
        self.width = msg.width
        self.height = msg.height
        self.fx = msg.projection.p[0]
        self.fy = msg.projection.p[5]
        self.cx = msg.projection.p[2]
        self.cy = msg.projection.p[6]
        self.tx = msg.projection.p[3]
        self.ty = msg.projection.p[7]
        self.s = msg.projection.p[1]

    def __str__(self):
        s = (
            "fx: "
            + str(self.fx)
            + "\n"
            + "fy: "
            + str(self.fy)
            + "\n"
            + "cx: "
            + str(self.cx)
            + "\n"
            + "cy: "
            + str(self.cy)
            + "\n"
            + "tx: "
            + str(self.tx)
            + "\n"
            + "ty: "
            + str(self.ty)
            + "\n"
            + "s: "
            + str(self.s)
        )
        return s

    @property
    def camera_matrix(self):
        P = [
            [self.fx, self.s, self.cx, self.tx],
            [0, self.fy, self.cy, self.ty],
            [0, 0, 1, 0],
        ]
        return np.array(P)


class PoseMonitor:
    """
    Calculate camera and target pose from pose_v messages.
    """

    def __init__(self):
        self.WORLD_NAME = "playpen"
        self.TARGET_NAME = "omni4rover"
        self.CAMERA_NAME = "mount"

        self._node = Node()

        # mutex for access to messages in subsciber callbacks
        self._mutex = Lock()

        # subscribe to pose_v messages
        self._target_pose_v_do_print_msg = True
        self._target_pose_v_msg = None
        self._target_pose_v_topic = f"/model/{self.TARGET_NAME}/pose"
        self._target_pose_v_sub = self._node.subscribe(
            Pose_V, self._target_pose_v_topic, self.target_pose_v_cb
        )
        self._target_pose = None

        self._camera_pose_v_do_print_msg = True
        self._camera_pose_v_msg = None
        self._camera_pose_v_topic = f"/model/{self.CAMERA_NAME}/pose"
        self._camera_pose_v_sub = self._node.subscribe(
            Pose_V, self._camera_pose_v_topic, self.camera_pose_v_cb
        )
        self._camera_pose = None

        # subscribe to camera_info messages
        self._camera_info_do_print_msg = True
        self._camera_info_msg = None
        # self._camera_info_topic = f"/world/{self.WORLD_NAME}/model/{self.CAMERA_NAME}/model/gimbal/link/pitch_link/sensor/camera/camera_info"
        self._camera_info_topic = "/world/playpen/model/mount/model/gimbal/link/pitch_link/sensor/camera/camera_info"
        self._camera_info_sub = self._node.subscribe(
            CameraInfo, self._camera_info_topic, self.camera_info_cb
        )
        self._camera_info = None

        # create update thread
        self._update_thread = Thread(target=self.update)
        self._update_thread.run()

    def target_pose_v_cb(self, msg: Pose_V):
        with self._mutex:
            self._target_pose_v_msg = msg

        if self._target_pose_v_do_print_msg:
            self._target_pose_v_do_print_msg = False
            print("target pose")
            for pose in self._target_pose_v_msg.pose:
                print(pose.name)

    def camera_pose_v_cb(self, msg: Pose_V):
        with self._mutex:
            self._camera_pose_v_msg = msg

        if self._camera_pose_v_do_print_msg:
            self._camera_pose_v_do_print_msg = False
            print("camera pose")
            for pose in self._camera_pose_v_msg.pose:
                print(pose.name)

    def camera_info_cb(self, msg: CameraInfo):
        with self._mutex:
            self._camera_info_msg = msg

        if self._camera_info_do_print_msg:
            self._camera_info_do_print_msg = False
            print("camera info")
            print(self._camera_info_msg)

    def update(self):
        update_rate = 1.0
        update_period = 1.0 / update_rate
        while True:
            if self._camera_pose_v_msg is not None:
                world = self.WORLD_NAME
                body = "mount::gimbal::pitch_link::camera"
                self._camera_pose = self.body_to_world(
                    self._camera_pose_v_msg, world, body
                )
                # print(self._camera_pose)
                # print()

            if self._target_pose_v_msg is not None:
                world = self.WORLD_NAME
                body = "omni4rover"
                self._target_pose = self.body_to_world(
                    self._target_pose_v_msg, world, body
                )
                # print(self._target_pose)
                # print()

            if (self._target_pose is not None) and (self._camera_pose is not None):
                self._camera_info = CameraInformation(self._camera_info_msg)
                # print(self._camera_info)

                # projection matrix (=opencv camera_matrix)
                camera_matrix = self._camera_info.camera_matrix
                image_point = project_point(
                    camera_matrix, self._camera_pose, self._target_pose
                )
                print(f"image_point: {image_point}")

            time.sleep(update_period)

    def body_to_world(self, pose_v_msg, world, body):
        """
        Calculate the transform from body to world from a pose_v_msg
        """
        # create dictionaries for the poses
        pose_dict = {}
        for pose in pose_v_msg.pose:
            pose_dict[pose.name] = pose

        child = body
        # print(f"{child}")
        parent = None
        X = np.identity(4)
        # print(X)
        while parent != world:
            pose = pose_dict[child]

            for data in pose.header.data:
                if data.key == "frame_id":
                    parent = data.value[0]
                    break

            # print(f"--> {child}")
            child = parent

            # convert pose to an affine transform
            X_p_c = np.array(self.pose_to_tf(pose))
            X = np.matmul(X_p_c, X)

        # print(f"--> {parent}")

        t, r, z, s = affines.decompose(X)
        q = quaternions.mat2quat(r)
        pose = Pose(world, body, t, q)
        return pose

    def pose_to_tf(self, pose):
        """
        Convert a pose message into an transforms3d.affine
        """
        t = [pose.position.x, pose.position.y, pose.position.z]
        q = [
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]
        r = quaternions.quat2mat(q)
        z = [1.0, 1.0, 1.0]
        a = affines.compose(t, r, z)
        # print(f"p: {t}")
        # print(f"q: {q}")
        # print(f"r: {r}")
        # print(f"a: {a}")
        return a


def project_point(camera_matrix, camera_pose, target_pose):
    """
    camera info
    header {
      stamp {
        sec: 1353
        nsec: 300000000
      }
      data {
        key: "frame_id"
        value: "mount::gimbal::pitch_link::camera"
      }
    }
    width: 640
    height: 480
    distortion {
      k: 0
      k: 0
      k: 0
      k: 0
      k: 0
    }
    intrinsics {
      k: 205.46962738037109
      k: 0
      k: 320
      k: 0
      k: 205.46965599060059
      k: 240
      k: 0
      k: 0
      k: 1
    }
    projection {
      p: 205.46962738037109   -> fx [0]
      p: 0                    -> s  [1]
      p: 320                  -> cx [2]
      p: 0                    -> tx [3]
      p: 0
      p: 205.46965599060059   -> fy [5]
      p: 240                  -> cy [6]
      p: 0                    -> ty [7]
      p: 0
      p: 0
      p: 1
      p: 0
    }
    rectification_matrix: 1
    rectification_matrix: 0
    rectification_matrix: 0
    rectification_matrix: 0
    rectification_matrix: 1
    rectification_matrix: 0
    rectification_matrix: 0
    rectification_matrix: 0
    rectification_matrix: 1
    """
    # from gz-msgs/src/gz/msgs/proto/camera_info.proto
    #
    # fx is the X Focal length
    # fy is the Y Focal length
    # cx is the X principal point
    # cy is the Y principal point
    # tx is the X position of the second camera in this camera's frame.
    # ty is the Y position of the second camera in this camera's frame.
    # s is the axis skew.
    #
    # intrinsic camera matrix for the raw (distorted) images can be
    # generated using the fx, fy, cx, and cy parameters contained in this
    # message. For example the intrinsic camera matrix K would be:
    #      [fx  s cx]
    #  K = [ 0 fy cy]
    #      [ 0  0  1]
    # Projects 3D points in the camera coordinate frame to 2D pixel
    # coordinates using the focal lengths (fx, fy) and principal point
    # (cx, cy).
    #
    # The projection/camera matrix can be generated using the values in
    # this message. For example, the projection matrix P would be:
    #
    #     [fx   s cx tx]
    # P = [ 0  fy cy ty]
    #     [ 0   0  1  0]
    #
    # fx = 205.46962738037109
    # fy = 205.46962738037109
    # cx = 320
    # cy = 240
    # tx = 0.0
    # ty = 0.0
    # s = 0.0
    # # projection matrix (opencv camera_matix)
    # P = [[fx, s, cx, tx], [0, fy, cy, ty], [0, 0, 1, 0]]

    # projection matrix (opencv camera_matix)
    P = camera_matrix

    # create the affines
    t = camera_pose.position
    r = quaternions.quat2mat(camera_pose.orientation)
    z = [1.0, 1.0, 1.0]
    X_w_c = affines.compose(t, r, z)

    t = target_pose.position
    r = quaternions.quat2mat(target_pose.orientation)
    z = [1.0, 1.0, 1.0]
    X_w_b = affines.compose(t, r, z)

    # compute the transform from the target frame to camera frame
    X_c_w = np.linalg.inv(X_w_c)
    X_c_b = np.matmul(X_c_w, X_w_b)
    # print(X_c_b)

    # transform the origin of the target to the camera frame
    v_b_b = np.transpose([[0, 0, 0, 1]])
    v_b_c = np.matmul(X_c_b, v_b_b)
    # print(v_b_b)
    # print(v_b_c)

    # rotate from the camera sensor frame to the camera optical frame
    #   camera sensor frame: FRU
    #   camera optical frame: RDF
    t = [0.0, 0.0, 0.0]
    q = euler.euler2quat(np.radians(90.0), 0.0, np.radians(90.0))
    r = quaternions.quat2mat(q)
    z = [1.0, 1.0, 1.0]
    X_c_o = affines.compose(t, r, z)
    X_o_c = np.linalg.inv(X_c_o)
    v_b_o = np.matmul(X_o_c, v_b_c)
    # print(f"v_b_o:\n{v_b_o[0:3]}")

    # project the target to screen space
    uvw = np.matmul(P, v_b_o)
    u = uvw[0, 0]
    v = uvw[1, 0]
    w = uvw[2, 0]
    # print(f"u: {u / w:0.1f}, v: {v / w:0.1f}")

    # using opencv
    # https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#projectpoints
    object_points = v_b_o[0:3, 0]
    # print(f"object_points:\n{object_points}")

    # set rvec, tvec to zero as object is already in the camera optical frame
    # rot3 = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    # rvec, _ = cv2.Rodrigues(rot3)
    rvec = np.array([0.0, 0.0, 0.0])
    tvec = np.array([0.0, 0.0, 0.0])

    # camera_matrix is the top-left 3x3 submatrix of P
    # camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    camera_matrix = P[0:3, 0:3]
    dist_coeffs = np.array([])
    image_points, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, dist_coeffs
    )
    # print(f"image_points:\n{image_points[0][0]}")

    return [u / w, v / w]


def main():
    pose_monitor = PoseMonitor()
    loop_rate_hz = 50.0
    loop_duration_s = 1.0 / loop_rate_hz

    try:
        while True:
            time.sleep(loop_duration_s)

    except KeyboardInterrupt:
        print("Exiting")


def test_project_to_screen_space():
    # test data corresponds to the starting postions in the playpen world
    world = "playpen"
    body = "mount::gimbal::pitch_link::camera"
    position = [-4.99624617e-09, -4.99999999e00, 9.05499925e-01]
    orientation = [7.06825124e-01, 2.83202326e-04, 2.81651369e-04, 7.07388213e-01]
    camera_pose = Pose(world, body, position, orientation)

    world = "playpen"
    body = "omni4rover"
    position = [-8.41037891e-03, 8.21343606e-05, 4.88579769e-02]
    orientation = [7.07106701e-01, -6.08646112e-07, 3.60883955e-07, 7.07106862e-01]
    target_pose = Pose(world, body, position, orientation)

    # print(camera_pose)
    # print(target_pose)

    # camera intrinsics
    fx = 205.46962738037109
    fy = 205.46962738037109
    cx = 320
    cy = 240
    tx = 0.0
    ty = 0.0
    s = 0.0

    # projection matrix
    P = [[fx, s, cx, tx], [0, fy, cy, ty], [0, 0, 1, 0]]
    camera_matrix = np.array(P)

    image_point = project_point(camera_matrix, camera_pose, target_pose)

    print(f"image_point: {image_point}")


if __name__ == "__main__":
    main()
    # test_project_to_screen_space()
