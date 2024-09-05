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

import copy
import cv2
import gi
import numpy as np
import threading
import time
import wx

from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.pose_v_pb2 import Pose_V

from gz.transport13 import Node

from transforms3d import affines
from transforms3d import euler
from transforms3d import quaternions


gi.require_version("Gst", "1.0")
from gi.repository import Gst


class Video:
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self.latest_frame = self._new_frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = "udpsrc port={}".format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = (
            "! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264"
        )
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = (
            "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert"
        )
        # Create a sink to get data
        self.video_sink_conf = (
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
        )

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = [
                "videotestsrc ! decodebin",
                "! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert",
                "! appsink",
            ]

        command = " ".join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name("appsink0")

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (caps_structure.get_value("height"), caps_structure.get_value("width"), 3),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8,
        )
        return array

    def frame(self):
        """Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """Get frame to update _new_frame"""

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf,
            ]
        )

        self.video_sink.connect("new-sample", self.callback)

    def callback(self, sink):
        sample = sink.emit("pull-sample")
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK


class CameraPanel(wx.Panel):
    def __init__(self, parent, video, camera_target_tracker, fps=30):
        wx.Panel.__init__(self, parent)

        self._video = video
        self._camera_target_tracker = camera_target_tracker

        # Shared between threads
        self._frame_lock = threading.Lock()
        self._latest_frame = None

        print("Waiting for video stream...")
        waited = 0
        while not self._video.frame_available():
            waited += 1
            print("\r  Frame not available (x{})".format(waited), end="")
            cv2.waitKey(30)
        print("\nSuccess! Video stream available")

        if self._video.frame_available():
            # Only retrieve and display a frame if it's new
            frame = copy.deepcopy(self._video.frame())

            # Frame size
            height, width, _ = frame.shape

            parent.SetSize((width, height))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            self.bmp = wx.Bitmap.FromBuffer(width, height, frame)

            self.timer = wx.Timer(self)
            self.timer.Start(int(1000 / fps))

            self.Bind(wx.EVT_PAINT, self.OnPaint)
            self.Bind(wx.EVT_TIMER, self.NextFrame)

    def OnPaint(self, evt):
        dc = wx.BufferedPaintDC(self)
        dc.DrawBitmap(self.bmp, 0, 0)

    def NextFrame(self, event):
        if self._video.frame_available():
            frame = copy.deepcopy(self._video.frame())

            # create 50x50 px green square about image point
            image_point = self._camera_target_tracker.image_point()
            u = int(image_point[0])
            v = int(image_point[1])
            x1 = u - 25
            y1 = v - 25
            x2 = u + 25
            y2 = v + 25
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

            # Convert frame to bitmap for wxFrame
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.bmp.CopyFromBuffer(frame)
            self.Refresh()


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


class CameraTargetTracker:
    """
    Calculate the position in pixels of a tracked object in the camera image.
    """

    def __init__(
        self,
        world_name="playpen",
        target_model_name="omni4rover",
        camera_model_name="mount",
        gimbal_model_name="gimbal",
        camera_link_name="pitch_link",
        camera_sensor_name="camera",
    ):
        self._world_name = world_name
        self._target_model_name = target_model_name
        self._camera_model_name = camera_model_name
        self._gimbal_model_name = gimbal_model_name
        self._camera_link_name = camera_link_name
        self._camera_sensor_name = camera_sensor_name

        # derive frame_ids
        self._target_frame_id = self._target_model_name

        self._camera_frame_id = (
            f"{self._camera_model_name}"
            + f"::{self._gimbal_model_name}"
            + f"::{self._camera_link_name}"
            + f"::{self._camera_sensor_name}"
        )

        self._node = Node()

        # mutex for access to messages in subsciber callbacks
        self._mutex = threading.Lock()

        # subscribe to pose_v messages
        self._target_pose_v_do_print_msg = False
        self._target_pose_v_msg = None
        self._target_pose_v_topic = f"/model/{self._target_model_name}/pose"
        self._target_pose_v_sub = self._node.subscribe(
            Pose_V, self._target_pose_v_topic, self.target_pose_v_cb
        )
        self._target_pose = None

        self._camera_pose_v_do_print_msg = False
        self._camera_pose_v_msg = None
        self._camera_pose_v_topic = f"/model/{self._camera_model_name}/pose"
        self._camera_pose_v_sub = self._node.subscribe(
            Pose_V, self._camera_pose_v_topic, self.camera_pose_v_cb
        )
        self._camera_pose = None

        # subscribe to camera_info messages
        self._camera_info_do_print_msg = False
        self._camera_info_msg = None
        self._camera_info_topic = (
            f"/world/{self._world_name}"
            + f"/model/{self._camera_model_name}"
            + f"/model/{self._gimbal_model_name}"
            + f"/link/{self._camera_link_name}"
            + f"/sensor/{self._camera_sensor_name}"
            + f"/camera_info"
        )
        self._camera_info_sub = self._node.subscribe(
            CameraInfo, self._camera_info_topic, self.camera_info_cb
        )
        self._camera_info = None

        # image point
        self._image_point = [0, 0]

        # create and start the update thread
        self._update_thread = threading.Thread(target=self.update)
        self._update_thread.start()

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
        update_rate = 10.0
        update_period = 1.0 / update_rate
        while True:
            if self._camera_pose_v_msg is not None:
                world = self._world_name
                body = self._camera_frame_id
                self._camera_pose = self.body_to_world(
                    self._camera_pose_v_msg, world, body
                )
                # print(self._camera_pose)
                # print()

            if self._target_pose_v_msg is not None:
                world = self._world_name
                body = self._target_frame_id
                self._target_pose = self.body_to_world(
                    self._target_pose_v_msg, world, body
                )
                # print(self._target_pose)
                # print()

            if (
                (self._target_pose is not None)
                and (self._camera_pose is not None)
                and (self._camera_info_msg is not None)
            ):
                self._camera_info = CameraInformation(self._camera_info_msg)
                # print(self._camera_info)

                # projection matrix (=opencv camera_matrix)
                camera_matrix = self._camera_info.camera_matrix
                self._image_point = CameraTargetTracker.project_point(
                    camera_matrix, self._camera_pose, self._target_pose
                )

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
            X_p_c = np.array(self.pose_to_affine(pose))
            X = np.matmul(X_p_c, X)

        # print(f"--> {parent}")

        t, r, z, s = affines.decompose(X)
        q = quaternions.mat2quat(r)
        pose = Pose(world, body, t, q)
        return pose

    def pose_to_affine(self, pose):
        """
        Convert a pose message into an transforms3d.affines
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
        return a

    def image_point(self):
        return self._image_point

    @staticmethod
    def project_point(camera_matrix, camera_pose, target_pose):
        """
        Given a camera matrix (camera intrinsics), a camera pose
        and a target pose in the world (ENU) frame, calculate the pixel
        position of the target in the camera image

        From gz-msgs/src/gz/msgs/proto/camera_info.proto

        - fx is the X Focal length
        - fy is the Y Focal length
        - cx is the X principal point
        - cy is the Y principal point
        - tx is the X position of the second camera in this camera's frame.
        - ty is the Y position of the second camera in this camera's frame.
        - s is the axis skew.

        Intrinsic camera matrix for the raw (distorted) images can be
        generated using the fx, fy, cx, and cy parameters contained in this
        message. For example the intrinsic camera matrix K would be:
              [fx  s cx]
          K = [ 0 fy cy]
              [ 0  0  1]
        Projects 3D points in the camera coordinate frame to 2D pixel
        coordinates using the focal lengths (fx, fy) and principal point
        (cx, cy).

        The projection/camera matrix can be generated using the values in
        this message. For example, the projection matrix P would be:

              [fx   s cx tx]
          P = [ 0  fy cy ty]
              [ 0   0  1  0]
        """
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

        # transform the origin of the target to the camera sensor frame
        v_b_b = np.transpose([[0, 0, 0, 1]])
        v_b_c = np.matmul(X_c_b, v_b_b)
        # print(v_b_b)
        # print(v_b_c)

        # rotate from the camera sensor frame to the camera optical frame
        #   camera sensor frame: FRU
        #   camera optical frame: RDF
        t = [0.0, 0.0, 0.0]
        q = euler.euler2quat(np.radians(-90.0), 0.0, np.radians(-90.0))
        r = quaternions.quat2mat(q)
        z = [1.0, 1.0, 1.0]
        X_c_o = affines.compose(t, r, z)
        X_o_c = np.linalg.inv(X_c_o)
        v_b_o = np.matmul(X_o_c, v_b_c)

        # project the target to screen space
        uvw = np.matmul(P, v_b_o)
        u = uvw[0, 0] / uvw[2, 0]
        v = uvw[1, 0] / uvw[2, 0]
        # print(f"XYZ: [{v_b_c[0, 0]:.2f}, {v_b_c[1, 0]:.2f}, {v_b_c[2, 0]:.2f}]")
        # print(f"xyz: [{v_b_o[0, 0]:.2f}, {v_b_o[1, 0]:.2f}, {v_b_o[2, 0]:.2f}]")
        # print(f"uv:  [{u:.2f}, {v:.2f}]")

        # # same calculation using opencv
        # # https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#projectpoints
        # object_points = v_b_o[0:3, 0]
        #
        # # set rvec, tvec to zero as object is already in the camera optical frame
        # rvec = np.array([0.0, 0.0, 0.0])
        # tvec = np.array([0.0, 0.0, 0.0])
        #
        # # camera_matrix is the top-left 3x3 submatrix of P
        # camera_matrix = P[0:3, 0:3]
        # dist_coeffs = np.array([])
        # image_points, _ = cv2.projectPoints(
        #     object_points, rvec, tvec, camera_matrix, dist_coeffs
        # )

        return [u, v]


def main():
    # create an object tracker
    camera_target_tracker = CameraTargetTracker()

    # create the video object
    video = Video(port=5600)

    # app must run on the main thread
    app = wx.App()
    wx_frame = wx.Frame(None)

    # create the camera panel
    camera_panel = CameraPanel(wx_frame, video, camera_target_tracker, fps=30)

    wx_frame.Show()
    app.MainLoop()


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

    image_point = CameraTargetTracker.project_point(
        camera_matrix, camera_pose, target_pose
    )
    print(f"image_point: {image_point}")


if __name__ == "__main__":
    main()
    # test_project_to_screen_space()