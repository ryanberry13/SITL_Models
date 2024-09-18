"""
Simple OpenCV object tracker using wxWidgets

Acknowledgments
---------------

Video class to capture GStreamer frames
  https://www.ardusub.com/developers/opencv.html

ImagePanel class to display openCV images in wxWidgets
  https://stackoverflow.com/questions/14804741/opencv-integration-with-wxpython


MAVLink Camera Commands
  https://ardupilot.org/dev/docs/mavlink-camera.html#mavlink-camera
"""

import copy
import cv2
import gi
import numpy as np
import threading
import time
import wx

from pymavlink import mavutil
from ultralytics import YOLO

gi.require_version("Gst", "1.0")
from gi.repository import Gst

ONBOARD_CONTROLLER_SYSID = 245
ONBOARD_CONTROLLER_COMPID = 0
IP = "127.0.0.1"
PORT = 14550

# Globals
connection = None

# GStreamer video capture


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


# Wx camera panel


class CameraPanel(wx.Panel):
    def __init__(self, parent, video, fps=30):
        wx.Panel.__init__(self, parent)

        self._video = video

        # Shared between threads
        self._frame_lock = threading.Lock()
        self._latest_frame = None

        print("Initialising stream...")
        waited = 0
        while not self._video.frame_available():
            waited += 1
            print("\r  Frame not available (x{})".format(waited), end="")
            cv2.waitKey(30)
        print("\nSuccess!\nStarting streaming")

        # Load a pretrained YOLO model
        self._model = YOLO("yolov8n.pt")

        # Display model information
        self._model.info()

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

            # Object detection
            results = self._model.track(frame, stream=True)

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # bounding box
                    cls = int(box.cls)
                    name = self._model.names[cls]
                    # if not (
                    #     name == "bus"
                    #     or name == "boat"
                    #     or name == "skateboard"
                    #     or name == "truck"
                    # ):
                    #     continue

                    print("Classification is: ", name, ", [", box.conf, "]")

                    # Draw bounding rectangle
                    x1, y1, x2, y2 = box.xyxy[0]

                    # Convert to int values
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

            # Convert frame to bitmap for wxFrame
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.bmp.CopyFromBuffer(frame)
            self.Refresh()


# MAVLink commands


# GCS -> FC
def connect_to_mavlink(ip, port):
    """Establish a MAVLink connection."""
    connection = mavutil.mavlink_connection(
        f"udp:{ip}:{port}", source_system=ONBOARD_CONTROLLER_SYSID
    )
    connection.wait_heartbeat()

    print(
        "Heartbeat received: system {}: component {}".format(
            connection.target_system, connection.target_component
        )
    )

    # Send a heartbeat back to register the device
    connection.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=mavutil.mavlink.MAV_STATE_UNINIT,
        mavlink_version=3,
    )
    return connection


# GCS -> FC
def send_camera_information(connection):
    """Send the camera information."""
    # Current time in milliseconds since boot
    time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF

    # Dummy data for camera information
    vendor_name = b"Gazebo" + b"\0" * (32 - len("Gazebo"))
    model_name = b"RGB" + b"\0" * (32 - len("RGB"))
    # Example: version 1.0.0.1
    firmware_version = (1 << 24) | (0 << 16) | (0 << 8) | 1
    focal_length = float("nan")  # Not known
    sensor_size_h = float("nan")  # Not known
    sensor_size_v = float("nan")  # Not known
    resolution_h = 640  # Example resolution
    resolution_v = 480  # Example resolution
    lens_id = 0  # Not known
    flags = 4095  # All capabilities
    cam_definition_version = 0  # Not known
    cam_definition_uri = b""  # Not known
    gimbal_device_id = 0  # Not known

    # Send camera information
    connection.mav.camera_information_send(
        time_boot_ms,
        vendor_name,
        model_name,
        firmware_version,
        focal_length,
        sensor_size_h,
        sensor_size_v,
        resolution_h,
        resolution_v,
        lens_id,
        flags,
        cam_definition_version,
        cam_definition_uri,
        gimbal_device_id,
    )
    print("Sent camera information")


# GCS -> FC
def send_cmd_camera_track_rectangle(connection):
    """Send a camera track rectangle command."""
    pass


# GCS -> FC
def send_cmd_camera_stop_tracking(connection):
    """Send a camera stop tracking command."""
    pass


def handle_camera_track_point(msg):
    print("Received MAV_CMD_CAMERA_TRACK_POINT")
    x1 = msg.param1
    y1 = msg.param2
    print("Tracking point ({}, {}): ".format(x1, y1))


def handle_camera_track_rectangle(msg):
    print("Received MAV_CMD_CAMERA_TRACK_RECTANGLE")
    x1 = msg.param1
    y1 = msg.param2
    x2 = msg.param1
    y2 = msg.param2
    print("Tracking rectangle ({}, {}), ({}, {}): ".format(x1, y1, x2, y2))


# Companion -> FC
def send_gimbal_manager_pitch_yaw_angles(connection):
    """Send pitch and yaw angles to the gimbal manager."""
    pass


def control_gimbal():
    global connection

    print("Started gimbal control thread")

    while True:
        msg = connection.recv_match(type="COMMAND_LONG", blocking=False)
        if not (msg and msg.get_type() == "COMMAND_LONG"):
            continue

        if not (msg.target_system == ONBOARD_CONTROLLER_SYSID):
            print("Ignoring camera command for system: {}".format(msg.target_system))
            continue

        if msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT:
            handle_camera_track_point(msg)
        elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE:
            handle_camera_track_rectangle(msg)


def main():
    global connection

    # Establish mavlink connection
    connection = connect_to_mavlink(IP, PORT)

    # Start threads
    control_thread = threading.Thread(target=control_gimbal, name="control")
    control_thread.start()

    # Send camera information and start tracking
    send_camera_information(connection)
    send_cmd_camera_track_rectangle(connection)

    # GCS -> FC: MAV_CMD_CAMERA_TRACK_RECTANGLE
    #
    # AP_Camera::handle_command
    #   AP_Camera::set_tracking
    #     AP_Camera_Backend::set_tracking
    #       AP_Camera_Backend::set_tracking_external
    #
    # FC -> Companion: MAV_CMD_CAMERA_TRACK_RECTANGLE
    #
    #         AP_Camera_Tracking::set_tracking
    #           GCS_MAVLINK::send_message
    #             MAVLINK_MSG_ID_COMMAND_LONG
    #               MAV_CMD_CAMERA_TRACK_RECTANGLE
    #               params1 = x1
    #               params2 = y1
    #               params3 = x2
    #               params4 = y2
    #
    # Companion -> FC: GIMBAL_MANAGER_SET_PITCHYAW

    # App must run on the main thread
    app = wx.App()
    wx_frame = wx.Frame(None)

    # Create the video and camera panel
    video = Video(port=5600)
    # camera_panel = CameraPanel(wx_frame, video, fps=30)

    wx_frame.Show()
    app.MainLoop()


if __name__ == "__main__":
    main()
