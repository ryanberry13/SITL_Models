"""
Simple OpenCV object tracker

References
- https://ardupilot.org/dev/docs/mavlink-camera.html#mavlink-camera
"""

import copy
import cv2
import time

from pymavlink import mavutil

ONBOARD_CONTROLLER_SYSID = 245
ONBOARD_CONTROLLER_COMPID = 0
IP = "127.0.0.1"
PORT = 14550


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


def send_cmd_camera_track_rectangle(connection):
    """Send a camera track rectangle command."""
    pass


def send_cmd_camera_stop_tracking(connection):
    """Send a camera stop tracking command."""
    pass


def send_gimbal_manager_pitch_yaw_angles(connection):
    """Send pitch and yaw angles to the gimbal manager."""
    pass


def main():
    # Connect to MAVLink
    connection = connect_to_mavlink(IP, PORT)

    # Send camera information
    send_camera_information(connection)

    # GCS -> FC
    # MAV_CMD_CAMERA_TRACK_RECTANGLE
    # AP_Camera::handle_command
    #   AP_Camera::set_tracking
    #     AP_Camera_Backend::set_tracking
    #       AP_Camera_Backend::set_tracking_external
    #         AP_Camera_Tracking::set_tracking
    #           GCS_MAVLINK::send_message
    #             MAVLINK_MSG_ID_COMMAND_LONG
    #               MAV_CMD_CAMERA_TRACK_RECTANGLE
    #               params1 = x1
    #               params2 = y1
    #               params3 = x2
    #               params4 = y2

    # FC -> Companion
    # MAV_CMD_CAMERA_TRACK_RECTANGLE
    # 

    # Companion -> FC
    # GIMBAL_MANAGER_SET_PITCHYAW


if __name__ == "__main__":
    main()
