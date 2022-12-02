import numpy as np
from typing import Tuple # Used for the depth map stuff

from math import atan2, cos, pi

import libpomp
import telemetry

import cam_controller.frame_of_reference_pb2 as cam_for_pb2
import cam_controller.control_mode_pb2 as cam_cm_pb2

import guidance.core as gdnc_core

import parrot.missions.samples.hello.airsdk.messages_pb2 as AirsdkMessages

import samples.hello.guidance.messages_pb2 as HelloGroundModeMessages

from arsdk import camera2_pb2 as pbuf_camera2

CONFIG_SUFFIX = "/" + HelloGroundModeMessages.Config.DESCRIPTOR.full_name

SENS_DEPTH = 1
SENS_PX_WIDTH = 2 * np.arctan(110 / 2) / 176
SENS_PX_HEIGHT = 2 * np.arctan(72 / 2) / 90

SENS_DEPTH_SQ = SENS_DEPTH * SENS_DEPTH
SENS_PX_WIDTH_SQ = SENS_PX_WIDTH * SENS_PX_WIDTH
SENS_PX_HEIGHT_SQ = SENS_PX_HEIGHT * SENS_PX_HEIGHT


zs = np.array(
    [
        [SENS_DEPTH / np.sqrt((w+0.5)*(w+0.5)*SENS_PX_WIDTH_SQ + (h+0.5)*(h+0.5)*SENS_PX_HEIGHT_SQ + SENS_DEPTH_SQ) for h in range(-45, 45) ]
        for w in range(-88, 88)
    ]
)

xs = np.array(
    [
        [(w+0.5)*SENS_PX_WIDTH / np.sqrt((w+0.5)*(w+0.5)*SENS_PX_WIDTH_SQ + (h+0.5)*(h+0.5)*SENS_PX_HEIGHT_SQ + SENS_DEPTH_SQ) for h in range(-45, 45) ]
        for w in range(-88, 88)
    ]
)

ys = np.array(
    [
        [(h+0.5)*SENS_PX_HEIGHT / np.sqrt((w+0.5)*(w+0.5)*SENS_PX_WIDTH_SQ + (h+0.5)*(h+0.5)*SENS_PX_HEIGHT_SQ + SENS_DEPTH_SQ) for h in range(-45, 45) ]
        for w in range(-88, 88)
    ]
)

# Checks for obstacles within several prisms relative to the camera
def depth_check_prisms(frame: np.ndarray, *prisms: Tuple[float, float, float, float, float, float]) -> Tuple[bool, ...]:
    """
    Checks whether there is a point in the depth map inside several prims, and returns a tuple of bools in
    the same order as the prisms were provided.

    :param frame: The depth cam frame to operate on
    :param prisms: The prisms to check, in (minX, maxX, minY, maxY, minZ, maxZ) order.
    """
    # Could be slightly optimised by batching all prisms for a particular frame
    cam_rel_x = xs * frame
    cam_rel_y = ys * frame
    cam_rel_z = zs * frame
    return tuple((
        np.any((cam_rel_x > prism[0]) & (cam_rel_x < prism[1])) \
        and np.any((cam_rel_y > prism[2]) & (cam_rel_y < prism[3])) \
        and np.any((cam_rel_z > prism[4]) & (cam_rel_z < prism[5]))
        for prism in prisms
    ))


class numpy_holder(object):
    pass


class GoToMode(gdnc_core.Mode):
    def __init__(self, guidance, name):
        super().__init__(guidance, name)
        self.log.error("SiteSee Go To Guidance mode init")
        self.loop = self.guidance.get_loop()
        self.msghub = self.guidance.get_message_hub()

        self.tlm_dctl = None

        self.channel = self.guidance.get_channel(gdnc_core.ChannelKind.GUIDANCE)
        self.evt_sender = gdnc_core.MessageSender(HelloGroundModeMessages.Event.DESCRIPTOR.full_name)
        self.instructions = None


    def shutdown(self):
        self.log.error("SiteSee Go To Guidance shutdown")
        self.loop = None
        self.msghub = None
        self.tlm_dctl = None
        self.evt_sender = None

    def get_triggers(self):
        return (gdnc_core.Trigger.TIMER, 30, 30)

    def configure(self, msg, disable_oa, override_fcam, override_stereo):
        self.log.error("SiteSee Go To Guidance configure msg=%s", msg)
        # if not msg.type_url.endswith(CONFIG_SUFFIX):
        #     raise ValueError("Go To: unexpected config: %s" % msg.type_url)

        config_message = HelloGroundModeMessages.GoTo_Guidance()
        msg.Unpack(config_message)
        
        self.instructions = {
            'x': config_message.x,
            'y': config_message.y,
            'z': config_message.z,
            'yaw': config_message.yaw,
            'speed': config_message.speed
        }

        self.log.error("SiteSee Go To Guidance configure instructions=%s", self.instructions)

        #self.step_count = 0

        subset = ['position_local.x',
                  'position_local.y',
                  'position_local.z',
                  'linear_velocity_global.x',
                  'linear_velocity_global.y',
                  'linear_velocity_global.z',
                  'attitude_euler_angles.yaw',
                  'attitude_euler_angles.pitch',
                  'attitude_euler_angles.roll',
                  'position_geo.latitude_north_deg',
                  'position_geo.longitude_east_deg'
        ]

        self.tlm_dctl = telemetry.TlmSection(
            "/dev/shm", "drone_controller", subset=subset
        )

        self.tlm_dctl.fetch_sample()

        drone_pos = np.array((self.tlm_dctl['position_local.x'], self.tlm_dctl['position_local.y']))
        # self.log.error("SiteSee Go To guidance position configure: Local_X:%0.6f, Local_Y:%0.6f, Local_Z:%0.6f, Local_Yaw:%0.6f", drone_pos[0], drone_pos[1])
        self.log.error("SiteSee Go To guidance position configure: Local_X:%0.6f, Local_Y:%0.6f", drone_pos[0], drone_pos[1])

        # # This section vs line 106 
        # ascend_mode_msg = HelloGroundModeMessages.Config()
        # msg.Unpack(ascend_mode_msg)
        # self.my_guidance_config = ascend_mode_msg.my_guidance_config

        # if self.instructions:
        #     self.log.error("SiteSee Guidance configure my_guidance_config") 

        # Camera Set Up
        self.output.has_front_cam_config = True
        self.output.front_cam_config.yaw.locked = True
        self.output.front_cam_config.yaw.filtered = False
        self.output.front_cam_config.roll.locked = True
        self.output.front_cam_config.roll.filtered = False
        self.output.front_cam_config.pitch.locked = True
        self.output.front_cam_config.pitch.filtered = False

    def enter(self):
        self.log.error("SiteSee Go To Guidance mode Enter")
        self.msghub.attach_message_sender(self.evt_sender, self.channel)

    def exit(self):
        self.msghub.detach_message_sender(self.evt_sender)

    def begin_step(self):
        self.tlm_dctl.fetch_sample()

        drone_lat = self.tlm_dctl['position_geo.latitude_north_deg']
        drone_lon = self.tlm_dctl['position_geo.longitude_east_deg']
        drone_pos_x = self.tlm_dctl['position_local.x']
        drone_pos_y = self.tlm_dctl['position_local.y']
        drone_height = self.tlm_dctl['position_local.z']
        drone_yaw = self.tlm_dctl['attitude_euler_angles.yaw']
        self.log.error("SiteSee Go To guidance position: X Position:%0.2f , Y Position:%0.2f, Height:%0.2f, Lat:%0.6f, Lon:%0.6f, Yaw:%0.3f", drone_pos_x, drone_pos_y, drone_height, drone_lat, drone_lon, drone_yaw)


    def end_step(self):
        pass

    def generate_attitude_reference(self):
        self.log.error("SiteSee Ascend Tower mode generate_attitude_reference")
        # Front
        self.output.has_front_cam_reference = True
        fcam_ref = self.output.front_cam_reference

        fcam_ref.yaw.ctrl_mode = cam_cm_pb2.POSITION
        fcam_ref.yaw.frame_of_ref = cam_for_pb2.NED
        fcam_ref.yaw.position = self.tlm_dctl["attitude_euler_angles.yaw"]

        fcam_ref.pitch.ctrl_mode = cam_cm_pb2.POSITION
        fcam_ref.pitch.frame_of_ref = cam_for_pb2.NED
        fcam_ref.pitch.position = (0.0 * np.pi / 180.0)

        fcam_ref.roll.ctrl_mode = cam_cm_pb2.POSITION
        fcam_ref.roll.frame_of_ref = cam_for_pb2.NED
        fcam_ref.roll.position = 0.0

        # Stereo
        self.output.has_stereo_cam_reference = True
        stcam_ref = self.output.stereo_cam_reference

        stcam_ref.yaw.ctrl_mode = cam_cm_pb2.POSITION
        stcam_ref.yaw.frame_of_ref = cam_for_pb2.NED
        stcam_ref.yaw.position = self.tlm_dctl["attitude_euler_angles.yaw"]

        stcam_ref.pitch.ctrl_mode = cam_cm_pb2.POSITION
        stcam_ref.pitch.frame_of_ref = cam_for_pb2.NED
        stcam_ref.pitch.position = self.tlm_dctl["attitude_euler_angles.pitch"]

        stcam_ref.roll.ctrl_mode = cam_cm_pb2.POSITION
        stcam_ref.roll.frame_of_ref = cam_for_pb2.NED
        stcam_ref.roll.position = self.tlm_dctl["attitude_euler_angles.roll"]


    def generate_drone_reference(self):
        horz_ref = self.output.horizontal_reference
        vert_ref = self.output.vertical_reference
        yaw_ref = self.output.yaw_reference

        self.output.has_horizontal_reference = True
        self.output.has_vertical_reference = True
        self.output.has_yaw_reference = True

        # horz_ref.trajectory_local.ref.x.x = self.my_guidance_config['x']
        horz_ref.trajectory_local.ref.x.x = self.instructions['x']
        horz_ref.trajectory_local.ref.y.x = self.instructions['y']
        vert_ref.trajectory.ref.x = self.instructions['z']
        # Yaw rate
        # yaw_ref.rate.ref = self.instructions['yaw']
        
        # To specified yaw angle
        yaw_ref.trajectory_ned.ref.x = self.instructions['yaw']
        yaw_ref.trajectory_ned.use_shortest_path = True

        drone_x = self.tlm_dctl['position_local.x']
        drone_y = self.tlm_dctl['position_local.y']
        drone_z = self.tlm_dctl['position_local.z']
        drone_yaw = self.tlm_dctl['attitude_euler_angles.yaw']

        # check if drone is within 0.5m of target position and within 0.05 radians of target yaw
        if (abs(drone_x - self.instructions['x']) < 0.5) and (abs(drone_y - self.instructions['y']) < 0.05) and (abs(drone_z - self.instructions['z']) < 0.05) and (abs(drone_yaw - self.instructions['yaw']) < 0.05):
            self.log.error("SiteSee Go To guidance position: X Position:%0.2f , Y Position:%0.2f, Height:%0.2f , Yaw Orientation:%0.6f ", drone_x, drone_y, drone_z , drone_yaw)
            self.log.error("SiteSee Go To guidance position: X Target:%0.2f , Y Target:%0.2f, Height Target:%0.2f, Yaw Target:%0.6f", self.instructions['x'], self.instructions['y'], self.instructions['z'] , self.instructions['yaw'])
            self.log.error("SiteSee Go To guidance position: X Error:%0.2f , Y Error:%0.2f, Height Error:%0.2f , Yaw Error:%0.6f ", abs(drone_x - self.instructions['x']), abs(drone_y - self.instructions['y']), abs(drone_z - self.instructions['z']) , abs(drone_yaw - self.instructions['yaw']))
            self.log.error("SiteSee Go To guidance position: Reached Target")
            self.log.error("SiteSee Go To guidance position: Sending Go To Complete")
            msg_event = HelloGroundModeMessages.Event() 
            msg_event.goto_complete.SetInParent()
            gdnc_core.msghub_send(self.evt_sender , msg_event)


        # if self.yaw is not None:
        #     yaw_ref = self.output.yaw_reference
        #     self.output.has_yaw_reference = True

        #     yaw_ref.trajectory_ned.ref.x = self.yaw
        #     yaw_ref.trajectory_ned.use_shortest_path = True

        #     current_yaw = self.tlm_dctl['attitude_euler_angles.yaw']
        #     yaw_err = self.yaw - current_yaw
        #     if yaw_err > np.pi:
        #         yaw_err -= 2 * np.pi
        #     elif yaw_err < -np.pi:
        #         yaw_err += 2 * np.pi

        # self.done = ((drone_x - self.x) ** 2 + (drone_y - self.y) ** 2 + (drone_z - self.z) ** 2) ** .5 < (self.yaw is None or abs(yaw_err) < 0.05)


    def correct_drone_reference(self):
        pass

    def generate_camera_reference(self):
        pass

    def correct_camera_reference(self):
        pass

GUIDANCE_MODES = {"com.parrot.missions.samples.hello.guidance.GoTo": GoToMode}