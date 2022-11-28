import numpy as np
from math import atan2, cos, pi

import libpomp
import telemetry

import cam_controller.frame_of_reference_pb2 as cam_for_pb2
import cam_controller.control_mode_pb2 as cam_cm_pb2

import guidance.core as gdnc_core

import parrot.missions.samples.hello.airsdk.messages_pb2 as AirsdkMessages

import samples.hello.guidance.messages_pb2 as HelloGroundModeMessages

from arsdk import camera2_pb2 as pbuf_camera2
# from fsup.message_center.airsdk_channel import AirSdkChannel

CONFIG_SUFFIX = "/" + HelloGroundModeMessages.Config.DESCRIPTOR.full_name

class FindTowerMode(gdnc_core.Mode):

    def __init__(self, guidance, name):
        super().__init__(guidance, name)
        self.loop = self.guidance.get_loop()
        self.msghub = self.guidance.get_message_hub()

        self.tlm_dctl = None

        self.channel = self.guidance.get_channel(gdnc_core.ChannelKind.GUIDANCE)
        self.evt_sender = gdnc_core.MessageSender(HelloGroundModeMessages.Event.DESCRIPTOR.full_name)

        # self.airchannel = AirsdhChannel('recipient_id', 'autopilot_service')
        self.instructions = None
        self.complete_mission = False

        self.drone_height = 0
        self.drone_x = 0
        self.drone_y = 0
        self.drone_yaw = 0

        self.drone_yaw_rate = 0
        self.drone_vertical_goal = 0
        self.start_drone_yaw = 0

    def shutdown(self):
        self.log.error("SiteSee Find Tower Guidance shutdown")
        self.loop = None
        self.msghub = None
        self.tlm_dctl = None
        self.evt_sender = None

    def get_triggers(self):
        return (gdnc_core.Trigger.TIMER, 30, 30)

    def configure(self, msg, disable_oa, override_fcam, override_stereo):
        self.log.error("SiteSee Find Tower Guidance configure msg=%s", msg)
        # if not msg.type_url.endswith(CONFIG_SUFFIX):
        #     raise ValueError("Flying: unexpected config: %s" % msg.type_url)

        config_message = HelloGroundModeMessages.FindTowerParameters_Guidance()
        msg.Unpack(config_message)
        
        self.instructions = {
            'x': config_message.x,
            'y': config_message.y,
            'visible': config_message.visible
        }

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

        self.drone_height = self.tlm_dctl['position_local.z']
        self.drone_x = self.tlm_dctl['position_local.x']
        self.drone_y = self.tlm_dctl['position_local.y']
        self.start_drone_yaw = self.tlm_dctl['attitude_euler_angles.yaw']
        self.drone_yaw = self.start_drone_yaw
        self.drone_vertical_goal = self.drone_height
        self.log.error("SiteSee Find Tower guidance position: Local_X:%0.6f, Local_Y:%0.6f", self.drone_x, self.drone_y)
        self.log.error("SiteSee Find Tower guidance position: Instructions X:%0.3f, Instructions Y:%0.3f, Instructions Visible:%d", self.instructions['x'], self.instructions['y'], self.instructions['visible'])

        
        self.log.error("SiteSee Rotate Tower guidance initial Yaw:%0.3f, Height:%0.3f", self.start_drone_yaw, self.drone_height)

        self.output.has_front_cam_config = True
        self.output.front_cam_config.yaw.locked = True
        self.output.front_cam_config.yaw.filtered = False
        self.output.front_cam_config.roll.locked = True
        self.output.front_cam_config.roll.filtered = False
        self.output.front_cam_config.pitch.locked = True
        self.output.front_cam_config.pitch.filtered = False


    def enter(self):
        self.log.error("SiteSee Find Tower Guidance mode Enter")
        self.msghub.attach_message_sender(self.evt_sender, self.channel)

        self.flying_state = 3 # Rotate State
        self.drone_yaw_rate = 0.25


    def exit(self):
        self.msghub.detach_message_sender(self.evt_sender)

    def begin_step(self):
        self.tlm_dctl.fetch_sample()

        self.drone_x = self.tlm_dctl['position_local.x']
        self.drone_y = self.tlm_dctl['position_local.y']
        self.drone_height = self.tlm_dctl['position_local.z']
        self.drone_yaw = self.tlm_dctl['attitude_euler_angles.yaw']

        drone_lat = self.tlm_dctl['position_geo.latitude_north_deg']
        drone_lon = self.tlm_dctl['position_geo.longitude_east_deg']
        
        self.log.error("SiteSee Find Tower guidance position: Hight:%0.2f, Lat:%0.6f, Lon:%0.6f, Yaw:%0.3f", self.drone_height, self.drone_x, self.drone_y, self.drone_yaw)

        if self.drone_x == self.instructions['x'] and self.drone_y == self.instructions['y']:
            self.complete_mission = True
            msg = HelloGroundModeMessages.Event()
            msg.mission_complete.SetInParent()
            gdnc_core.msghub_send(self.evt_sender, msg)

            self.log.warning("\nSiteSee Find Tower guidance Complete\n")

        yaw_diff = ((self.drone_yaw) - self.start_drone_yaw)
        # self.log.warning("Sitesee Rotate Tower Guidance Yaw Diff:%0.4f", yaw_diff)

        #Change the state it is in (Rotation/Rise), increase height when complete in 360degrees rotation
        if self.flying_state == 0: # Rotation
            self.drone_yaw_rate = 0.25
            if (yaw_diff) > -0.025 and (yaw_diff) < 0.025:
                self.flying_state = 1 #Change to Rise

        elif self.flying_state == 1: # Rise
            self.drone_yaw_rate = 0
            self.drone_vertical_goal = self.drone_height - 2
            self.flying_state = 3
        elif self.flying_state == 3:
            #DO Nothing until reach height
            if self.drone_height <= self.drone_vertical_goal: # Probably change to include a buffer zone (eg: 0.25m)
                self.drone_vertical_goal = self.drone_height
                self.drone_yaw_rate = 0.25
                if abs(yaw_diff) > 0.5:
                    self.flying_state = 0 #Change to Rotation

    def end_step(self):
        pass

    def generate_drone_reference(self):
        self.log.error("SiteSee Find Tower Guidance mode generate_drone_reference")

        if self.instructions['visible'] == True:
            horz_ref = self.output.horizontal_reference
            self.output.has_horizontal_reference = True

            horz_ref.velocity_trajectory.ref.x.x = 1
            horz_ref.velocity_trajectory.ref.y.x = 1
        else:
             # Rotate Drone
            yaw_ref = self.output.yaw_reference
            self.output.has_yaw_reference = True
            yaw_ref.rate.ref = self.drone_yaw_rate

            # Increase Height of Drone
            vert_ref = self.output.vertical_reference
            self.output.has_vertical_reference = True # has_vertical_velocity_target
            vert_ref.trajectory.ref.x =  self.drone_vertical_goal # Move to 2 m above ground
        
        
        # self.log.error("SiteSee Tower mode generate_drone_reference")
        #pass

    def correct_drone_reference(self):
        pass

    def generate_attitude_references(self):
        # self.log.error("SiteSee Find Tower mode generate_attitude_reference")
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

class RotateTowerMode(gdnc_core.Mode):

    def __init__(self, guidance, name):
        super().__init__(guidance, name)
        self.loop = self.guidance.get_loop()
        self.msghub = self.guidance.get_message_hub()

        self.tlm_dctl = None

        self.channel = self.guidance.get_channel(gdnc_core.ChannelKind.GUIDANCE)
        self.evt_sender = gdnc_core.MessageSender(HelloGroundModeMessages.Event.DESCRIPTOR.full_name)

        self.drone_yaw_rate = 0
        self.drone_vertical_pos = 0

        self.start_drone_yaw = 0

    def shutdown(self):
        self.log.error("SiteSee Rotate Tower Guidance shutdown")
        self.loop = None
        self.msghub = None
        self.tlm_dctl = None
        self.evt_sender = None

    def get_triggers(self):
        return (gdnc_core.Trigger.TIMER, 30, 30)

    def configure(self, msg, disable_oa, override_fcam, override_stereo):
        self.log.error("SiteSee Rotate Tower Guidance configure msg=%s", msg)
        if not msg.type_url.endswith(CONFIG_SUFFIX):
            raise ValueError("Flying Rotate: unexpected config: %s" % msg.type_url)

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

        # Get the initial Yaw angle of the drone
        self.start_drone_yaw = self.tlm_dctl['attitude_euler_angles.yaw']
        self.drone_vertical_pos = self.tlm_dctl['position_local.z']
        self.log.error("SiteSee Rotate Tower guidance initial Yaw:%0.3f, Height:%0.3f", self.start_drone_yaw, self.drone_vertical_pos)

        # self.airsdhChannel = AirSdkChannel(self.channel)
        # self.airsdhChannel.start()

        find_mode_msg = HelloGroundModeMessages.Config()
        msg.Unpack(find_mode_msg)
        self.my_guidance_config = find_mode_msg.my_guidance_config


        if self.my_guidance_config:
            self.log.error("SiteSee Guidance configure my_guidance_config")

        self.output.has_front_cam_config = True
        self.output.front_cam_config.yaw.locked = True
        self.output.front_cam_config.yaw.filtered = False
        self.output.front_cam_config.roll.locked = True
        self.output.front_cam_config.roll.filtered = False
        self.output.front_cam_config.pitch.locked = True
        self.output.front_cam_config.pitch.filtered = False


    def enter(self):
        self.log.error("SiteSee Rotate Tower Guidance mode Enter")

        self.flying_state = 3 # Rotate State
        self.drone_yaw_rate = 0.25

        self.msghub.attach_message_sender(self.evt_sender, self.channel)
        

    def exit(self):
        self.msghub.detach_message_sender(self.evt_sender)

    def begin_step(self):
        self.tlm_dctl.fetch_sample()

        self.take_image()

        drone_lat = self.tlm_dctl['position_geo.latitude_north_deg']
        drone_lon = self.tlm_dctl['position_geo.longitude_east_deg']
        drone_height = self.tlm_dctl['position_local.z']
        drone_yaw = self.tlm_dctl['attitude_euler_angles.yaw']
        self.log.error("SiteSee Find Tower guidance position: Hight:%0.2f, Lat:%0.6f, Lon:%0.6f, Yaw:%0.3f", drone_height, drone_lat, drone_lon, drone_yaw)

        yaw_diff = ((drone_yaw) - self.start_drone_yaw)
        self.log.warning("Sitesee Rotate Tower Guidance Yaw Diff:%0.4f", yaw_diff)

        #Change the state it is in (Rotation/Rise), increase height when complete in 360degrees rotation
        if self.flying_state == 0: # Rotation
            self.drone_yaw_rate = 0.25
            if (yaw_diff) > -0.025 and (yaw_diff) < 0.025:
                self.flying_state = 1 #Change to Rise

        elif self.flying_state == 1: # Rise
            self.drone_yaw_rate = 0
            self.drone_vertical_pos = drone_height - 2
            self.flying_state = 3
        elif self.flying_state == 3:
            #DO Nothing until reach height
            if drone_height <= self.drone_vertical_pos: # Probably change to include a buffer zone (eg: 0.25m)
                self.drone_vertical_pos = drone_height
                self.drone_yaw_rate = 0.25
                if abs(yaw_diff) > 0.5:
                    self.flying_state = 0 #Change to Rotation

        self.log.warning("Sitesee Rotate Tower Guidance State:%d", self.flying_state)

    def end_step(self):
        pass

    def generate_drone_reference(self):
        self.log.error("SiteSee Rotate Tower Guidance mode generate_drone_reference")

        # Rotate Drone
        yaw_ref = self.output.yaw_reference
        self.output.has_yaw_reference = True
        yaw_ref.rate.ref = self.drone_yaw_rate

        # Increase Height of Drone
        vert_ref = self.output.vertical_reference
        self.output.has_vertical_reference = True # has_vertical_velocity_target
        vert_ref.trajectory.ref.x =  self.drone_vertical_pos # Move to 2 m above ground
        


    def correct_drone_reference(self):
        pass

    def generate_attitude_references(self):
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
    
    def take_image(self):
        # msg = pbuf_camera2.Event()
        # state = pbuf_camera2.Event.State()
        # clist = pbuf_camera2.Event.CameraList()
        # cPhoto = pbuf_camera2.Event.Photo()
        # self.log.warning("SiteSee Take Image State: %s , \nList: %s \nPhoto:%s", [state], [clist], [cPhoto])
        # self.log.warning("SiteSee Take Image msg: %s ", msg)
        # self.log.warning("SiteSee Take Image msg State: %s ", msg.State())
        # self.airsdhChannel.send_message(msg)
        # find_msg = pbuf_camera2.Event()
        # msg.Unpack(find_mode_msg)

        # msg.Photo = {"camera_id":0, "type":'taking_photo', "media_id":'', "stop_reason":'user_request', "resource_id":''}
        # msg = Photo(camera_id=0, type='start', media_id='', stop_reason='user_request', resource_id='')
        # gdnc_core.msghub_send(self.evt_sender, msg)
        self.log.warning("SiteSee Take Image")


GUIDANCE_MODES = {"com.parrot.missions.samples.hello.RotateTower": RotateTowerMode,
"com.parrot.missions.samples.hello.FindTower": FindTowerMode}
