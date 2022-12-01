from fsup.missions.default.flying.stage import FLYING_STAGE as DEF_FLYING_STAGE
from fsup.missions.default.flying.manual import MANUAL_STATE
from fsup.genstate import State, guidance_modes

from msghub_utils import is_msg

import guidance.relative_move_pb2 as rm_pb

from arsdk import camera2_pb2 as pbuf_camera2

import samples.hello.guidance.messages_pb2 as hello_gdnc_mode_msgs
import parrot.missions.samples.hello.airsdk.messages_pb2 as hello_airsdk_mode_msgs

from ..uid import UID

_Ascend_Mode_Name = "com.parrot.missions.Sitesee.Tower.AscendTower"
_Find_Mode_Name = UID + ".FindTower"
_Rotate_Mode_Name = UID + ".RotateTower"
_Go_To_Mode_Name = "com.parrot.missions.samples.hello.guidance.GoTo"

@guidance_modes(_Ascend_Mode_Name)
class AscendTower(State):
    def enter(self, msg):
        self.log.error(f"SiteSee Ascend Tower mode Enter {msg}")
        self.set_guidance_mode(_Ascend_Mode_Name, hello_gdnc_mode_msgs.Config(my_guidance_config=True))

    def step(self, msg):
        # self.log.warning("SiteSee Ascend Tower mode event step")
        if is_msg(msg, hello_gdnc_mode_msgs.Event, "path_obstructed"):
            self.log.info("Sitesee ascent Tower event: msg=%s", msg)
            self.mission.ext_ui_msgs.evt.sender.path_obstructed()
        pass

    def exit(self, msg):
        self.log.error("SiteSee Ascend Tower mode event exit")
        

@guidance_modes(_Find_Mode_Name)
class FindTower(State):
    def enter(self, msg):
        self.log.error(f"SiteSee Find Tower mode Enter {msg}")

        config = hello_airsdk_mode_msgs.FindTowerParameters() 
        config.CopyFrom(msg.start_find_tower) # overwrites the message with the given message's values 
        self.set_guidance_mode(_Find_Mode_Name, hello_gdnc_mode_msgs.FindTowerParameters_Guidance(x=config.x, y=config.y, visible=config.visible))

                # , 'video_recording_mode': pbuf_camera2.PHOTO_MODE_SINGLE
    # 'video_recording_resolution': pbuf_camera2.VIDEO_RESOLUTION_1080P,
    # 'video_recording_framerate': pbuf_camera2.FRAMERATE_120,
    # 'video_recording_dynamic_range': pbuf_camera2.DYNAMIC_RANGE_STANDARD
        self.supervisor.video_manager.start()

    def step(self, msg):

        pass

    def exit(self, msg):
        self.log.error("SiteSee Find Tower mode event exit")

@guidance_modes(_Rotate_Mode_Name)
class RotateTower(State):
    def enter(self, msg):
        self.log.error("SiteSee Rotate Tower mode Enter")
        self.set_guidance_mode(_Rotate_Mode_Name, hello_gdnc_mode_msgs.Config(my_guidance_config=True))

        self.supervisor.video_manager.unlock_config()
        self.supervisor.video_manager.configure(config={'camera_mode': pbuf_camera2.CAMERA_MODE_PHOTO})
        current_config = self.supervisor.video_manager.get_config()
        self.log.error("SiteSee Ascend Tower mode Enter camer config: %s", current_config)

    def step(self, msg):
        pbuf_camera2.Command.StartPhoto()
        pbuf_camera2.Command.StopPhoto()
        # self.supervisor.video_manager.PHOTO_EVENT_START()

        # passi

    def exit(self, msg):
        self.log.error("SiteSee Rotate Tower mode event exit")

@guidance_modes(_Go_To_Mode_Name)
class SiteSeeGoTo(State):
    def enter(self, msg):
        self.log.error("SiteSee GoTo mode Enter")

        config = hello_airsdk_mode_msgs.GoTo() 
        config.CopyFrom(msg.go_to) # overwrites the message with the given message's values 
        self.set_guidance_mode(_Go_To_Mode_Name, hello_gdnc_mode_msgs.GoTo_Guidance(x=config.x , y=config.y, z=config.z, yaw=config.yaw , speed=config.speed)) 
        self.log.error("SiteSee GoTo mode Enter %s", config)

    def step(self, msg):
        # self.log.error("SiteSee GoTo mode step")
        pass

    def exit(self, msg):
        self.log.error("SiteSee GoTo mode event exit")


ASCEND_TOWER_STATE = {"name": "ascend_tower", "class": AscendTower}
FIND_TOWER_STATE = {"name": "find_tower", "class": FindTower}
ROTATE_TOWER_STATE = {"name": "rotate_tower", "class": RotateTower}
GO_TO_STATE = {"name": "sitesee_go_to", "class": SiteSeeGoTo}

FLYING_STAGE = {
    "name": "flying",
    "class": DEF_FLYING_STAGE["class"],
    "initial": "manual",
    "children": [MANUAL_STATE] + [ASCEND_TOWER_STATE, FIND_TOWER_STATE, ROTATE_TOWER_STATE, GO_TO_STATE],
}
