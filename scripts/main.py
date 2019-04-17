#!/usr/bin/env python
# coding: utf-8

import sys
import json
import os
import yaml
import rospy
import rospkg
from mrobot_srvs.srv import KVPair

path = None 

DOOR_NUM = "door_num"
AUDIO_CHANNEL = "audio_channel"
CONVEYOR_LOCK = "conveyor_lock"
SONAR_TYPE = "sonar_type"

AUDIO_CHANNEL_RK = "rk"
AUDIO_CHANNEL_X86 = "x86"

SONAR_TYPE_M30_8 = "m30_8"
SONAR_TYPE_NOAH_14 = "noah_14"
SONAR_TYPE_NOAH_16 = "noah_16"

settings = {DOOR_NUM: 3, AUDIO_CHANNEL: AUDIO_CHANNEL_RK, CONVEYOR_LOCK: True, SONAR_TYPE: SONAR_TYPE_M30_8}

def check_param_value_valid(param, value):
    if param == DOOR_NUM:
        if isinstance(value, str):
            if value.isdigit():
                if int(value) > 0 and int(value) < 10:
                    return True
        elif isinstance(value, int):
            if value > 0 and value < 10:
                return True
        return False
    elif param == AUDIO_CHANNEL:
        if value == AUDIO_CHANNEL_RK or value == AUDIO_CHANNEL_X86:
            return True
        return False
    elif param == CONVEYOR_LOCK:
        if isinstance(value, str):
            if value == "True" or value == "False":
                return True
        elif isinstance(value, bool):
            return True
        return False
    elif param == SONAR_TYPE:
        if value == SONAR_TYPE_M30_8 or value == SONAR_TYPE_NOAH_14 or value == SONAR_TYPE_NOAH_16:
            return True
        return False
    return False


def set_param_to_server(param, value):
    if param == DOOR_NUM:
        if isinstance(value, str):
            if value.isdigit():
                rospy.set_param(param, int(value))
        elif isinstance(value, int):
            rospy.set_param(param, value)
    elif param == CONVEYOR_LOCK:
        if isinstance(value, str):
            if value == "True":
                rospy.set_param(CONVEYOR_LOCK, True)
            elif value == "False":
                rospy.set_param(CONVEYOR_LOCK, False)
        elif isinstance(value, bool):
            rospy.set_param(CONVEYOR_LOCK, value)
    elif param == AUDIO_CHANNEL:
        rospy.set_param(AUDIO_CHANNEL, value)
    elif param == SONAR_TYPE:
        rospy.set_param(SONAR_TYPE, value)


def file_init():
    global path
    if not os.path.exists(path):
        rospy.logerr("%s: file not exists, create new one", path)
        file_handle = open(path, 'w')
        file_handle.close()


def read_all_params():
    global path
    if not os.path.exists(path):
        rospy.logerr("%s: file not exists", path)
    else:
        param_file = open(path, 'r')
        global settings
        need_write_flag = False
        settings_tmp = yaml.load(param_file)
        if settings is not None:
            if settings_tmp is not None:
                for param in settings:
                    if param in settings_tmp:
                        if check_param_value_valid(param, settings_tmp[param]):
                            if settings[param] != settings_tmp[param]:
                                need_write_flag = True
                                settings[param] = settings_tmp[param]
                        else:
                            rospy.logerr("read from file: param %s value is %s,  error, using default value %s", param, settings_tmp[param], settings[param])
            else:
                need_write_flag = True
            for param in settings:
                set_param_to_server(param, settings[param])
        param_file.close()
        if need_write_flag == True:
            param_file = open(path, 'w')
            rospy.loginfo("get all params: %s", settings)
            yaml.dump(settings, param_file)
            param_file.close()

def write_param(param, value):
    rospy.loginfo("write_param:   param: %s , value: %s", param, value)
    global path
    if check_param_value_valid(param, value):
        if not os.path.exists(path):
            rospy.logerr("%s: file not exists", path)
        else:
            param_file = open(path, 'w')
            global settings
            rospy.loginfo("settings: %s", settings)
            if settings is not None:
                if param in settings:
                    old_value = settings[param]
                    print 'old value: ', old_value, ' new value: ', value
                    if old_value != value:
                        rospy.loginfo("change param %s :  %s -> %s ", param, old_value, value)

                        if param == DOOR_NUM:
                            if value.isdigit():
                                door_num = int(value)
                                settings[DOOR_NUM] = door_num
                            else:
                                rospy.logerr("input door num is not digit  : %s", value)
                        elif param == CONVEYOR_LOCK:
                            lock_ = True
                            if isinstance(value, str):
                                if value == 'True':
                                    lock_ = True
                                elif value == 'False':
                                    lock_ = False
                                settings[CONVEYOR_LOCK] = lock_
                            elif isinstance(value, bool):
                                settings[CONVEYOR_LOCK] = value

                        settings[param] = value
                        yaml.dump(settings, param_file)
                    else:
                        rospy.logwarn("param %s: new value == old_value", param)
                else:
                    rospy.logerr("have no such param %s !", param)
            else:
                rospy.logerr("error: setings is None !")
            param_file.close()
    else:
        rospy.logerr("param error !: param: %s, value: %s", param, value)

def set_param(req):
    rospy.loginfo("set param, req.request: %s", req)
    #setting = json.loads(req)
    if req.key in settings:
        if check_param_value_valid(req.key, req.value):
            write_param(req.key, req.value)
        else:
            rospy.logerr("req param error:   req.key: %s,  req.value: %s", req.key, req.value)
    else:
        rospy.logerr("param %s is not support:", req.key)
        return [True, 'false']
    read_all_params()

    return [True, 'ok']

def main():
    global path
    rospy.init_node("factory_settings", anonymous = True)
    rospy.Service('/factory_settings/set_param', KVPair, set_param)
    path = os.path.join(rospkg.get_ros_home(), "factory_settings.yaml")
    rospy.loginfo("path: %s", path)
    file_init()
    read_all_params()
    rospy.spin()

if __name__  == '__main__':
    try:
        main()
    except Exception:
        rospy.logerr(sys.exc_info())
        rospy.logerr("lost connection")
        exit(1)
