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

SONAR_TYPE_VALUE = (SONAR_TYPE_M30_8, SONAR_TYPE_NOAH_14, SONAR_TYPE_NOAH_16)
AUDIO_CHANNEL_VALUE = (AUDIO_CHANNEL_RK, AUDIO_CHANNEL_X86)

settings = {DOOR_NUM: 3, AUDIO_CHANNEL: AUDIO_CHANNEL_RK, CONVEYOR_LOCK: True, SONAR_TYPE: SONAR_TYPE_M30_8}

'''
return     : [result, changed, param, value]
 result    : is value valid
changed    : is value changed
 param     : input param
 value     : output value
'''
def check_param_value_valid(param, value):
    if param == DOOR_NUM:
        if isinstance(value, str):
            if value.isdigit():
                if int(value) >= 0 and int(value) < 10:
                    return [True, True, DOOR_NUM, int(value)]
        elif isinstance(value, int):
            if value >= 0 and value < 10:
                return [True, False, DOOR_NUM, value]
        return [False, False, DOOR_NUM, value]
    elif param == AUDIO_CHANNEL:
        return [value in AUDIO_CHANNEL_VALUE, False, AUDIO_CHANNEL, value]
    elif param == CONVEYOR_LOCK:
        if isinstance(value, str):
            if value in ['True', 'False', 'true', 'false']:
                return [True, True, CONVEYOR_LOCK, value in ['True', 'true']]
        elif isinstance(value, bool):
            return [True, False, CONVEYOR_LOCK, value]
        return [False, False, CONVEYOR_LOCK, value]
    elif param == SONAR_TYPE:
        return [value in SONAR_TYPE_VALUE, False, SONAR_TYPE, value]
    return [False, False, SONAR_TYPE, value]


def set_param_to_server(param, value):
    rospy.set_param(param, value)


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
                        [result, changed, param, value] = check_param_value_valid(param, settings_tmp[param])
                        if result:
                            settings[param] = value
                        else:
                            rospy.logerr("read from file: param %s value is %s,  error, using default value %s", param, settings_tmp[param], settings[param])
                        if changed:
                            rospy.logwarn("changed")
                            need_write_flag = True
            else:
                need_write_flag = True
            for param in settings:
                set_param_to_server(param, settings[param])
        param_file.close()
        if need_write_flag:
            param_file = open(path, 'w')
            rospy.loginfo("get all params: %s", settings)
            rospy.logwarn("read_all_params: write yaml file, write content: %s", settings)
            yaml.dump(settings, param_file)
            param_file.close()

def write_param(param, value):
    rospy.loginfo("write_param:   param: %s , value: %s", param, value)
    global path
    if not os.path.exists(path):
        rospy.logerr("%s: file not exists", path)
    else:
        global settings
        rospy.loginfo("settings: %s", settings)
        if param in settings:
            old_value = settings[param]
            print 'old value: ', old_value, ' new value: ', value
            if old_value != value:
                param_file = open(path, 'w')
                rospy.loginfo("change param %s :  %s -> %s ", param, old_value, value)
                settings[param] = value
                set_param_to_server(param, value)
                rospy.logwarn("write_param: write yaml file, write content: %s ", settings)
                print "test settings:", settings
                yaml.dump(settings, param_file)
                param_file.close()
            else:
                rospy.logwarn("param %s: new value == old_value", param)
        else:
            rospy.logerr("have no such param %s !", param)

def set_param(req):
    rospy.loginfo("set param, req.request: %s", req)
    if req.key in settings:
        [result, changed, param, value] = check_param_value_valid(req.key, req.value)
        if result:
            write_param(param, value)
        else:
            rospy.logerr("req param error:   req.key: %s,  req.value: %s", req.key, req.value)
            return [False, 'param\'s value is not valid']
    else:
        rospy.logerr("param %s is not support:", req.key)
        return [False, 'param is not support']

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
