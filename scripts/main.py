#!/usr/bin/env python
# coding: utf-8

import sys
import json
import os
import yaml
import rospy
import rospkg


#path = "/home/kaka/.ros/factory_settings.yaml"
path = None 
settings = None


def read_param(param):
    global path
    if not os.path.exists(path):
        rospy.logerr("%s: file not exists", path)
    else:
        rospy.loginfo("%s: open ok", path)
        param_file = open(path, 'r')
        global settings
        settings = yaml.load(param_file)
        rospy.loginfo("get param string: %s", str(settings))
        rospy.loginfo("%s: %s", param, settings.get(param, ''))
        param_file.close()

def write_param(param, value):
    global path
    if not os.path.exists(path):
        rospy.logerr("%s: file not exists", path)
    else:
        rospy.loginfo("%s: open ok", path)
        param_file = open(path, 'w')
        global settings
        rospy.loginfo("settings: %s", settings)
        settings[param] = value
        yaml.dump(settings, param_file)
        param_file.close()


def main():
    global path
    rospy.init_node("factory_settings", anonymous = True)
    path = os.path.join(rospkg.get_ros_home(), "factory_settings.yaml")
    rospy.loginfo("path: %s", path)
    read_param("lock_num")
    write_param("conveyor_lock", True)
    write_param("audio_channel", 'rk')

if __name__  == '__main__':
    try:
        main()
    except Exception:
        rospy.logerr(sys.exc_info())
        rospy.logerr("lost connection")
        exit(1)
