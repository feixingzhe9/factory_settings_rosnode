#!/usr/bin/env python
# coding: utf-8

import sys
import json
import os
import yaml
import rospy
import rospkg
from mrobot_srvs.srv import JString
from mrobot_srvs.srv import KVPair

path = None 
settings = None

def read_param(param):
    global path
    if not os.path.exists(path):
        rospy.logerr("%s: file not exists", path)
    else:
        #rospy.loginfo("%s: open ok", path)
        param_file = open(path, 'r')
        global settings
        settings = yaml.load(param_file)
        rospy.loginfo("get param string: %s", settings)
        rospy.loginfo("%s: %s", param, settings.get(param, ''))
        param_file.close()


def read_all_params():
    global path
    if not os.path.exists(path):
        rospy.logerr("%s: file not exists", path)
    else:
        #rospy.loginfo("%s: open ok", path)
        param_file = open(path, 'r')
        global settings
        settings = yaml.load(param_file)
        for param in settings:
            rospy.set_param(param, settings[param])
        rospy.loginfo("get all params: %s", settings)
        param_file.close()

def write_param(param, value):
    global path
    if not os.path.exists(path):
        rospy.logerr("%s: file not exists", path)
    else:
        #rospy.loginfo("%s: open ok", path)
        param_file = open(path, 'w')
        global settings
        rospy.loginfo("settings: %s", settings)
        if settings is not None:
            if param in settings:
                rospy.loginfo("get param %s", param)
                old_value = settings[param]
                if old_value != value:
                    rospy.loginfo("change param %s :  %s -> %s ", param, old_value, value)
                    settings[param] = value
                    yaml.dump(settings, param_file)
                else:
                    rospy.logwarn("param %s: new value == old_value", param)
                    yaml.dump(settings, param_file)
            else:
                rospy.logwarn("have no param %s, add", param)
                settings[param] = value
                yaml.dump(settings, param_file)
        else:
            settings = {param: value}

        param_file.close()

def set_param(req):
    rospy.loginfo("set param, req.request: %s", req)
    #setting = json.loads(req)
    write_param(req.key, req.value)
    read_all_params()

    return [True, 'ok']

def main():
    global path
    rospy.init_node("factory_settings", anonymous = True)
    rospy.Service('/factory_settings/set_param', KVPair, set_param)
    path = os.path.join(rospkg.get_ros_home(), "factory_settings.yaml")
    rospy.loginfo("path: %s", path)
    read_all_params()
    rospy.spin()

if __name__  == '__main__':
    try:
        main()
    except Exception:
        rospy.logerr(sys.exc_info())
        rospy.logerr("lost connection")
        exit(1)
