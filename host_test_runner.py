#!/usr/bin/env python3
"""Run the full regression suite on a workstation without ROS installed.

These shims only satisfy imports and construct the small message objects used
by unit tests. They do not create a ROS node or connect to robot hardware.
"""
import sys
import types
import unittest
from types import SimpleNamespace
from unittest.mock import Mock


def module(name):
    value = types.ModuleType(name)
    sys.modules[name] = value
    return value


rospy = module("rospy")
rospy.INFO = 20
rospy.Time = SimpleNamespace(now=Mock(return_value=SimpleNamespace(to_sec=lambda: 0.0)))
for name in ("logwarn_throttle", "logerr_throttle", "loginfo", "logwarn", "logerr"):
    setattr(rospy, name, Mock())
rospy.wait_for_service = Mock()
rospy.ServiceProxy = Mock()
rospy.Subscriber = Mock()

for package, names in (
    ("geometry_msgs", ("PoseStamped", "PoseWithCovarianceStamped")),
    ("nav_msgs", ("OccupancyGrid",)),
    ("sensor_msgs", ("BatteryState", "CameraInfo", "Image", "LaserScan")),
    ("std_msgs", ("Bool",)),
    ("std_srvs", ("Empty", "Trigger")),
):
    parent = module(package)
    child = module(package + (".srv" if package == "std_srvs" else ".msg"))
    setattr(parent, "srv" if package == "std_srvs" else "msg", child)
    for name in names:
        setattr(child, name, type(name, (), {}))


class Bool:
    def __init__(self, data=False):
        self.data = bool(data)


class PoseStamped:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))


sys.modules["std_msgs.msg"].Bool = Bool
sys.modules["geometry_msgs.msg"].PoseStamped = PoseStamped


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.discover(".")
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)
