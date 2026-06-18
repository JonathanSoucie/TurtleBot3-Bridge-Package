

import math
import time
import threading
import os
import roslibpy


BRIDGE_HOST = os.environ.get("TB3_BRIDGE_HOST", "localhost")
BRIDGE_PORT = int(os.environ.get("TB3_BRIDGE_PORT", "9090"))

# Mode: "sim" (default) - talk to Gazebo's native topics
# Mode: "real" - talk through the C++ bridge using /student/* topics
MODE = os.environ.get("TB3_MODE", "sim").lower()

_ros = None
_spin_running = False

def _resolve_topic(topic):
    """In sim mode, strip the /student/ prefix so the same code drives Gazebo
    directly. In real mode, leave it alone so it lands on the C++ bridge."""
    if MODE == "sim" and topic.startswith("/student/"):
        return "/" + topic[len("/student/"):]
    return topic


def init(bridge_host=None, bridge_port=None, mode=None):
    """Connect to rosbridge. Call this before creating any nodes."""
    global _ros, BRIDGE_HOST, BRIDGE_PORT, MODE
    if bridge_host:
        BRIDGE_HOST = bridge_host
    if bridge_port:
        BRIDGE_PORT = bridge_port
    if mode:
        MODE = mode.lower()
    if MODE not in ("sim", "real"):
        raise ValueError(f"TB3_MODE must be 'sim' or 'real', got '{MODE}'")

    _ros = roslibpy.Ros(host=BRIDGE_HOST, port=BRIDGE_PORT)
    _ros.run()
    if not _ros.is_connected:
        raise ConnectionError(
            f"Could not connect to rosbridge at {BRIDGE_HOST}:{BRIDGE_PORT}  (MODE={MODE})\n")
    print(f"[roslib_wrapper] connected to {BRIDGE_HOST}:{BRIDGE_PORT}  (MODE={MODE})")


def spin(node):
    """Block and process callbacks until Ctrl-C."""
    global _spin_running
    _spin_running = True
    try:
        while _spin_running and _ros and _ros.is_connected:
            now = time.time()
            for timer in node._timers:
                if now - timer['last'] >= timer['period']:
                    timer['callback']()
                    timer['last'] = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


def ok():
    return _spin_running and _ros is not None and _ros.is_connected


def shutdown():
    global _ros, _spin_running
    _spin_running = False
    if _ros:
        _ros.terminate()
        _ros = None


#  MESSAGE TYPES - dot-notation wrappers that act like real ROS 2 messages
        
class _DotDict:
    def __init__(self, d=None):
        if d:
            for k, v in d.items():
                if isinstance(v, dict):
                    setattr(self, k, _DotDict(v))
                elif isinstance(v, list):
                    setattr(self, k, v)
                else:
                    setattr(self, k, v)

    def _to_dict(self):
        result = {}
        for k, v in self.__dict__.items():
            if isinstance(v, _DotDict):
                result[k] = v._to_dict()
            else:
                result[k] = v
        return result


class Vector3(_DotDict):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Twist(_DotDict):
    """geometry_msgs/msg/Twist"""
    _ros_type = "geometry_msgs/Twist"

    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class Quaternion(_DotDict):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class Point(_DotDict):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Pose(_DotDict):
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()


class PoseWithCovariance(_DotDict):
    def __init__(self):
        self.pose = Pose()
        self.covariance = [0.0] * 36


class TwistPlain(_DotDict):
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class TwistWithCovariance(_DotDict):
    def __init__(self):
        self.twist = TwistPlain()
        self.covariance = [0.0] * 36


class Header(_DotDict):
    def __init__(self):
        self.stamp = _DotDict({"sec": 0, "nanosec": 0})
        self.frame_id = ""


class LaserScan(_DotDict):
    """sensor_msgs/msg/LaserScan"""
    _ros_type = "sensor_msgs/LaserScan"

    def __init__(self):
        self.header = Header()
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.time_increment = 0.0
        self.scan_time = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        self.ranges = []
        self.intensities = []


class Odometry(_DotDict):
    """nav_msgs/msg/Odometry"""
    _ros_type = "nav_msgs/Odometry"

    def __init__(self):
        self.header = Header()
        self.child_frame_id = ""
        self.pose = PoseWithCovariance()
        self.twist = TwistWithCovariance()


def _dict_to_msg(msg_class, d):
    msg = msg_class()
    _fill_from_dict(msg, d)
    return msg


def _fill_from_dict(obj, d):
    if not isinstance(d, dict):
        return
    for k, v in d.items():
        if hasattr(obj, k):
            attr = getattr(obj, k)
            if isinstance(v, dict) and isinstance(attr, _DotDict):
                _fill_from_dict(attr, v)
            else:
                setattr(obj, k, v)


#  PUBLISHER / SUBSCRIBER

class _Publisher:
    def __init__(self, ros, msg_type, topic, qos):
        resolved = _resolve_topic(topic)
        if resolved != topic:
            print(f"[roslib_wrapper] sim-mode remap: publisher {topic} -> {resolved}")
        self._topic = roslibpy.Topic(ros, resolved, msg_type._ros_type)
        self._topic.advertise()

    def publish(self, msg):
        self._topic.publish(roslibpy.Message(msg._to_dict()))


class _Subscription:
    def __init__(self, ros, msg_type, topic, callback, qos):
        resolved = _resolve_topic(topic)
        if resolved != topic:
            print(f"[roslib_wrapper] sim-mode remap: subscription {topic} -> {resolved}")
        self._msg_type = msg_type
        self._callback = callback
        self._topic = roslibpy.Topic(ros, resolved, msg_type._ros_type)
        self._topic.subscribe(self._on_message)

    def _on_message(self, raw_dict):
        msg = _dict_to_msg(self._msg_type, raw_dict)
        self._callback(msg)


#  NODE

class Node:
    def __init__(self, name):
        self._name = name
        self._timers = []
        self._publishers = []
        self._subscriptions = []

    def create_publisher(self, msg_type, topic, qos_depth):
        pub = _Publisher(_ros, msg_type, topic, qos_depth)
        self._publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos_depth):
        sub = _Subscription(_ros, msg_type, topic, callback, qos_depth)
        self._subscriptions.append(sub)
        return sub

    def create_timer(self, period_sec, callback):
        self._timers.append({
            'period': period_sec,
            'callback': callback,
            'last': time.time(),
        })

    def get_logger(self):
        return _Logger(self._name)

    def destroy_node(self):
        pass


class _Logger:
    def __init__(self, name):
        self._name = name

    def info(self, msg):
        print(f"[INFO] [{self._name}]: {msg}")

    def warn(self, msg):
        print(f"[WARN] [{self._name}]: {msg}")

    def error(self, msg):
        print(f"[ERROR] [{self._name}]: {msg}")
