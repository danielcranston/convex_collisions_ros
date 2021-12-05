#! /usr/bin/env python
import rospy
import rospkg
import numpy as np
import tf.transformations as tr
import tf2_ros
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion


def make_tf(frame_name, info):
    T = dh_to_T(**info["dh_parameters"])

    return TransformStamped(
        header=Header(frame_id=info["parent_frame"], stamp=rospy.Time.now()),
        child_frame_id=frame_name,
        transform=T_to_transform(T),
    )


def dh_to_T(theta, alpha, a, d):
    # https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Denavit%E2%80%93Hartenberg_matrix
    # https://www.youtube.com/watch?v=cEXaah0QGZ4
    C = np.cos
    S = np.sin
    return np.array(
        [
            [C(theta), -S(theta) * C(alpha), S(theta) * S(alpha), a * C(theta)],
            [S(theta), C(theta) * C(alpha), -C(theta) * S(alpha), a * S(theta)],
            [0, S(alpha), C(alpha), d],
            [0, 0, 0, 1],
        ]
    )


def T_to_transform(T):
    return Transform(
        translation=Vector3(*T[:3, 3]),
        rotation=Quaternion(*tr.quaternion_from_matrix(T)),
    )


def make_marker(frame_id, mesh_path, alpha):
    m = Marker()
    m.frame_locked = True
    m.header.frame_id = frame_id
    m.id = 0
    m.pose.orientation.w = 1.0
    m.type = Marker.MESH_RESOURCE
    m.scale.x, m.scale.y, m.scale.z = 1.0, 1.0, 1.0
    m.color.a = alpha
    m.mesh_resource = "file://" + mesh_path
    m.mesh_use_embedded_materials = True
    return m


class DenavitHartenbergDemoNode(object):
    def __init__(self):
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        kinematics = rospy.get_param("~kinematics")
        self.tf_frames = [make_tf(frame_name, d) for frame_name, d in kinematics.items()]

        mesh_path = rospy.get_param("~mesh_path")
        self.marker = make_marker("base", mesh_path, alpha=0.5)

        self.marker_pub = rospy.Publisher("bodies", MarkerArray, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.publish_cb)

    def publish_cb(self, _):
        self.marker_pub.publish([self.marker])
        self.broadcaster.sendTransform(self.tf_frames)


if __name__ == "__main__":
    rospy.init_node("denavit_hartenberg_demo_node")
    dhdn = DenavitHartenbergDemoNode()
    rospy.spin()
