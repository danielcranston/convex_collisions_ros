#! /usr/bin/env python
import rospy
import rospkg
import trimesh
from visualization_msgs.msg import MarkerArray, Marker
from convex_collision_msgs.srv import CheckCollision
from convex_collision_msgs.msg import ConvexHull
from geometry_msgs.msg import Vector3, Point


def get_box():
    vertices = []
    for x in [-0.5, 0.5]:
        for y in [-0.5, 0.5]:
            for z in [-0.5, 0.5]:
                vertices.append(Point(x, y, z))
    normals = [Vector3(1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0)]
    edges = normals
    return vertices, normals, edges


def get_ray():
    vertices = [Point(0, 0, 0), Point(3, 0, 0)]
    normals = [Point(1, 0, 0)]
    edges = normals
    return vertices, normals, edges


def get_mesh(mesh_path):
    m = trimesh.load(mesh_path)
    vertices = [Point(*v) for v in m.vertices]
    normals = [Vector3(*n) for n in m.face_normals]
    edges = [Vector3(*(m.vertices[e[0]] - m.vertices[e[1]])) for e in m.edges]
    return vertices, normals, edges


def make_convex_hull(frame_id, vertices, normals, edges):
    ret = ConvexHull()
    ret.pose.header.frame_id = frame_id
    ret.pose.pose.orientation.w = 1.0
    ret.vertices, ret.normals, ret.edges = vertices, normals, edges
    return ret


def make_marker(frame_id, id, marker_type, mesh_path=None):
    m = Marker()
    m.frame_locked = True
    m.header.frame_id = frame_id
    m.id = id
    m.pose.orientation.w = 1.0
    m.type = marker_type
    m.scale.x, m.scale.y, m.scale.z = 1.0, 1.0, 1.0
    m.color.a = 0.5
    if marker_type == Marker.MESH_RESOURCE:
        m.mesh_resource = "file://" + mesh_path
    elif marker_type == Marker.ARROW:
        m.points = [Point(0, 0, 0), Point(3, 0, 0)]
        m.scale.x, m.scale.y, m.scale.z = 0.05, 0.1, 0.1
    return m


class DemoNode(object):
    def __init__(self):
        self.marker_pub = rospy.Publisher("bodies", MarkerArray, queue_size=1)
        rospy.wait_for_service("check_collision")
        self.check_collision_srv = rospy.ServiceProxy("check_collision", CheckCollision)

        demo_type = rospy.get_param("~demo_type", "box")
        mesh_path = None
        if demo_type == "box":
            marker_types = [Marker.CUBE, Marker.CUBE]
            body1_info, body2_info = [get_box()] * 2
        elif demo_type == "mesh":
            marker_types = [Marker.MESH_RESOURCE, Marker.MESH_RESOURCE]
            mesh_path = rospkg.RosPack().get_path("convex_collision_ros") + "/meshes/convex.obj"
            body1_info, body2_info = [get_mesh(mesh_path)] * 2
        elif demo_type == "ray":
            marker_types = [Marker.CUBE, Marker.ARROW]
            body1_info, body2_info = get_box(), get_ray()
        else:
            raise RuntimeError("Unexpected demo_type: {}".format(demo_type))

        self.body1 = make_convex_hull("body1", *body1_info)
        self.body2 = make_convex_hull("body2", *body2_info)

        self.marker1 = make_marker("body1", 1, marker_types[0], mesh_path)
        self.marker2 = make_marker("body2", 2, marker_types[1], mesh_path)

        rospy.Timer(rospy.Duration(0.1), self.check_and_publish_cb)

    def check_and_publish_cb(self, _):
        try:
            ret = self.check_collision_srv.call(self.body1, self.body2)
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to call /check_collision service: {}".format(e.message))
            return

        self.marker1.color.r, self.marker1.color.g = (1.0, 0.0) if ret.is_colliding else (0.0, 1.0)
        self.marker2.color = self.marker1.color
        self.marker_pub.publish([self.marker1, self.marker2])


if __name__ == "__main__":
    rospy.init_node("demo_node")
    dn = DemoNode()
    rospy.spin()
