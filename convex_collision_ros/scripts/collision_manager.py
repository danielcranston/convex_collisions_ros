#! /usr/bin/env python
import numpy as np
import rospy
import tf
import tf.transformations as tr
from convex_collision_msgs.srv import CheckCollision, CheckCollisionResponse
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


def make_arrow_marker(frame_id, id, dir, color):
    m = Marker()
    m.header.frame_id = frame_id
    m.id = id
    m.pose.orientation.w = 1.0
    m.type = Marker.ARROW
    m.points = [Point(0, 0, 0), Point(dir[0], dir[1], dir[2])]
    m.scale.x, m.scale.y, m.scale.z = 0.05, 0.1, 0.1
    m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 0.25
    return m


def apply_homog_transform(T, data, rotation_only=False):
    if rotation_only:
        return T[:3, :3].dot(data.T).T
    else:
        data_homog = np.hstack([data, np.ones([data.shape[0], 1])])
        transformed_homog = T.dot(data_homog.T).T
        return np.array([t[:3] / t[3] for t in transformed_homog])


def to_matrix(pos, quat):
    # or alternatively: tr.compose_matrix(pos, tr.euler_from_quaternion(quat))
    T = tr.quaternion_matrix(quat)
    T[:3, 3] = pos
    return T


def convex_hull_to_numpy(msg):
    vertices = np.array([vec3_to_numpy(n) for n in msg.vertices])
    normals = np.array([vec3_to_numpy(n) for n in msg.normals])
    edges = np.array([vec3_to_numpy(e) for e in msg.edges])
    return vertices, normals, edges


def vec3_to_numpy(msg):
    return np.array([msg.x, msg.y, msg.z])


def quat_to_numpy(msg):
    return np.array([msg.x, msg.y, msg.z, msg.w])


def make_separating_axes(normals1, edges1, normals2, edges2):
    return np.vstack(
        [
            normals1,
            normals2,
            np.array([np.cross(e1, e2) for e1 in edges1 for e2 in edges2]),
        ]
    )


def is_separating_axis(separating_axis, vertices1, vertices2):
    proj1 = separating_axis.dot(vertices1.T)
    proj2 = separating_axis.dot(vertices2.T)
    if np.min(proj2) > np.max(proj1) or np.min(proj1) > np.max(proj2):
        return True
    else:
        return False


class CollisionManager(object):
    def __init__(self):
        self.publish_separating_axes = rospy.get_param("~publish_separating_axes", False)
        self.verbose = rospy.get_param("~verbose", False)
        self.marker_pub = rospy.Publisher("separating_axes", MarkerArray, queue_size=1)
        self.tf_lst = tf.TransformListener()
        rospy.Service("check_collision", CheckCollision, self.check_collision_cb)

    def check_collision_cb(self, req):
        try:
            T_body1frame_body2frame = to_matrix(
                *self.tf_lst.lookupTransform("/body1", "/body2", rospy.Time(0))
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Failed to lookup transform between bodies: {}".format(e.message))
            return CheckCollisionResponse(success=False)

        T_body1frame_body1origin = to_matrix(
            vec3_to_numpy(req.body_a.pose.pose.position),
            quat_to_numpy(req.body_a.pose.pose.orientation),
        )

        T_body2frame_body2origin = to_matrix(
            vec3_to_numpy(req.body_b.pose.pose.position),
            quat_to_numpy(req.body_b.pose.pose.orientation),
        )

        T_body1origin_body2origin = np.linalg.inv(T_body1frame_body1origin).dot(
            T_body1frame_body2frame.dot(T_body2frame_body2origin)
        )

        vertices1, normals1, edges1 = convex_hull_to_numpy(req.body_a)
        vertices2, normals2, edges2 = convex_hull_to_numpy(req.body_b)

        # Transform body B data to body A's frame.
        # Note: normals and edges are directional vectors, so only apply the rotational component
        vertices2 = apply_homog_transform(T_body1origin_body2origin, vertices2, rotation_only=False)
        normals2 = apply_homog_transform(T_body1origin_body2origin, normals2, rotation_only=True)
        edges2 = apply_homog_transform(T_body1origin_body2origin, edges2, rotation_only=True)

        # Now that both bodies are expressed in the same frame, we can calculate the axes that
        # might be separating ...
        candidate_axes = make_separating_axes(normals1, edges1, normals2, edges2)

        # ... then project the bodies vertices onto each axes, evaluating separation ...
        was_separating_axis = [
            is_separating_axis(axis, vertices1, vertices2) for axis in candidate_axes
        ]

        if self.publish_separating_axes:
            self.marker_pub.publish(
                [
                    make_arrow_marker(
                        req.body_a.pose.header.frame_id,
                        i,
                        direction,
                        [0, 1, 0] if was_separating_axis[i] else [1, 0, 0],
                    )
                    for i, direction in enumerate(candidate_axes)
                ]
            )

        if self.verbose:
            rospy.loginfo(
                "Out of all {} candidate axes, {} were separating".format(
                    len(was_separating_axis), np.sum(was_separating_axis)
                )
            )

        # ... and flagging collision iff there were no axes separating the two bodies.
        return CheckCollisionResponse(success=True, is_colliding=not any(was_separating_axis))


if __name__ == "__main__":
    rospy.init_node("collision_manager")
    cm = CollisionManager()
    rospy.spin()