#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

"""
Obsolete
"""
def tag_callback(msg):
    global tf_buffer
    if len(msg.detections) == 0:
        return

    for det in msg.detections:
        tag_id = det.id[0]
        pose = det.pose.pose.pose

        pos = pose.position
        ori = pose.orientation

        frame_cam = det.pose.header.frame_id  # normalmente front_camera_link

        rospy.loginfo("========== TAG DETECTADA ==========")
        rospy.loginfo(f"ID: {tag_id}")
        rospy.loginfo(f"Frame da detecção: {frame_cam}")

        rospy.loginfo(
            "Posição NO FRAME DA CÂMERA [m]: x={:.4f}, y={:.4f}, z={:.4f}".format(
                pos.x, pos.y, pos.z
            )
        )

        rospy.loginfo(
            "Orientação (quaternion): x={:.4f}, y={:.4f}, z={:.4f}, w={:.4f}".format(
                ori.x, ori.y, ori.z, ori.w
            )
        )

        # ---------- TRANSFORMAÇÃO PARA base_link ----------
        p_cam = PointStamped()
        p_cam.header.stamp = rospy.Time(0)  # usa o TF mais recente
        p_cam.header.frame_id = frame_cam

        p_cam.point.x = pos.x
        p_cam.point.y = pos.y
        p_cam.point.z = pos.z

        try:
            p_base = tf_buffer.transform(
                p_cam,
                "base_link",
                rospy.Duration(1.0)
            )

            rospy.loginfo(
                "Posição NO FRAME base_link [m]: x={:.4f}, y={:.4f}, z={:.4f}".format(
                    p_base.point.x,
                    p_base.point.y,
                    p_base.point.z
                )
            )

        except Exception as e:
            rospy.logwarn(f"Falha ao transformar ponto para base_link: {e}")

        rospy.loginfo("===================================")

def main():
    global tf_buffer
    rospy.init_node("print_apriltag_pose")

    rospy.Subscriber(
        "/tag_detections",
        AprilTagDetectionArray,
        tag_callback
    )

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo("Nó print_apriltag_pose iniciado.")
    rospy.spin()

if __name__ == "__main__":
    main()

