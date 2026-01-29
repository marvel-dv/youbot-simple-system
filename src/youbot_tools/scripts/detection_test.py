#!/usr/bin/env python3
import rospy
import csv
import math
import numpy as np

from gazebo_msgs.srv import SetModelState, GetLinkState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped


# ==============================
# Variável global da AprilTag
# ==============================
tag_pose = None


def tag_cb(msg):
    global tag_pose
    tag_pose = msg


# ==============================
# Move o bloco no Gazebo (WORLD)
# ==============================
def move_model(set_state_srv, x, y, z):
    state = ModelState()
    state.model_name = "block_test"
    state.reference_frame = "world"

    state.pose.position.x = x
    state.pose.position.y = y
    state.pose.position.z = z

    state.pose.orientation.x = 0.0
    state.pose.orientation.y = 0.0
    state.pose.orientation.z = 0.0
    state.pose.orientation.w = 1.0

    set_state_srv(state)


# ==============================
# Aguarda detecção da tag
# ==============================
def wait_for_tag(timeout=1.0):
    global tag_pose
    t0 = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        if tag_pose is not None:
            return True
        if rospy.Time.now().to_sec() - t0 > timeout:
            return False
        rospy.sleep(0.05)

    return False


# ==============================
# Ground Truth via Gazebo
# ==============================
def get_block_pose_relative(get_link_state_srv):
    block_link = "block_test::block_link"
    robot_base = "youbot::base_footprint"

    resp = get_link_state_srv(
        link_name=block_link,
        reference_frame=robot_base
    )

    if not resp.success:
        raise RuntimeError("Falha no get_link_state")

    return resp.link_state.pose


# ==============================
# Main
# ==============================
if __name__ == "__main__":

    rospy.init_node("detection_test")

    rospy.loginfo("Aguardando serviços do Gazebo...")
    rospy.wait_for_service("/gazebo/set_model_state")
    rospy.wait_for_service("/gazebo/get_link_state")

    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

    rospy.Subscriber("/apriltag/pose_target", PoseStamped, tag_cb)

    rospy.sleep(0.5)

    # ==============================
    # GRID (Z fixo)
    # ==============================
    xs = np.arange(0.3, 0.8, 0.025)
    ys = np.arange(-0.5, 0.5, 0.025)
    z = 0.05
    
    #xs = np.arange(0.3, 2, 0.05)
    #ys = np.arange(-0.9, 0.9, 0.05)
    #z = 0.05

    rospy.loginfo(
        f"GRID configurado: {len(xs)} x {len(ys)} = {len(xs)*len(ys)} pontos"
    )

    # ==============================
    # CSV
    # ==============================
    with open("detection_test.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "x_gt", "y_gt", "z_gt",
            "x_tag", "y_tag", "z_tag",
            "err_x", "err_y", "err_z", "err_norm"
        ])

        # ==============================
        # Loop
        # ==============================
        for x in xs:
            for y in ys:

                rospy.loginfo(
                    f"Testando posição WORLD: x={x:.3f}, y={y:.3f}, z={z:.2f}"
                )

                tag_pose = None
                move_model(set_state, x, y, z)
                rospy.sleep(1)

                if not wait_for_tag():
                    rospy.logwarn("AprilTag não detectada")
                    continue

                # Ground truth (base do robô)
                gt_pose = get_block_pose_relative(get_link_state)

                # Detecção Apriltag (já em base)
                px = tag_pose.pose.position.x
                py = tag_pose.pose.position.y
                pz = tag_pose.pose.position.z

                # Erros
                err_x = px - gt_pose.position.x
                err_y = py - gt_pose.position.y
                err_z = pz - gt_pose.position.z
                err_norm = math.sqrt(
                    err_x**2 + err_y**2+ err_z**2)

                writer.writerow([
                    gt_pose.position.x,
                    gt_pose.position.y,
                    gt_pose.position.z,
                    px, py, pz,
                    err_x, err_y,err_z,
                    err_norm
                ])

    rospy.loginfo("Experimento finalizado. CSV salvo: detection_test.csv")


