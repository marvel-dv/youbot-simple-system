#!/usr/bin/env python3
import rospy
import csv
import math

from gazebo_msgs.srv import SetModelState, GetLinkState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped
from youbot_tools.srv import ApproachTag

"""
rosrun youbot_tools approach_test.py 

Para cada execução:
- Move o bloco no Gazebo
- Mede erro de percepção (AprilTag × ground truth)
- Executa o approach para uma distância desejada
- Mede erro final de aproximação
"""

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
def move_model(set_state_srv, x, y, z,model):
    state = ModelState()
    state.model_name = model
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
# Aguarda AprilTag
# ==============================
def wait_for_tag(timeout=1.5):
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
# Ground Truth (Gazebo)
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

    rospy.init_node("approach_test")

    rospy.loginfo("Aguardando serviços...")
    rospy.wait_for_service("/gazebo/set_model_state")
    rospy.wait_for_service("/gazebo/get_link_state")
    rospy.wait_for_service("/approach_tag")

    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    approach_srv = rospy.ServiceProxy("/approach_tag", ApproachTag)

    rospy.Subscriber("/apriltag/pose_target", PoseStamped, tag_cb)

    rospy.sleep(1.0)

    # ==============================
    # Configuração do experimento
    # ==============================
    d_goal_start = 0.40       #distância desejada inicial [m]
    d_goal_step  = 0.0        # variação da distância desejada
    d_step = 0.2
    x_start   = 0.60          # posição inicial do bloco
    n_tests   = 20            # número de testes
    n = 1
    
    y = 0.
    z = 0.10

    rospy.loginfo(f"Teste de approach: {n_tests} execuções")

    # ==============================
    # CSV
    # ==============================
    with open("approach_test.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "d_goal","x_gt", "y_gt",
            "x_tag", "y_tag",
            "err_tag_x", "err_tag_y", "err_tag_norm",
            "dist_final", "err_approach"
        ])

        # ==============================
        # Loop de testes
        # ==============================
        for j in range(n):
            for i in range(n_tests):

                x = x_start + i * d_step
                d_goal = d_goal_start + i * d_goal_step
            
                if d_goal > 0.6:
                   d_goal = 0.4
            
                rospy.loginfo(f"[TEST {i+1}] Bloco x={x:.2f}, d_goal={d_goal:.3f}")

                # ------------------------------
                # 1) Move bloco
                # ------------------------------
                tag_pose = None
                move_model(set_state, x, y, z,"block_test")
                rospy.sleep(0.6)        
            
                      
                # ------------------------------
                # 2) Espera AprilTag
                # ------------------------------
                if not wait_for_tag():
                    rospy.logwarn("AprilTag não detectada")
                    continue

                # ------------------------------
                # 3) Chama approach
                # ------------------------------
                resp = approach_srv(desired_distance=d_goal)
                if not resp.success:
                    rospy.logwarn("Approach falhou")
                    continue

                rospy.sleep(0.5)

                # ------------------------------
                # 4) Ground Truth final
                # ------------------------------
                gt_pose = get_block_pose_relative(get_link_state)

                # ------------------------------
                # 5) Detecção AprilTag
                # ------------------------------
                px = tag_pose.pose.position.x
                py = tag_pose.pose.position.y

                # ------------------------------
                # 6) Erro de percepção
                # ------------------------------
                err_tag_x = px - gt_pose.position.x
                err_tag_y = py - gt_pose.position.y
                err_tag_norm = math.sqrt(
                    err_tag_x**2 + err_tag_y**2
                )

                # ------------------------------
                # 7) Erro de approach
                # ------------------------------
                dist_final = math.sqrt(
                    gt_pose.position.x**2 +
                    gt_pose.position.y**2
                )

                err_approach = dist_final - d_goal

                # ------------------------------
                # 8) Salva CSV
                # ------------------------------
                writer.writerow([
                    d_goal,
                    gt_pose.position.x,
                    gt_pose.position.y,
                    px,
                    py,
                    err_tag_x,
                    err_tag_y,
                    err_tag_norm,
                    dist_final,
                    err_approach
                ])
            move_model(set_state, 0, 0, 0.1,"youbot")

        rospy.loginfo("Experimento finalizado → approach_test.csv")

