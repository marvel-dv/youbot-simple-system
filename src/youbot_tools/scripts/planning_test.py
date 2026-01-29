#!/usr/bin/env python3
import rospy
import csv
import math
import numpy as np

import tf2_ros
import tf.transformations as tftr

from youbot_tools.srv import GoToPoseMoveIt

"""
Teste do planejamento do braço usando uma LISTA DE POSES conhecidas.

Para cada pose:
- Move o braço via MoveIt
- Lê TF do gripper_tcp
- Calcula erro de posição
- Calcula erro de orientação (quaternion)
- Repete N vezes para análise de repetibilidade
"""

# ==============================
# Funções auxiliares
# ==============================

def normalize_quat(q):
    """Normaliza quaternion para norma 1"""
    q = np.array(q, dtype=float)
    return q / np.linalg.norm(q)


def quat_angle_error(q_des, q_real):
    """
    Calcula erro angular entre dois quaternions.
    Retorna erro em radianos.
    """

    q_des = normalize_quat(q_des)
    q_real = normalize_quat(q_real)

    q_real_inv = tftr.quaternion_conjugate(q_real)
    q_err = tftr.quaternion_multiply(q_des, q_real_inv)

    w = np.clip(abs(q_err[3]), -1.0, 1.0)
    theta = 2 * math.acos(w)

    return theta


def quat_to_rpy(q):
    q = normalize_quat(q)
    return tftr.euler_from_quaternion(q)


def get_tcp_pose(tf_buffer,
                 base_frame="base_footprint",
                 tcp_frame="gripper_tcp"):
    try:
        trans = tf_buffer.lookup_transform(
            base_frame,
            tcp_frame,
            rospy.Time(0),
            rospy.Duration(0.5)
        )

        p = trans.transform.translation
        q = trans.transform.rotation

        pos = np.array([p.x, p.y, p.z])
        quat = np.array([q.x, q.y, q.z, q.w])

        return pos, quat

    except Exception as e:
        rospy.logwarn(f"[TF] Falha ao obter TCP: {e}")
        return None, None


def move_arm(moveit_srv, x, y, z, roll, pitch, yaw):
    resp = moveit_srv(x, y, z, roll, pitch, yaw)
    if not resp.success:
        rospy.logerr("[MOVEIT] " + resp.message)
        return False
    return True


# ==============================
# Main
# ==============================
if __name__ == "__main__":

    rospy.init_node("planning_test")

    rospy.loginfo("Aguardando serviço MoveIt...")
    rospy.wait_for_service("/youbot/go_to_pose_moveit")
    moveit_srv = rospy.ServiceProxy(
        "/youbot/go_to_pose_moveit",
        GoToPoseMoveIt
    )

    # TF
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(1.0)

    # ==============================
    # LISTA DE POSES TESTADAS
    # ==============================
    poses = [
        {"x": 0.30, "y": 0.00, "z": 0.15, "q": [0, 1, 0, 0]},#1
        {"x": 0.40, "y": 0.10, "z": 0.12, "q": [0, 1, 0, 0]},#2
        {"x": 0.25, "y": -0.20, "z": 0.10, "q": [0.72, -0.7, 0, 0]},#3(orientação não paralela a algum eixo)
        {"x": 0.35, "y": 0.25, "z": 0.1, "q": [0, 1, 0, 0]},#4
        {"x": 0.50, "y": 0.00, "z": 0.00, "q": [0, 1, 0, 0]},#5(falhou uma vez)(limite de uma junta)
        {"x": 0.00, "y": 0.00, "z": 0.15, "q": [0, 1, 0, 0]},#6(para tras)
        {"x": 0.20, "y": 0.00, "z": 0.60, "q": [0, 0, 0, 1]},#7 (candle)
        {"x": 0.30, "y": 0.20, "z": 0.30, "q": [-0.7, 0.4, 0, 0.6]},#8 (orientação não paralela a algum eixo)(erro sistematico nessa pose)
    ]                                                               #orientação mais complexa(quaternion com mais vetores)

    n_reps = 3

    rospy.loginfo(
        f"Iniciando teste com {len(poses)} poses, {n_reps} repetições"
    )

    # ==============================
    # CSV
    # ==============================
    with open("planning_test.csv", "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "rep", "pose_id",
            "x_des", "y_des", "z_des",
            "x_tcp", "y_tcp", "z_tcp",
            "err_x", "err_y", "err_z", "err_norm",
            "err_ang_rad", "err_ang_deg"
        ])

        # ==============================
        # Loop principal
        # ==============================
        for j in range(n_reps):
            for i, pose in enumerate(poses):

                rospy.loginfo(
                    f"[rep {j+1}/{n_reps}] Pose {i+1}/{len(poses)}"
                )

                x = pose["x"]
                y = pose["y"]
                z = pose["z"]

                q_des = normalize_quat(pose["q"])
                roll, pitch, yaw = quat_to_rpy(q_des)

                if not move_arm(
                    moveit_srv,
                    x, y, z,
                    roll, pitch, yaw
                ):
                    rospy.logerr("Falha no planejamento")
                    continue

                rospy.sleep(4.0)

                tcp_pos, tcp_quat = get_tcp_pose(tf_buffer)
                if tcp_pos is None or tcp_quat is None:
                    continue

                err = tcp_pos - np.array([x, y, z])
                err_norm = np.linalg.norm(err)

                err_ang = quat_angle_error(q_des, tcp_quat)
                err_ang_deg = math.degrees(err_ang)

                writer.writerow([
                    j, i+1,
                    x, y, z,
                    tcp_pos[0], tcp_pos[1], tcp_pos[2],
                    err[0], err[1], err[2], err_norm,
                    err_ang, err_ang_deg
                ])

                rospy.loginfo(
                    f"Erro posição: {err_norm:.4f} m | "
                    f"Erro orientação: {err_ang_deg:.2f} deg"
                )

    rospy.loginfo("Teste finalizado. CSV salvo: planning_test.csv")

    

  

