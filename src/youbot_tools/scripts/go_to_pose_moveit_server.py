#!/usr/bin/env python3
import rospy
import tf.transformations as tftr
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotTrajectory
from youbot_tools.srv import GoToPoseMoveIt, GoToPoseMoveItResponse

"""
Not optmized
"""


# -----------------------------------------------------
#  Função robusta que extrai RobotTrajectory do MoveIt
# -----------------------------------------------------
def extract_trajectory(plan_raw):

    # Caso 1: Retorno direto RobotTrajectory
    if hasattr(plan_raw, "joint_trajectory"):
        return plan_raw

    # Caso 2: dict
    if isinstance(plan_raw, dict):
        if "joint_trajectory" in plan_raw:
            traj = RobotTrajectory()
            traj.joint_trajectory = plan_raw["joint_trajectory"]
            return traj
        return None

    # Caso 3: tuple (Noetic)
    if isinstance(plan_raw, tuple):

        # Formato mais comum: (success_bool, RobotTrajectory, error_code)
        if len(plan_raw) >= 2 and isinstance(plan_raw[0], bool):

            success = plan_raw[0]
            if not success:
                return None

            candidate = plan_raw[1]
            if hasattr(candidate, "joint_trajectory"):
                return candidate
            return None

        # Caso raro: primeiro elemento já é a traj
        if hasattr(plan_raw[0], "joint_trajectory"):
            return plan_raw[0]

        return None

    # Tipo desconhecido
    return None


# -----------------------------------------------------
# Handler do serviço
# -----------------------------------------------------
def handle_go_to_pose(req):

    rospy.loginfo(
        "[IK SERVICE] Target pose: x=%.3f y=%.3f z=%.3f rpy=(%.2f %.2f %.2f)",
        req.x, req.y, req.z, req.roll, req.pitch, req.yaw
    )

    # Converter RPY → quaternion
    quat = tftr.quaternion_from_euler(req.roll, req.pitch, req.yaw)

    # Criar pose desejada
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = req.x
    pose.pose.position.y = req.y
    pose.pose.position.z = req.z
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    try:
        # Atualizar estado atual do robô
        arm.set_start_state_to_current_state()
        
        # Parâmetros de velocidade 
        arm.set_max_velocity_scaling_factor(0.8)
        arm.set_max_acceleration_scaling_factor(0.8)

        # Parâmetros de tolerancia
        arm.set_goal_position_tolerance(0.025)     # 2.5 cm 
        arm.set_goal_orientation_tolerance(0.1)  # ~5.7 graus -
        arm.set_planning_time(15.0)
        
        # DEFINIR O END-EFFECTOR CORRETO
        #END_EFFECTOR_LINK = "gripper_tcp",end_effector_link=END_EFFECTOR_LINK

        # Mandar pose alvo
        arm.set_pose_target(pose)

        # Planejar
        plan_raw = arm.plan()

        # Extrair trajetória válida
        traj = extract_trajectory(plan_raw)
        if traj is None:
            rospy.logerr("MoveIt: no valid trajectory found")
            return GoToPoseMoveItResponse(False, "MoveIt failed to generate a trajectory")

        if len(traj.joint_trajectory.points) == 0:
            rospy.logerr("MoveIt: empty trajectory")
            return GoToPoseMoveItResponse(False, "Empty trajectory")

        # Executar movimento
        arm.execute(traj, wait=True)
        arm.stop()
        arm.clear_pose_targets()
        #rospy.sleep(0.25)

        return GoToPoseMoveItResponse(True, "MoveIt motion executed")

    except Exception as e:
        rospy.logerr("MoveIt exception: %s", str(e))
        return GoToPoseMoveItResponse(False, f"Exception: {str(e)}")


# -----------------------------------------------------
# Inicialização do node e serviço
# -----------------------------------------------------
def start_server():
    rospy.init_node("go_to_pose_moveit_server")

    global arm
    roscpp_initialize([])

    arm = MoveGroupCommander("arm_1")
    arm.set_planning_time(5.0)
    arm.allow_replanning(True)

    service = rospy.Service(
        "/youbot/go_to_pose_moveit",
        GoToPoseMoveIt,
        handle_go_to_pose
    )

    rospy.loginfo("Service /youbot/go_to_pose_moveit READY")
    rospy.spin()

    roscpp_shutdown()


if __name__ == "__main__":
    start_server()






