#!/usr/bin/env python3
import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

"""
Para dar os comandos:

rosservice call /arm_1/gripper/open
rosservice call /arm_1/gripper/close
rosservice call /arm_1/gripper/grasp
rosservice call /arm_1/gripper/fakegrasp
rosservice call /arm_1/gripper/fakeopen
"""

class YoubotGripperNode:
    def __init__(self):
        rospy.loginfo("Inicializando gripper_node...")

        # ------------------ PARÂMETROS ------------------
        self.OPEN_POS = 0.011
        self.CLOSE_POS = 0.000
        self.MOVE_TIME = 1.0
        self.POS_EPS = 1e-4
        self.STALL_TIME = 0.5
        self.RATE = 50
        self.OBJECT_NAME = "block_test"   # objeto fixo da simulação
        # ------------------------------------------------

        # Estado das juntas
        self.left_pos = None
        self.right_pos = None
        self.prev_left = None
        self.prev_time = None

        self.attached_object = None

        rospy.Subscriber("/joint_states", JointState, self.joint_cb)

        # Action client do gripper
        self.client = actionlib.SimpleActionClient(
            "/arm_1/gripper_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

        rospy.loginfo("Aguardando controlador do gripper...")
        self.client.wait_for_server()
        rospy.loginfo("Controlador do gripper conectado.")

        # Serviços ROS
        rospy.Service("/arm_1/gripper/open", Empty, self.open_srv)
        rospy.Service("/arm_1/gripper/close", Empty, self.close_srv)
        rospy.Service("/arm_1/gripper/grasp", Empty, self.grasp_srv)
        rospy.Service("/arm_1/gripper/fakegrasp", Empty, self.fakegrasp_srv)
        rospy.Service("/arm_1/gripper/fakeopen", Empty, self.fakeopen_srv)

        # Serviços do Gazebo (attach / detach)
        rospy.wait_for_service("/link_attacher_node/attach")
        rospy.wait_for_service("/link_attacher_node/detach")

        self.attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
        self.detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)

        rospy.loginfo("Serviços do gripper prontos.")

    # --------------------------------------------------
    # CALLBACKS
    # --------------------------------------------------

    def joint_cb(self, msg):
        if "gripper_finger_joint_l" in msg.name:
            i = msg.name.index("gripper_finger_joint_l")
            self.left_pos = msg.position[i]

        if "gripper_finger_joint_r" in msg.name:
            i = msg.name.index("gripper_finger_joint_r")
            self.right_pos = msg.position[i]

    # --------------------------------------------------
    # CONTROLE BÁSICO
    # --------------------------------------------------

    def send_position(self, pos, time_sec):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [
            "gripper_finger_joint_l",
            "gripper_finger_joint_r"
        ]

        point = JointTrajectoryPoint()
        point.positions = [pos, pos]
        point.time_from_start = rospy.Duration(time_sec)

        goal.trajectory.points.append(point)
        self.client.send_goal(goal)

    # --------------------------------------------------
    # SERVIÇOS
    # --------------------------------------------------

    def open_srv(self, req):
        rospy.loginfo("Abrindo gripper")
        self.send_position(self.OPEN_POS, self.MOVE_TIME)
        self.client.wait_for_result()
        return EmptyResponse()

    def close_srv(self, req):
        rospy.loginfo("Fechando gripper")
        self.send_position(self.CLOSE_POS, self.MOVE_TIME)
        self.client.wait_for_result()
        return EmptyResponse()

    def grasp_srv(self, req):
        """
        Tentativa de grasp real baseada em stall da junta.
        OBS: não é confiável no Gazebo, mantido apenas para estudo.
        """
        rospy.loginfo("Executando GRASP com detecção de contato (stall)")

        self.prev_left = None
        self.prev_time = rospy.Time.now()
        stalled_time = 0.0

        self.send_position(self.CLOSE_POS, self.MOVE_TIME)

        rate = rospy.Rate(self.RATE)

        while not rospy.is_shutdown():
            # Se o controlador terminou o movimento
            if self.client.get_state() >= 3:
                rospy.logwarn("Gripper fechou totalmente. Nenhum objeto detectado.")
                return EmptyResponse()

            if self.left_pos is None:
                rate.sleep()
                continue

            now = rospy.Time.now()

            if self.prev_left is not None:
                delta = abs(self.left_pos - self.prev_left)

                if delta < self.POS_EPS:
                    stalled_time += (now - self.prev_time).to_sec()
                else:
                    stalled_time = 0.0

                if stalled_time > self.STALL_TIME:
                    rospy.loginfo("Contato detectado! Grasp bem sucedido.")
                    self.client.cancel_goal()
                    return EmptyResponse()

            self.prev_left = self.left_pos
            self.prev_time = now
            rate.sleep()

        return EmptyResponse()

    def fakegrasp_srv(self, req):
        """
        Grasp simbólico:
        - Fecha o gripper (visual)
        - Cria junta fixa entre TCP e objeto no Gazebo
        """
        if self.attached_object is not None:
            rospy.logwarn("Já existe um objeto preso ao gripper.")
            return EmptyResponse()

        rospy.loginfo(f"[FAKEGRASP] Pegando objeto: {self.OBJECT_NAME}")

        # Fecha gripper só para efeito visual
        self.grasp_srv(None)
        rospy.sleep(0.5)

        attach_req = AttachRequest()
        attach_req.model_name_1 = "youbot"
        attach_req.link_name_1 = "gripper_finger_link_l"
        attach_req.model_name_2 = self.OBJECT_NAME
        attach_req.link_name_2 = "block_link"

        try:
            self.attach_srv.call(attach_req)
            self.attached_object = self.OBJECT_NAME
            rospy.loginfo("[FAKEGRASP] Objeto preso com sucesso.")
        except rospy.ServiceException as e:
            rospy.logerr(f"[FAKEGRASP] Erro ao prender objeto: {e}")

        return EmptyResponse()

    def fakeopen_srv(self, req):
        """
        Solta o objeto preso:
        - Remove junta fixa
        - Abre o gripper
        """
        if self.attached_object is None:
            rospy.logwarn("Nenhum objeto preso ao gripper.")
            return EmptyResponse()

        rospy.loginfo(f"[FAKEOPEN] Soltando objeto: {self.attached_object}")

        detach_req = AttachRequest()
        detach_req.model_name_1 = "youbot"
        detach_req.link_name_1 = "gripper_finger_link_l"
        detach_req.model_name_2 = self.attached_object
        detach_req.link_name_2 = "block_link"

        try:
            self.detach_srv.call(detach_req)
            rospy.loginfo("[FAKEOPEN] Objeto solto com sucesso.")
        except rospy.ServiceException as e:
            rospy.logerr(f"[FAKEOPEN] Erro ao soltar objeto: {e}")

        rospy.sleep(0.2)
        self.open_srv(None)
        self.attached_object = None

        return EmptyResponse()


def main():
    rospy.init_node("gripper_node")
    YoubotGripperNode()
    rospy.spin()


if __name__ == "__main__":
    main()

