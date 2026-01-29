#!/usr/bin/env python3
import rospy
import tf.transformations as tftr

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import Int32

from tf.transformations import quaternion_from_euler, quaternion_multiply
import math


from youbot_tools.srv import (
    ApproachTag,
    GoToPoseMoveIt
)


"""
rosservice call /execute_pick
"""
# =====================================================
# Pick Supervisor
# =====================================================
class PickSupervisor:

    def __init__(self):
        rospy.init_node("pick_supervisor")

        rospy.loginfo("[PICK SUPERVISOR] Initializing...")
        self.pose_target = None
        
        self.pub_id = rospy.Publisher("/filter",Int32,queue_size=2,latch=True)        


        # Subscriber da AprilTag
        rospy.Subscriber(
            "/apriltag/pose_target",
            PoseStamped,
            self.pose_cb,
            queue_size=1
        )

        # --------------------------
        # Esperar serviços
        # --------------------------
        rospy.wait_for_service("/approach_tag")
        rospy.wait_for_service("/youbot/go_to_pose_moveit")
        rospy.wait_for_service("/arm_1/gripper/fakegrasp")
        rospy.wait_for_service("/arm_1/gripper/fakeopen")
        rospy.wait_for_service("/arm_1/gripper/open")

        self.approach_srv = rospy.ServiceProxy("/approach_tag", ApproachTag)
        self.moveit_srv = rospy.ServiceProxy("/youbot/go_to_pose_moveit", GoToPoseMoveIt)
        self.fakegrasp_srv = rospy.ServiceProxy("/arm_1/gripper/fakegrasp", Empty)
        self.fakeopen_srv = rospy.ServiceProxy("/arm_1/gripper/fakeopen", Empty)
        self.open_srv = rospy.ServiceProxy("/arm_1/gripper/open", Empty)

        # --------------------------
        # Serviço supervisor
        # --------------------------
        self.exec_srv = rospy.Service(
            "/execute_pick",
            Trigger,
            self.handle_execute_pick
        )

        rospy.loginfo("[PICK SUPERVISOR] READY - service /execute_pick")

    # =================================================
    # Callback da pose da tag
    # =================================================
    def pose_cb(self, msg):
        """
        Callback que recebe a pose do apriltag já em base_link
        """
        self.pose_target = msg

    # =================================================
    # Utilitário
    # =================================================
    def quat_to_rpy(self, Q):
        return tftr.euler_from_quaternion(Q)
        
    def rot_quat(self, qx, qy, qz, qw):
        q_tag = [qx, qy, qz, qw]
        q_rot = quaternion_from_euler(math.pi/2, 0, 0)
        q_new = quaternion_multiply(q_tag, q_rot)   
        return q_new

    def move_arm(self, x, y, z, roll, pitch, yaw):
        resp = self.moveit_srv(x, y, z, roll, pitch, yaw)
        if not resp.success:
            rospy.logerr("[MOVEIT] " + resp.message)
        return resp.success

    def fail(self, msg):
        rospy.logerr("[PICK SUPERVISOR] " + msg)
        return TriggerResponse(False, msg)
        
    def publish_tag_id(self, tag_id):
        # Utilitário para publicar ID da tag
        msg = Int32()
        msg.data = tag_id
        self.pub_id.publish(msg)
        rospy.loginfo(f"[FILTER] Publicando tag ID: {tag_id}")
        rospy.sleep(0.05)
        
        
    def execute_with_retry(self, func, step_name, max_attempts=3, delay=0.5):
        """
        Executa uma função com múltiplas tentativas
        """
        for attempt in range(1, max_attempts + 1):
            try:
                rospy.loginfo(f"[PICK SUPERVISOR] Pick attempt {attempt}/{max_attempts}")
                return func()
            except Exception as e:
                if attempt < max_attempts - 1:
                    rospy.logwarn(f"[{step_name}] Attempt {attempt+1} failed: {str(e)}")
                    rospy.loginfo(f"[{step_name}] Retrying in {delay} seconds...")
                    rospy.sleep(delay)
                else:
                    raise Exception(f"{step_name} failed after {max_attempts} attempts: {str(e)}")
                return None
                    
 

    # =================================================
    # Serviço principal
    # =================================================

    def handle_execute_pick(self, req):
        rospy.loginfo("[PICK SUPERVISOR] Pick requested")
        
        max_overall_attempts = 3
        
        for overall_attempt in range(1, max_overall_attempts + 1):
            rospy.loginfo(f"\n{'='*50}")
            rospy.loginfo(f"[PICK SUPERVISOR] Overall attempt {overall_attempt}/{max_overall_attempts}")
            rospy.loginfo(f"{'='*50}")
            
            try:
                # Resetar para nova tentativa
                self.pose_target = None
                
                # Esperar pose da tag
                # ---------------------------------
                # 0) Configuração inicial e espera da pose alvo
                # ---------------------------------
                self.publish_tag_id(1)

                timeout = rospy.Time.now() + rospy.Duration(2.0)
                while self.pose_target is None and rospy.Time.now() < timeout:
                    rospy.sleep(0.05)

                if self.pose_target is None:
                    raise Exception("No AprilTag pose received")

                pos = self.pose_target.pose.position
                ori = self.pose_target.pose.orientation

                # ---------------------------------
                # 1) Aproximar da tag (base)
                # ---------------------------------
                
                
                rospy.loginfo("[STEP 1] Approaching tag")
                resp = self.approach_srv(desired_distance=0.4)
                if not resp.success:
                    raise Exception("Approach failed: " + resp.message)

                # ---------------------------------
                # 2) Pose inicial da tarefa
                # ---------------------------------
                rospy.loginfo("[STEP 2] Going to start pose")
                q_new = [0,1,0,0]
                roll, pitch, yaw = self.quat_to_rpy(q_new)
                
                if not self.move_arm(
                    x=0.3, y=0.0, z=0.15,
                    roll=roll, pitch=pitch, yaw=yaw
                ):
                    raise Exception("Start pose failed")

                # ---------------------------------
                # 3) Pre-grasp alto
                # ---------------------------------
                rospy.loginfo("[STEP 3] Pre-grasp high")
                
                q_new = self.rot_quat(ori.x, ori.y, ori.z, ori.w)
                roll, pitch, yaw = self.quat_to_rpy(q_new)
                
                max_inner_attempts = 3
                step3_success = False
                self.open_srv()
                for inner_attempt in range(max_inner_attempts):
                    rospy.loginfo(f"Inner attempt {inner_attempt+1}/{max_inner_attempts}")
                    if self.move_arm(
                        x=pos.x-0.01,
                        y=pos.y,
                        z=pos.z + 0.11,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw
                    ):      
                        rospy.loginfo("Success")
                        step3_success = True
                        break
                    elif inner_attempt < max_inner_attempts - 1:
                        rospy.logwarn(f"Attempt {inner_attempt+1} failed, retrying...")
                        rospy.sleep(0.2)
                
                if not step3_success:
                    raise Exception("Pre-grasp high failed after all attempts")

                # ---------------------------------
                # 4) Pre-grasp baixo
                # ---------------------------------
                rospy.loginfo("[STEP 4] Pre-grasp low")
                
                q_new = self.rot_quat(ori.x, ori.y, ori.z, ori.w)
                roll, pitch, yaw = self.quat_to_rpy(q_new)
                             
                step4_success = False
                for inner_attempt in range(max_inner_attempts):
                    rospy.loginfo(f"Inner attempt {inner_attempt+1}/{max_inner_attempts}")
                    if self.move_arm(
                        x=pos.x-0.01,
                        y=pos.y,
                        z=pos.z + 0.09,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw
                    ):      
                        rospy.loginfo("Success")
                        step4_success = True
                        break
                    elif inner_attempt < max_inner_attempts - 1:
                        rospy.logwarn(f"Attempt {inner_attempt+1} failed, retrying...")
                        rospy.sleep(0.2)
                
                if not step4_success:
                    raise Exception("Pre-grasp low failed after all attempts")

                # ---------------------------------
                # 5) Fake grasp
                # ---------------------------------
                rospy.loginfo("[STEP 5] Grasping object")
                self.fakegrasp_srv()
                rospy.sleep(0.25)

                # ---------------------------------
                # 6) Retorno seguro
                # ---------------------------------
                rospy.loginfo("[STEP 6] Returning to safe pose")
                q_new = [0,1,0,0]
                roll, pitch, yaw = self.quat_to_rpy(q_new)
                
                if not self.move_arm(
                    x=0.3, y=0.0, z=0.15,
                    roll=roll, pitch=pitch, yaw=yaw
                ):
                    raise Exception("Return pose failed")

                # ---------------------------------
                # 7) Ir em direção ao marcador
                # ---------------------------------              
                self.publish_tag_id(2)
                
                rospy.loginfo("[STEP 7] Approaching tag marker")
                resp = self.approach_srv(desired_distance=0.4)
                if not resp.success:
                    raise Exception("Approach failed: " + resp.message)
                
                # ---------------------------------
                # 8) Posicionar o bloco na posição
                # ---------------------------------
                rospy.loginfo("[STEP 8] Positioning")
                #q_new = self.rot_quat(ori.x, ori.y, ori.z, ori.w)
                roll, pitch, yaw = self.quat_to_rpy(q_new)
                
                if not self.move_arm( 
                    x=0.4,
                    y=0,
                    z=0.1,
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw
                ):
                    
                    raise Exception("Place pose failed")
                
                # ---------------------------------
                # 9) Largar o bloco
                # ---------------------------------
                rospy.loginfo("[STEP 9] Releasing object")
                rospy.sleep(1)
                self.fakeopen_srv()
                
                # ---------------------------------
                # 10) Posição inicial
                # ---------------------------------
                rospy.loginfo("[STEP 10] Returning to safe pose")
                q_new = [0,1,0,0]
                roll, pitch, yaw = self.quat_to_rpy(q_new)
                
                if not self.move_arm(
                    x=0.3, y=0.0, z=0.15,
                    roll=roll, pitch=pitch, yaw=yaw
                ):
                    raise Exception("Final return pose failed")
                

                rospy.loginfo(f"[PICK SUPERVISOR] Pick executed successfully on attempt {overall_attempt}")
                return TriggerResponse(True, f"Pick completed successfully (attempt {overall_attempt})")
                
            except Exception as e:
                rospy.logerr(f"[ATTEMPT {overall_attempt}] Failed: {str(e)}")
                
                if overall_attempt < max_overall_attempts:
                    rospy.loginfo(f"Waiting 1 seconds before next attempt...")
                    rospy.sleep(1.0)
                else:
                    rospy.logerr(f"All {max_overall_attempts} attempts failed")
                    return self.fail(f"Pick failed after {max_overall_attempts} attempts: {str(e)}")
        
        # Nunca deve chegar aqui, mas por segurança:
        return self.fail("Unexpected error in pick execution")
# =====================================================
# Main
# =====================================================
if __name__ == "__main__":
    PickSupervisor()
    rospy.spin()

