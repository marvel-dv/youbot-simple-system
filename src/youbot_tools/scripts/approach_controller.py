#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from youbot_tools.srv import ApproachTag, ApproachTagResponse


"""
Exemplo:
rosservice call /approach_tag "desired_distance: 0.4"
"""

class TagPositionController:
    def __init__(self):
        rospy.init_node("tag_position_controller")

        # ======================
        # Parâmetros
        # ======================
        self.Kx = rospy.get_param("~Kx", 0.8)
        self.Ktheta = rospy.get_param("~Ktheta", 1.5)

        self.max_v = rospy.get_param("~max_linear_velocity", 0.3)
        self.max_w = rospy.get_param("~max_angular_velocity", 0.8)

        self.x_tol = rospy.get_param("~x_tolerance", 0.02)
        self.theta_tol = rospy.get_param("~theta_tolerance", 0.05)

        self.rate = rospy.Rate(20)

        # ======================
        # Estado interno
        # ======================
        self.tag_pose = None
        self.active = False
        self.x_des = 0.4

        # ======================
        # ROS I/O
        # ======================
        rospy.Subscriber(
            "/apriltag/pose_target",
            PoseStamped,
            self.tag_callback,
            queue_size=1
        )

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.service = rospy.Service(
            "/approach_tag",
            ApproachTag,
            self.handle_service
        )

        rospy.loginfo("Tag Position Controller ready.")

    # ======================
    # Callback da tag
    # ======================
    def tag_callback(self, msg):
        # A pose já está no frame base_link
        self.tag_pose = msg

    # ======================
    # Serviço
    # ======================
    def handle_service(self, req):
        rospy.loginfo("Approach tag requested (d = %.2f m)", req.desired_distance)

        # Espera até 2s por uma pose válida
        timeout = rospy.Time.now() + rospy.Duration(2.0)
        while self.tag_pose is None and rospy.Time.now() < timeout:
            rospy.sleep(0.05)

        if self.tag_pose is None:
            return ApproachTagResponse(False, "No tag detected")

        self.x_des = req.desired_distance
        self.active = True
        
        try:
            while not rospy.is_shutdown() and self.active:
                self.control_step()
                self.rate.sleep()

            if not rospy.is_shutdown():
                self.publish_stop()
                return ApproachTagResponse(True, "Target position reached")
                
        except rospy.ROSException as e:
            rospy.logerr("ROS error during approach: %s", str(e))
            self.publish_stop()
            self.active = False
            return ApproachTagResponse(False, "ROS time error / simulation reset")
            
        except Exception as e:
            rospy.logerr("Unexpected error: %s", str(e))
            self.publish_stop()
            self.active = False
            return ApproachTagResponse(False, "Unexpected controller error")
            
        finally:
            # Garantia absoluta de parada
            self.publish_stop()
            self.active = False

    # ======================
    # Controle
    # ======================
    def control_step(self):
        if self.tag_pose is None:
            rospy.logwarn("Lost tag during approach")
            self.active = False
            return

        x = self.tag_pose.pose.position.x
        y = self.tag_pose.pose.position.y

        # Erros
        ex = x - self.x_des
        etheta = math.atan2(y, x)

        # Condição de parada
        if abs(ex) < self.x_tol and abs(etheta) < self.theta_tol:
            rospy.loginfo("Target reached")
            self.active = False
            return

        # Controle proporcional
        v = self.Kx * ex
        w = self.Ktheta * etheta

        # Saturação
        v = max(-self.max_v, min(self.max_v, v))
        w = max(-self.max_w, min(self.max_w, w))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    # ======================
    # Utilitário
    # ======================
    def publish_stop(self):
        self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    controller = TagPositionController()
    rospy.spin()

