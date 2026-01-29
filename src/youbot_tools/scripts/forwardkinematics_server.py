#!/usr/bin/env python3
import rospy
from youbot_tools.srv import ForwardKinematics, ForwardKinematicsResponse
from moveit_commander import MoveGroupCommander, RobotCommander

def fk_callback(req):
    try:
        #roscpp_initialize(sys.argv)
        
        robot = RobotCommander()
        group = MoveGroupCommander("arm_1")

        #Aplica as juntas recebidas
        joints = [req.joint1, req.joint2, req.joint3, req.joint4, req.joint5]
        group.set_joint_value_target(joints)

        # Planeja 
        #group.plan()
        
        # Planeja 
        group.go(wait=True)
        #group.stop()

        # Pega a pose resultante
        pose = group.get_current_pose().pose

        return ForwardKinematicsResponse(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            "FK calculada com sucesso",
            True
        )

    except Exception as e:
        return ForwardKinematicsResponse(
            0, 0, 0, 0, 0, 0, 1,
            f"Erro: {str(e)}",
            False
        )

def main():
    rospy.init_node("forward_kinematics_server")
    srv = rospy.Service("/youbot/forward_kinematics", ForwardKinematics, fk_callback)
    rospy.loginfo("Serviço de Forward Kinematics ativo ")
    rospy.spin()

if __name__ == "__main__":
    main()



