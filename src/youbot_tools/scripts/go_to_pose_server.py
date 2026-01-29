#!/usr/bin/env python3
import rospy
import tf.transformations as tftr
from youbot_tools.srv import GoToPose, GoToPoseResponse
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


"""
Obsolete
"""
def solve_ik_from_pose(group_name, pose):
    rospy.wait_for_service('/compute_ik')
    compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    req = GetPositionIKRequest()
    req.ik_request.group_name = group_name
    req.ik_request.pose_stamped = pose
    req.ik_request.avoid_collisions = False

    resp = compute_ik(req)
    if resp.error_code.val != 1:
        return None
    
    arm_joints = ["arm_joint_1","arm_joint_2","arm_joint_3",
                  "arm_joint_4","arm_joint_5"]

    joint_map = dict(zip(resp.solution.joint_state.name,
                         resp.solution.joint_state.position))

    return [joint_map[j] for j in arm_joints]
    
    
pub = None


def move_arm(joints):
    global pub
    if pub is None:
        pub = rospy.Publisher("/arm_1/arm_controller/command", JointTrajectory, queue_size=1)
        rospy.sleep(0.5)
        
    traj = JointTrajectory()
    traj.joint_names = ["arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"]
    pt = JointTrajectoryPoint()
    pt.positions = joints
    pt.time_from_start = rospy.Duration(3.0)
    
    traj.points.append(pt)
    pub.publish(traj)
    rospy.loginfo("Trajectory published")

def handle_go_to_pose(req):
    rospy.loginfo("Request received: x=%.3f y=%.3f z=%.3f", req.x, req.y, req.z)
    # compose PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = req.x
    pose.pose.position.y = req.y
    pose.pose.position.z = req.z
    quat = tftr.quaternion_from_euler(req.roll, req.pitch, req.yaw)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    # solve IK 
    joints = solve_ik_from_pose("arm_1", pose)
    if joints is None:
        rospy.loginfo("IK falhou, posição invalida")
        return GoToPoseResponse(False, "IK failed")


    # move
    move_arm(joints)
    return GoToPoseResponse(True, "Motion started")

def go_server():
    rospy.init_node('go_to_pose_server')
    s = rospy.Service('/youbot/go_to_pose', GoToPose, handle_go_to_pose)
    rospy.loginfo("Service /youbot/go_to_pose ready")
    rospy.spin()

if __name__ == "__main__":
    go_server()

