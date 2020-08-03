import yaml
import sensor_msgs.msg
import trajectory_msgs.msg
import rospy
import control_msgs
import control_msgs.msg

afile = open('/home/iskander/ws_moveit/src/trajectory1.yaml','r')
content = yaml.load(afile)


traj_msg = trajectory_msgs.msg.JointTrajectory()
traj_msg.header.frame_id = content["joint_trajectory"]["header"]["frame_id"]
traj_msg.joint_names = content['joint_trajectory']['joint_names']

for point in content['joint_trajectory']['points']:
	new_point = trajectory_msgs.msg.JointTrajectoryPoint()
	new_point.positions = point['positions']
	new_point.velocities = point['velocities']
	new_point.accelerations = point['accelerations']
	new_point.time_from_start.secs = point["time_from_start"]["secs"]
	new_point.time_from_start.nsecs = point["time_from_start"]["nsecs"]
	traj_msg.points.append(new_point)

print traj_msg

goal = control_msgs.msg.FollowJointTrajectoryActionGoal()

goal.goal.trajectory = traj_msg


pub = rospy.Publisher('/asp/controller/trajectory/follow_joint_trajectory/goal', control_msgs.msg.FollowJointTrajectoryActionGoal, queue_size = 1)
rospy.init_node('node_name')
pub.publish(goal)

