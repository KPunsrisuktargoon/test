import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
class DetectTunnel():
    def __init__(self):
        self.sub_arrival_status = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.cbGetNavigationResult, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)

        self.pub_tunnel_return = rospy.Publisher('/detect/tunnel_stamped', UInt8, queue_size=1)
        self.pub_goal_pose_stamped = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pub_init_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size = 1)
        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)

        self.StepOfTunnel = Enum('StepOfTunnel', 'searching_tunnel_sign go_in_to_tunnel navigation go_out_from_tunnel exit')

        self.is_navigation_finished = False
        self.is_tunnel_finished = False

        self.last_current_theta = 0.0

   def fnPubGoalPose(self):
        goalPoseStamped = PoseStamped()

        goalPoseStamped.header.frame_id = "map"
        goalPoseStamped.header.stamp = rospy.Time.now()

        goalPoseStamped.pose.position.x = 0.15
        goalPoseStamped.pose.position.y = -1.76
        goalPoseStamped.pose.position.z = 0.0

        goalPoseStamped.pose.orientation.x = 0.0
        goalPoseStamped.pose.orientation.y = 0.0
        goalPoseStamped.pose.orientation.z = 0.0
        goalPoseStamped.pose.orientation.w = 1.0

        self.pub_goal_pose_stamped.publish(goalPoseStamped)
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_tunnel')
    node = DetectTunnel()
    node.main()