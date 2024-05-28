import math
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

class Bug0(Node):
    #Initialization
    def __init__(self):
        super().__init__('bugx')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.r_sensor_sub = self.create_subscription(Range, '/r_range_sensor', self.r_sensor_callback, 10)
        self.l_sensor_sub = self.create_subscription(Range, '/l_range_sensor', self.l_sensor_callback, 10)
        self.f_sensor_sub = self.create_subscription(Range, '/f_range_sensor', self.f_sensor_callback, 10)       
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)
        self.goal_x = 0
        self.goal_y = 0
        self._state=0
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.fl_sensor_value = 0.0
        self.fr_sensor_value = 0.0
        self.l_sensor_value=0.0
        self.r_sensor_value=0.0
        self.f_sensor_value=0.0
        self.cmd_vel_msg = Twist()
        self.state="GO_TO_GOAL"

    #Method for goal update
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    #Method for robot current position
    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.current_theta = transform.euler_from_quaternion(q)[2] #[-pi, pi]

    #Methods for updating sensor values
    def fl_sensor_callback(self, msg):
        self.fl_sensor_value = msg.range       #front left

    def fr_sensor_callback(self, msg):         #front right
        self.fr_sensor_value = msg.range
    
    def r_sensor_callback(self, msg):          
        self.r_sensor_value = msg.range

    def l_sensor_callback(self, msg):
        self.l_sensor_value = msg.range

    def f_sensor_callback(self, msg):          #front
        self.f_sensor_value = msg.range

    

    def bug_algorithm_callback(self):
        if self.goal_x is None or self.goal_y is None:
            return  # No goal set yet
        print("Current X: " , self.current_x)
        print("Current Y: " , self.current_y)
        print("Current theta: " , self.current_theta)
        print("FL Sensor: " , self.fl_sensor_value)
        print("FR Sensor: " , self.fr_sensor_value)
        print("F Sensor: ", self.f_sensor_value)
        print("Goal X: " , self.goal_x)
        print("Goal Y: " , self.goal_y)

        if self.state == "GO_TO_GOAL":
            self.go_to_goal()
        elif self.state == "FOLLOW_WALL":
            self.follow_wall()

    
    def go_to_goal(self):
        if self.goal_is_reached():
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel_msg)
            print('Goal reached')
            return
        else:
            self.state="FOLLOW_WALL"
        angle_to_goal = math.atan2(self.goal_y-self.current_y, self.goal_x-self.current_x)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_theta)
        self.cmd_vel_msg.linear.x = 0.15
        self.cmd_vel_msg.angular.z = 0.5* angle_diff
        self.cmd_pub.publish(self.cmd_vel_msg)


    def follow_wall(self):
        if self.f_sensor_value > 0.5 and self.fl_sensor_value > 0.5 and self.fr_sensor_value > 0.5:
            self.state = "GO_TO_GOAL"

        elif self.f_sensor_value < 0.5 and self.fl_sensor_value > 0.5 and self.fr_sensor_value > 0.5:
            self.cmd_vel_msg.angular.z = 0.9
            self.cmd_pub.publish(self.cmd_vel_msg)

        elif self.f_sensor_value > 0.5 and self.fl_sensor_value > 0.5 and self.fr_sensor_value < 0.5:
            self.cmd_vel_msg.linear.x = 0.15
            self.cmd_pub.publish(self.cmd_vel_msg)

        elif self.f_sensor_value > 0.5 and self.fl_sensor_value < 0.5 and self.fr_sensor_value > 0.5:
            self.cmd_vel_msg.linear.x=0.15
            self.cmd_vel_msg.angular.z = -0.9
            self.cmd_pub.publish(self.cmd_vel_msg)

        elif self.f_sensor_value <0.5 and self.fl_sensor_value > 0.5 and self.fr_sensor_value < 0.5:
            self.cmd_vel_msg.angular.z = 0.9
            self.cmd_pub.publish(self.cmd_vel_msg)

        elif self.f_sensor_value < 0.5 and self.fl_sensor_value < 0.5 and self.fr_sensor_value > 0.5:
            self.cmd_vel_msg.angular.z = 0.9
            self.cmd_pub.publish(self.cmd_vel_msg)

        elif self.f_sensor_value < 0.5 and self.fl_sensor_value < 0.5 and self.fr_sensor_value < 0.5:
            self.cmd_vel_msg.angular.z = 0.9
            self.cmd_pub.publish(self.cmd_vel_msg)


        elif self.f_sensor_value > 0.5 and self.fl_sensor_value < 0.5 and self.fr_sensor_value < 0.5:
            self.cmd_vel_msg.linear.x=0.15
            self.cmd_vel_msg.angular.z = -0.9
            self.cmd_pub.publish(self.cmd_vel_msg)




    def normalize_angle(self,angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle


    
    def goal_is_reached(self):
        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        return distance_to_goal < 0.2
       
def main(args=None):

    rclpy.init(args=args)
    bug_node = Bug0()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
