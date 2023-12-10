import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from geometry_msgs.msg import Twist # Twist añadido porque es el tipo de mensaje que recibe cmd_vel
from geometry_msgs.msg import Point # Point añadido porque es el tipo de mensaje que recibe odom
from nav_msgs.msg import Odometry
import sys
import math

class robot_go_to_goal(Node):

    def __init__(self):
        super().__init__('goal_movement_node', parameter_overrides=[])
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.02  # seconds

        self.pose_sub = self.create_subscription(
            Odometry,'/odom', self.pose_callback, 10)
        # self.pose_sub  # prevent unused variable warning

        self.timer = self.create_timer(
            timer_period, self.go_to_goal_function)

        self.robot_pose = Point() # robot_pose es un Point que contiene la posición y orientación del robot
        self.goal_pose = Point() # goal_pose es un Point que contiene la posición y orientación del goal
        self.vel_msg = Twist() # vel_msg es un Twist que contiene la velocidad lineal y angular del robot
        self.distance_to_goal = 0.0 # distance_to_goal es un float de la distancia entre el robot y el goal
        self.angle_to_goal = 0.0 # angle_to_goal es un float del ángulo entre el robot y el goal
        self.angle_offset = 0.0 # angle_offset es un float del ángulo de offset del robot, lo que sería el ángulo inicial del robot
        self.angle_to_turn = 0.0 # angle_to_turn es un float del ángulo que tiene que girar el robot para estar orientado hacia el goal



    def pose_callback(self, data):
        self.robot_pose.x = data.pose.pose.position.x # robot_pose es un float de la posición x del robot dada en movimiento pose
        self.robot_pose.y = data.pose.pose.position.y # robot_pose es un float de la posición y del robot dada en movimiento pose

        quaternion = data.pose.pose.orientation # quaternion es un quaternion que contiene la orientación del robot dada en movimiento pose

        (roll, pitch, yaw) = self.euler_from_quaternion(
            quaternion.x, quaternion.y, quaternion.z, quaternion.w) # euler_from_quaternion es una función que convierte el quaternion en ángulos de euler
        self.robot_pose.z = yaw # robot_pose es un float de la orientación z del robot dada en movimiento pose


    def go_to_goal_function(self):
        self.goal_pose.x = float(sys.argv[1]) # goal_pose es un float de la posición x del goal, lo que sería Xfinal
        self.goal_pose.y = float(sys.argv[2]) # goal_pose es un float de la posición y del goal, lo que sería Yfinal
        self.angle_offset = float(sys.argv[3]) # angle_offset es un float del ángulo de offset del robot, lo que sería el ángulo inicial del robot

        # distance_to_goal es un float de la distancia entre el robot y el goal sqrt((Xfinal - Xinicial)**2 + (Yfinal - Yinicial)**2)
        self.distance_to_goal = math.sqrt(pow((self.goal_pose.x - self.robot_pose.x), 2) + pow((self.goal_pose.y - self.robot_pose.y), 2))

        self.angle_to_goal = math.atan2((self.goal_pose.y - self.robot_pose.y), (self.goal_pose.x - self.robot_pose.x)) + self.angle_offset
        self.angle_to_turn = self.angle_to_goal - self.robot_pose.z

        if abs(self.angle_to_turn) > 0.1:
            self.vel_msg.linear.x = 0.0 # si el robot no está orientado hacia el goal, no avanza hacia el goal
            self.vel_msg.angular.z = self.angle_to_turn # si el robot no está orientado hacia el goal, gira hacia el goal
        else:
            self.vel_msg.linear.x = self.distance_to_goal # si el robot está orientado hacia el goal, avanza hacia el goal
            self.vel_msg.angular.z = 0.0 # si el robot está orientado hacia el goal, no gira hacia el goal

        msg = 'Distance to goal:{:2f} Angle to turn:{:3f}'.format(self.distance_to_goal, self.angle_to_turn)
        self.get_logger().info(msg)
        self.vel_publisher.publish(self.vel_msg)




    # def go_to_goal_function(self):

    #     self.goal_pose.x = float(sys.argv[1]) # goal_pose es un float de la posición x del goal, lo que sería Xfinal
    #     self.goal_pose.y = float(sys.argv[2]) # goal_pose es un float de la posición y del goal, lo que sería Yfinal
    #     self.angle_offset = float(sys.argv[3]) # angle_offset es un float del ángulo de offset del robot, lo que sería el ángulo inicial del robot

    #     vel_msg = Twist()

    #     self.distance_to_goal = math.sqrt(pow((self.goal_pose.x - self.robot_pose.x), 2) + pow((self.goal_pose.y - self.robot_pose.y), 2)) # distancia entre el robot y el goal sqrt((Xfinal - Xinicial)**2 + (Yfinal - Yinicial)**2)
    #     self.angle_to_goal = math.atan2((self.goal_pose.y - self.robot_pose.y), (self.goal_pose.x - self.robot_pose.x)) + self.angle_offset # ángulo entre el robot y el goal atan2((Yfinal - Yinicial), (Xfinal - Xinicial)) + offset

    #     self.angle_to_turn = self.angle_to_goal - self.robot_pose.z # ángulo que tiene que girar el robot para estar orientado hacia el goal

    #     if abs(self.angle_to_turn) > 0.1:
    #         self.vel_msg.linear.x = 0.0 # si el robot no está orientado hacia el goal, no avanza hacia el goal
    #         self.vel_msg.angular.z = self.angle_to_turn # si el robot no está orientado hacia el goal, gira hacia el goal

    #     else:
    #         self.vel_msg.linear.x = self.distance_to_goal
    #         self.vel_msg.angular.z = 0.0

    #     msg = 'Distance to goal:{:2f} Angle to turn:{:3f}'.format(self.distance_to_goal, self.angle_to_turn) # msg es un string que contiene la distancia y el ángulo entre el robot y el goal
    #     self.get_logger().info(msg) # imprime el mensaje en la terminal
    #     self.vel_publisher.publish(self.vel_msg) # publica el mensaje en el topic /cmd_vel



    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw



def main(args=None):
    rclpy.init(args=args)

    gtg_node = robot_go_to_goal()

    rclpy.spin(gtg_node)
    gtg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()