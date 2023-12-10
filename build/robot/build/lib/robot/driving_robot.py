import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from geometry_msgs.msg import Twist # Twist añadido porque es el tipo de mensaje que recibe cmd_vel

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('vel_publisher')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0



    # def timer_callback(self):
    #     msg = Twist()

    #     # msg.linear.x = 0.8  # Set an appropriate linear velocity
    #     msg.angular.z = -0.5  # Set an appropriate angular velocity

    #     self.vel_pub.publish(msg)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0  # Ajusta a la velocidad lineal deseada
        msg.angular.z = 1.5  # Ajusta a la velocidad angular deseada

        print(f"Publishing Twist message: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        self.vel_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()