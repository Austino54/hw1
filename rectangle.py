import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # calls timer_callback every half second
        self.time = 0

    def create_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    # called every half a second
    def get_twist_msg(self):
        # 2 seconds of 1.6 angular velocity is a close approximation of a 90 degree turn
        if self.time < 5:
            msg = self.create_twist(1.0, 0.0) # line
        elif self.time >= 6 and self.time < 8:
            msg = self.create_twist(0.0, 1.6) # turn
        elif self.time >= 10 and self.time < 15:
            msg = self.create_twist(1.0, 0.0) # line
        elif self.time >= 16 and self.time < 18:
            msg = self.create_twist(0.0, 1.6) # turn
        elif self.time >= 19 and self.time < 24:
            msg = self.create_twist(1.0, 0.0) # line
        elif self.time >= 25 and self.time < 27:
            msg = self.create_twist(0.0, 1.6) # turn
        elif self.time >= 28 and self.time < 33:
            msg = self.create_twist(1.0, 0.0) # line
        else:
            msg = self.create_twist(0.0, 0.0) # stop
        return msg
    
    # runs every half a second
    def timer_callback(self):
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        self.time += 1
        print("time: {}".format(self.time))

def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController()

    # start
    rclpy.spin(turtle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()