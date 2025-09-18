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
        if self.time < 2:
            msg = self.create_twist(0.0, 0.8) # makes a 45 degree angle
        # the following if/else statements are just from the rectangle
        elif self.time >= 3 and self.time < 8: 
            msg = self.create_twist(1.0, 0.0)
        elif self.time >= 9 and self.time < 11:
            msg = self.create_twist(0.0, 1.6)
        elif self.time >= 12 and self.time < 17:
            msg = self.create_twist(1.0, 0.0)
        elif self.time >= 18 and self.time < 20:
            msg = self.create_twist(0.0, 1.6)
        elif self.time >= 21 and self.time < 26:
            msg = self.create_twist(1.0, 0.0)
        elif self.time >= 27 and self.time < 29:
            msg = self.create_twist(0.0, 1.6)
        elif self.time >= 30 and self.time < 35:
            msg = self.create_twist(1.0, 0.0)
        else:
            # stop moving when done
            msg = self.create_twist(0.0, 0.0)
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