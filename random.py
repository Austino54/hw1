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
        if self.time < 6: # go forward
            msg = self.create_twist(1.0, 0.0) 
        elif self.time >= 7 and self.time < 8: # turn around
            msg = self.create_twist(0.0, 6.4)
        elif self.time >= 9 and self.time < 12: # go halfway back
            msg = self.create_twist(1.0, 0.0)
        elif self.time >= 13 and self.time < 14: # turn 90 degrees
            msg = self.create_twist(0.0, 3.2)
        elif self.time >= 15 and self.time < 18: # go half of initial distance forward
            msg = self.create_twist(1.0, 0.0)
        elif self.time >= 19 and self.time < 21: # turn around
            msg = self.create_twist(0.0, 3.2)
        elif self.time >= 22 and self.time < 28: # go full distance forward
            msg = self.create_twist(1.0, 0.0)
        elif self.time >= 29 and self.time < 31: # turn around
            msg = self.create_twist(0.0, 3.2)
        elif self.time >= 32 and self.time < 35: # go half distance forward
            msg = self.create_twist(1.0, 0.0)
        elif self.time >= 36 and self.time < 39: # turn 270 degree the same way (facing towards start, accounting for error)
            msg = self.create_twist(0.0, 3.0)
        elif self.time >= 40 and self.time < 43: # go half distance forward to the origin
            msg = self.create_twist(1.0, 0.0)
        else:
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

if __name__ == '__main__':
    main()