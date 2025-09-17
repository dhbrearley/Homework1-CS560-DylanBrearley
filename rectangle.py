import rclpy # ros2 library for python
from rclpy.node import Node # turtle controller will be an instance of Node
from geometry_msgs.msg import Twist # library that sends messages to robot, controls velocity and rotation

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller') # gives the controller a name

        # vel is the turtle's movement channel, this creates a publisher and sends the Twist messages to vel
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) # queue size is 10
        timer_period = 0.5  # seconds

        #self.timer creates an interrupt like system that calls timer_callback every 0.5 sec
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0

    def create_twist(self, linear_x, angular_z): # helper function that creates a new Twist object
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def get_twist_msg(self): # rotate and move forward after certain time intervals
        if self.time < 10:
            msg = self.create_twist(1.0, 0.0) # call create_twist function and pass velo and angle
        elif self.time >= 10 and self.time < 12:
            msg = self.create_twist(0.0, 1.6)
        elif self.time >= 12 and self.time < 15:
            msg = self.create_twist(1.0, 0.0)
        elif self.time >= 15 and self.time < 17:
            msg = self.create_twist(0.0, 1.522)
        elif self.time >= 17 and self.time < 27:
            msg = self.create_twist(1.0, 0.0)
        elif self.time >= 27 and self.time < 29:
            msg = self.create_twist(0.0, 1.6)
        elif self.time >= 29 and self.time < 32:
            msg = self.create_twist(1.0, 0.0)
        else:
            msg = self.create_twist(0.0, 0.0)
        return msg
   
    def timer_callback(self): # function that runs every 0.5 seconds
        msg = self.get_twist_msg()      
        self.publisher.publish(msg)
        self.time += 1
        print("time: {}".format(self.time))

def main(args=None):
    rclpy.init(args=args) #initialize ros2 sys

    turtle_controller = TurtleController()

    rclpy.spin(turtle_controller) # basically start the program and start using the timer functionality

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()