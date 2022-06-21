from unittest import result
from pkg_resources import resource_listdir
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from rclpy.parameter import Parameter
from rclpy.action import ActionServer

from cnc_interface.cnc_class import cnc
from cnc_msgs.msg import CncPosition
from cnc_msgs.action import CncMoveTo





class CNCInterfaceNode(Node):
    def __init__(self):
        super().__init__('cnc_interface_node')

        # Create and initialise cnc object
        self.cnc_obj = cnc()
        self.declare_param()
        self.cnc_init()

        # Action server
        self.action_server = ActionServer(
            self,
            CncMoveTo,
            'cnc_action',
            self.action_cb)

        # Publishers
        self.pos_pub = self.create_publisher(Twist,'/cnc_interface/position', 10)
        self.status_pub = self.create_publisher(String, '/cnc_interface/status', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, 'cnc_interface/pos', self.cmd_callback, 10)
        self.stop_cmd_sub = self.create_subscription(String, 'cnc_interface/cmd', self.stop_callback, 10)

        # timer callback
        timer_period = 0.1 # 10hz
        self.timer = self.create_timer(timer_period, self.loop)

        # Action stuff
        self.current_goal_handle = None
        self.goal_start_time = None
        self.goal_timeout = 45
        self.busy_with_goal = False

    def declare_param(self):
        port          = '/dev/ttyUSB0'
        baud_rate         = 115200
        acc           = 50.0
        x_max           = 400.0
        y_max           = 1.0
        z_max           = 1100.0
        default_speed = 2000.0
        speed_x        = 2000.0
        speed_y        = 0
        speed_z        = 2000.0
        steps_x       = 108.0
        steps_y       = 0
        steps_z       = 415.0
        self.declare_parameter('port', port)
        self.declare_parameter('baud_rate', baud_rate)
        self.declare_parameter('acceleration', acc)
        self.declare_parameter('x_max', x_max)
        self.declare_parameter('y_max', y_max)
        self.declare_parameter('z_max', z_max)
        self.declare_parameter('default_speed', default_speed)
        self.declare_parameter('x_max_speed', speed_x)
        self.declare_parameter('y_max_speed', speed_y)
        self.declare_parameter('z_max_speed', speed_z)
        self.declare_parameter('x_steps_mm', steps_x)
        self.declare_parameter('y_steps_mm', steps_y)
        self.declare_parameter('z_steps_mm', steps_z)


    def cnc_init(self):
        port          = self.get_parameter('port').get_parameter_value().string_value
        baud          = self.get_parameter('baud_rate').get_parameter_value().integer_value
        acc           = self.get_parameter('acceleration').get_parameter_value().double_value
        max_x           = self.get_parameter('x_max').get_parameter_value().double_value
        max_y           = self.get_parameter('y_max').get_parameter_value().double_value
        max_z           = self.get_parameter('z_max').get_parameter_value().double_value
        default_speed = self.get_parameter('default_speed').get_parameter_value().double_value
        speed_x        = self.get_parameter('x_max_speed').get_parameter_value().double_value
        speed_y        = self.get_parameter('y_max_speed').get_parameter_value().double_value
        speed_z        = self.get_parameter('z_max_speed').get_parameter_value().double_value
        steps_x       = self.get_parameter('x_steps_mm').get_parameter_value().double_value
        steps_y       = self.get_parameter('y_steps_mm').get_parameter_value().double_value
        steps_z       = self.get_parameter('z_steps_mm').get_parameter_value().double_value
        self.get_logger().info(f"max_x: {max_x}")
        self.get_logger().info(f"max_y: {max_y}")
        self.get_logger().info(f"max_z: {max_z}")


        self.cnc_obj.startup(port,baud,acc,max_x,max_y,max_z,default_speed,speed_x,speed_y,
                speed_z,steps_x,steps_y,steps_z,self.get_logger())

    def cmd_callback(self, msg):
        self.get_logger().info(self.get_name() + ": " + str(msg))
        # if (msg.linear.x < float(self.cnc_obj.origin[0]) or msg.linear.x > float(self.cnc_obj.limits[0]) or msg.linear.z < self.cnc_obj.origin[2] or msg.linear.z > self.cnc_obj.limits[2]):
        #     self.get_logger().info(f"INVALID VALUES: x: {msg.linear.x}, y:{msg.linear.y}, z:{msg.linear.z}")
        #     return

        self.get_logger().info(f"x: {msg.linear.x}, y:{msg.linear.y}, z:{msg.linear.z}")
        print(msg.linear.x, msg.linear.y, msg.linear.z)
        self.cnc_obj.moveTo(msg.linear.x, msg.linear.y, msg.linear.z, blockUntilComplete=True)

    def stop_callback(self, msg):
        #stop steppers
        if  msg.data == 's':
            self.cnc_obj.disableSteppers()
            # fire steppers
        elif msg.data == 'h':
            self.cnc_obj.home()
        elif msg.data == 'f':
            self.cnc_obj.enableSteppers()

    def action_cb(self, goal_handle):
        self.get_logger().info(f"received goal of position: x:\
            {goal_handle.request.goal.x}, y: {goal_handle.request.goal.y},\
            z: {goal_handle.request.goal.z}")
        self.busy_with_goal = True
        self.goal_start_time = self.get_clock().now().to_msg().sec
        self.cnc_obj.moveTo(goal_handle.request.goal.x,
            goal_handle.request.goal.y,
            goal_handle.request.goal.z)

        result = None
        self.current_goal_handle = goal_handle
        while (self.busy_with_goal):
            self.get_logger().info("looping in action callback")
            result = self.action_feedback_loop()
            self.loop()
        self.get_logger().info("exit loop")
        if result is not None:
            self.get_logger().info("returning result")
            return result

    def action_feedback_loop(self):
        if (not self.busy_with_goal):
            return

        success = False
        cnc_pos_twist =  self.cnc_obj.getTwist()
        current_pos = CncPosition()
        current_pos.x = cnc_pos_twist.linear.x
        current_pos.y = cnc_pos_twist.linear.y
        current_pos.z = cnc_pos_twist.linear.z
        self.get_logger().info(f"cnc is at position: x: {current_pos.x},\
            y: {current_pos.y}, z: {current_pos.z}")

        if (current_pos.x == self.current_goal_handle.request.goal.x and
            current_pos.y == self.current_goal_handle.request.goal.y and
            current_pos.z == self.current_goal_handle.request.goal.z):
            success = True
            self.current_goal_handle.succeed()
            result  = CncMoveTo.Result()
            result.position.x = current_pos.x
            result.position.y = current_pos.y
            result.position.z = current_pos.z
            result.success = success
            self.busy_with_goal = False
            # self.current_goal_handle = None
            self.goal_start_time = None
            self.get_logger().info("Reached goal position!")
            self.busy_with_goal = False
            return result

        feedback_msg = CncMoveTo.Feedback()
        feedback_msg.position = current_pos
        feedback_msg.success = success
        self.current_goal_handle.publish_feedback(feedback_msg)

        if (self.get_clock().now().to_msg().sec -
            self.goal_start_time > self.goal_timeout):
            failure_msg = CncMoveTo.Result()
            failure_msg.position = current_pos
            failure_msg.success = success
            self.get_logger().error(f"Action timed out as it took longer\
                than {self.goal_timeout} seconds!")
            self.busy_with_goal = False
            # self.current_goal_handle = None
            self.goal_start_time = None
            return None
        return None

    def loop(self):
        print("p")
        # self.get_logger().info("loop")
        status     = self.cnc_obj.getStatus()
        cnc_pose   = self.cnc_obj.getTwist()
        # self.get_logger().info("after getters")
        ros_status = String()
        ros_status.data = status
        self.pos_pub.publish(cnc_pose)
        self.status_pub.publish(ros_status)
        # self.action_feedback_loop()
        # self.get_logger().info("end loop")

        # Decide if we should call get_parameter() here so we can update
        # the parameters on the go





def main():
    rclpy.init()
    node = CNCInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
