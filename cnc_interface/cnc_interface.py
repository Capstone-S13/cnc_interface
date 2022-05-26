import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from rclpy.parameter import Parameter

from cnc_interface.cnc_class import cnc


class CNCInterfaceNode(Node):
    def __init__(self):
        super().__init__('cnc_interface_node')

        # Create and initialise cnc object
        self.cnc_obj = cnc()
        self.declare_param()
        self.cnc_init()

        # Publishers
        self.pos_pub = self.create_publisher(Twist,'/cnc_interface/position', 10)
        self.status_pub = self.create_publisher(String, '/cnc_interface/status', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, 'cnc_interface/pos', self.cmd_callback, 10)
        self.stop_cmd_sub = self.create_subscription(String, 'cnc_interface/cmd', self.stop_callback, 10)

        # timer callback
        timer_period = 0.1 # 10hz
        self.timer = self.create_timer(timer_period, self.loop)

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
        max_z           = self.get_parameter('x_max').get_parameter_value().double_value
        default_speed = self.get_parameter('default_speed').get_parameter_value().double_value
        speed_x        = self.get_parameter('x_max_speed').get_parameter_value().double_value
        speed_y        = self.get_parameter('y_max_speed').get_parameter_value().double_value
        speed_z        = self.get_parameter('z_max_speed').get_parameter_value().double_value
        steps_x       = self.get_parameter('x_steps_mm').get_parameter_value().double_value
        steps_y       = self.get_parameter('y_steps_mm').get_parameter_value().double_value
        steps_z       = self.get_parameter('z_steps_mm').get_parameter_value().double_value


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
