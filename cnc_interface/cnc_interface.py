import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from cnc_class import cnc


class CNCInterfaceNode(Node):
	def __init__(self):
		super().__init__('CNC_interface_node')

		# Create and initialise cnc object
		self.cnc_object = cnc()
		self.cnc_init()

		# Publishers
		self.pos_pub = self.create_publisher(Twist,'/cnc_interface/position', 10)
		self.status_pub = self.create_publisher(String, '/cnc_interface/status', 10)

		# Subscribers
		self.cmd_sub = self.create_subscription(Twist, 'cnc_interface/cmd', self.cmd_callback)
		self.stop_cmd_sub = self.create_subscription(String, 'cnc_interface/stop', self.stop_callback)

		# timer callback
		timer_period = 0.1 # 10hz
		self.timer = self.create_timer(timer_period, self.loop)

	def cnc_init(self):
		# may need to convert this into the correct types
		port          = self.get_parameter('cnc_interface/port')
		baud          = self.get_parameter('cnc_interface/baudrate')
		acc           = self.get_parameter('cnc_interface/acceleration')
		max_x 		  = self.get_parameter('cnc_interface/x_max')
		max_y 		  = self.get_parameter('cnc_interface/y_max')
		max_z 		  = self.get_parameter('cnc_interface/x_max')
		default_speed = self.get_parameter('cnc_interface/default_speed')
		speed_x  	  = self.get_parameter('cnc_interface/x_max_speed')
		speed_y  	  = self.get_parameter('cnc_interface/y_max_speed')
		speed_z  	  = self.get_parameter('cnc_interface/z_max_speed')
		steps_x 	  = self.get_parameter('cnc_interface/x_steps_mm')
		steps_y 	  = self.get_parameter('cnc_interface/y_steps_mm')
		steps_z 	  = self.get_parameter('cnc_interface/z_steps_mm')
		self.cnc_obj.startup(port,baud,acc,max_x,max_y,max_z,default_speed,speed_x,speed_y,
				speed_z,steps_x,steps_y,steps_z)

	def cmd_callback(self, msg):
		self.get_logger().info(self.get_name() + ": " + str(msg))
		print(msg.linear.x, msg.linear.y, msg.linear.z)
		self.cnc_obj.moveTo(msg.linear.x, msg.linear.y, msg.linear.z, blockUntilComplete=True)

	def stop_callback(self, msg):
		#stop steppers
		if   msg.data == 's':
			cnc_obj.disableSteppers()
			# fire steppers
		elif msg.data == 'f':
			cnc_obj.enableSteppers()

	def loop(self):
		status     = cnc_obj.getStatus()
		cnc_pose   = cnc_obj.getTwist()
		ros_status = String(status)
		self.pos_pub.publish(cnc_pose)
		self.status_pub.publish(ros_status)

		# TODO: Decide if we should call get_parameter() here so we can update
		# the parameters on the go


def main():
	rclpy.init()
	node = CNCInterfaceNode(Node)
	rclpy.spin(node)

if __name__ == '__main__':
    main()
