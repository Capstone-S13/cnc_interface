import sys
import rclpy
import argparse


from cnc_msgs.action import CncMoveTo
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability


class CncActionNode(Node):
    def __init__(self, argv=sys.argv):
        parser = argparse.ArgumentParser()
        parser.add_argument('-c' '--command', required=False,
                            type=str, help="commands: 'h' to home, 's' to stop")
        parser.add_argument('-x' '--x', required=False,
                            type=float, help='x coordinate')
        parser.add_argument('-y' '--y', required=False,
                            type=float, help='y coordinate')
        parser.add_argument('-z' '--z', required=False,
                            type=float, help='z coordinate')
        self.args = parser.parse_args(argv[1:])
        qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL
        )

        self.action_client = ActionClient(
            self, CncMoveTo, 'cnc_action'
        )

        self.cnc_cmd_pub = self.create_publisher(String,
            topic='cnc_interface/cmd',
            qos_profile=qos
        )

    def run(self):
        if self.args.command:
            self.send_command()
        elif self.args.x and self.args.y and self.args.z:
            self.send_goal()
        else:
            self.get_logger().error('incomplete arguments!')

    def send_goal(self):
        goal = CncMoveTo.Goal()
        goal.goal.x = self.args.x
        goal.goal.x = self.args.y
        goal.goal.x = self.args.z
        self.action_client.send_goal(goal)
        return

    def send_command(self):
        msg = String()
        msg.data = self.args.command
        self.cnc_cmd_pub.publish(msg)
        return

def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    action = CncActionNode(argv=sys.argv)
    action.run()
    rclpy.shutdown()