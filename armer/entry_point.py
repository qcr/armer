import os
import rclpy
import ament_index_python
from rclpy.node import Node
from armer.armer import Armer

__path__ = ament_index_python.packages.get_package_share_directory('armer')

class ArmerNode(Node):
  def __init__(self):
    super().__init__('armer')
    self.declare_parameter('config')
    
    # NOTE: defaults to the panda simulation 
    self.armer = Armer.load(
      self, 
      self.get_parameter_or('config').get_parameter_value().string_value or os.path.join(__path__, 'cfg/panda_sim.yaml')
    )

    self.last_time = self.get_clock().now()
    self.timer = self.create_timer(1 / self.armer.frequency, self.timer_callback)

  def timer_callback(self):
    current_time = self.get_clock().now()
    self.armer.step((current_time - self.last_time).nanoseconds / 1e-9, current_time)
    self.last_time = current_time

if __name__ == '__main__':
    rclpy.init(args=None)

    armer = ArmerNode()

    rclpy.spin(armer)

    armer.destroy_node()
    rclpy.shutdown()