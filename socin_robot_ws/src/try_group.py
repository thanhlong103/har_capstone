import rclpy
from rclpy.node import Node
from nav2_costmap_2d import Costmap2D, Costmap2DROS
from nav2_costmap_2d.plugins import Costmap2DPlugin
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter

class MyCostmapPlugin(Costmap2DPlugin):
    def __init__(self):
        super().__init__()

    def initialize(self):
        self.get_logger().info("Initializing MyCostmapPlugin")
        
        # Here you can initialize any parameters or services for your plugin
        self.declare_parameter("my_custom_param", 1.0)

    def update_costmap(self, costmap: Costmap2D):
        # Here you can write your logic to update the costmap
        self.get_logger().info("Updating costmap in MyCostmapPlugin")
        
        # Example: Just updating the costmap with a simple pattern
        width = costmap.get_size_in_cells_x()
        height = costmap.get_size_in_cells_y()
        
        for i in range(width):
            for j in range(height):
                # Example logic for updating costmap
                # Here, we mark some cells with high cost (e.g., an obstacle)
                if i % 2 == 0 and j % 2 == 0:
                    costmap.set_cost(i, j, Costmap2D.UNKNOWN)

        return costmap

def main(args=None):
    rclpy.init(args=args)

    my_plugin_node = MyCostmapPlugin()

    # Create the node that will use this plugin
    rclpy.spin(my_plugin_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
