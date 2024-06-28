#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap

class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')
        self.client = self.create_client(LoadMap, '/map_server/load_map')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.load_map()

    def load_map(self):
        request = LoadMap.Request()
        request.map_url = '/home/nuc/h3.yaml'
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        # Result code defintions
        # uint8 RESULT_SUCCESS=0
        # uint8 RESULT_MAP_DOES_NOT_EXIST=1
        # uint8 RESULT_INVALID_MAP_DATA=2
        # uint8 RESULT_INVALID_MAP_METADATA=3
        # uint8 RESULT_UNDEFINED_FAILURE=255
        
        if self.future.result().result == 0:
            print('OK')
        else:
            if self.future.result().result == 1:
                print('problem: map does not exist.')
            elif self.future.result().result == 2:
                print('problem: invalid map data.')
            elif self.future.result().result == 3:
                print('problem: invalid map metadata.')
            elif self.future.result().result == 255:
                print('problem: undefined failure.')

def main(args=None):
    rclpy.init()
    map_loader = MapLoader()
    map_loader.destroy_node()
    rclpy.shutdown()

    # rclpy.init(args=args)
    # map_loader = MapLoader()
    # rclpy.spin(map_loader)
    # map_loader.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()