# my_ros2_package/blackboard_subscriber.py

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType
from system_interfaces.msg import SetBlackboardGroup
from rclpy.qos import qos_profile_services_default

class BlackboardGroupSubscriber(Node):

    def __init__(self):
        super().__init__('blackboard_group_subscriber')
        self.subscription = self.create_subscription(
            SetBlackboardGroup,
            '/blackboard_group',
            self.listener_callback,
            qos_profile_services_default)
        
        # Create/Open log file
        self.log_file_path = 'blackboard_log.txt'
        self.log_file = open(self.log_file_path, 'w')

        self.get_logger().info('Blackboard Group Subscriber Node has been started.')

    def listener_callback(self, msg: SetBlackboardGroup):
        log_entries = []
        for param in msg.bb_params:
            logging = False
            # Determine the type and log the appropriate value
            if param.value.type == ParameterType.PARAMETER_BOOL:
                value = param.value.bool_value
            elif param.value.type == ParameterType.PARAMETER_INTEGER:
                value = param.value.integer_value
            elif param.value.type == ParameterType.PARAMETER_DOUBLE:
                value = param.value.double_value
            elif param.value.type == ParameterType.PARAMETER_STRING:
                value = param.value.string_value
            else:
                value = 'Unknown Type'

            entry = f"{self.get_clock().now()} Name: {param.name}, Value: {value}\n"
            if param.name == "segmentation_entropy":
                logging = True

            if param.name == "camera_autofocus_needed":
                logging = True

            if logging == True:
                self.get_logger().info(entry)
                log_entries.append(entry)
        
        # Write to log file
        self.log_file.writelines(log_entries)

    def __del__(self):
        self.log_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = BlackboardGroupSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Blackboard Group Subscriber Node.')
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()