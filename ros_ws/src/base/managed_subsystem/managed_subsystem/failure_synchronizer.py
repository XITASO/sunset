import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from system_interfaces.msg import SetBlackboardGroup 

class BlackboardListener(Node):
    def __init__(self):
        super().__init__('blackboard_listener')

        # Publisher for `/blackboard_group`
        self.publisher_ = self.create_publisher(SetBlackboardGroup, '/blackboard_group', 10)

        # Subscriber to the topic with `SetBlackboardGroup` message
        self.create_subscription(SetBlackboardGroup, '/input_blackboard', self.blackboard_callback, 10)

        self.get_logger().info("Blackboard listener node started... Listening for conditions in the SetBlackboardGroup messages.")

    def blackboard_callback(self, msg):
        segmentation_entropy_value = None
        camera_autofocus_needed_value = None
        freq_condition_met = False

        # Evaluate conditions on the parameters received
        for param in msg.bb_params:
            if param.name == "segmentation_entropy" and float(param.value) > 0.6:
                segmentation_entropy_value = param.value

            if param.name == "camera_autofocus_needed" and param.value.lower() == "true":
                camera_autofocus_needed_value = param.value

            if "freq" in param.name and float(param.value) == 0.0:
                freq_condition_met = True

        # If conditions are met, publish to `/blackboard_group`
        if segmentation_entropy_value and camera_autofocus_needed_value and freq_condition_met:
            blackboard_message = SetBlackboardGroup()
            blackboard_message.bb_params = [
                Parameter('segmentation_entropy', Parameter.Type.STRING, segmentation_entropy_value),
                Parameter('camera_autofocus_needed', Parameter.Type.STRING, camera_autofocus_needed_value),
                Parameter('freq_condition_met', Parameter.Type.BOOL, freq_condition_met)
            ]
            self.publisher_.publish(blackboard_message)
            self.get_logger().info("Published conditions met message to /blackboard_group.")

def main(args=None):
    rclpy.init(args=args)
    blackboard_listener = BlackboardListener()

    try:
        rclpy.spin(blackboard_listener)
    except KeyboardInterrupt:
        blackboard_listener.get_logger().info('Node interrupted by user.')
    finally:
        blackboard_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()