import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)

    # We don't need a full class for this simple node
    node = rclpy.create_node('user_input_node')
    publisher = node.create_publisher(String, 'user_query', 10)

    node.get_logger().info('--- User Input Node has started ---')
    node.get_logger().info(">>> Type your query and press Enter. Type 'exit' to quit.")

    try:
        while rclpy.ok():
            # Get input from the user in the terminal
            user_text = input("You: ")

            if user_text.lower() == 'exit':
                break

            # Prepare the ROS message
            msg = String()
            msg.data = user_text

            # Publish the message and log it to the console
            publisher.publish(msg)
            node.get_logger().info(f"Publishing to /user_query: '{user_text}'")

    except KeyboardInterrupt:
        # This handles Ctrl+C to exit gracefully
        pass
    finally:
        # Clean up when the loop is exited
        node.get_logger().info('Shutting down user input node.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()