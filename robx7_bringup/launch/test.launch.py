from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    Id =LaunchDescription()

    custom_subscriber_node = Node(
        package="cpp_example",
        executable="custom_subscriber_cpp")

    custom_publisher_node = Node(
        package="cpp_example",
        executable="custom_publisher_cpp")

    number_publisher_node = Node(
        package="cpp_example",
        executable="number_publisher",
        parameters=[{"number": 10, "timer_period": 1.0}])

    Id.add_action(custom_subscriber_node)
    Id.add_action(custom_publisher_node)
    Id.add_action(number_publisher_node)
    return Id