import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import sys

def read_ros2_bag(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    while reader.has_next():
        topic, data, t = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        print(f"Time: {t}")
        print(f"Topic: {topic}")
        print(f"Message Type: {type_map[topic]}")
        print(f"Message Content: {msg}")
        print("---")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print("Usage: python script_name.py <path_to_ros2_bagfile>")
        sys.exit(1)

    bag_path = sys.argv[1]
    print(f"Reading ROS2 bag file: {bag_path}")
    read_ros2_bag(bag_path)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
