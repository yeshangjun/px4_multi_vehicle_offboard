#!/usr/bin/env python3

__author__ = 'Arthur Astier'

# Import the subprocess and time modules
import rclpy
from rclpy.node import Node
import subprocess
import time
import os
import yaml
from ament_index_python.packages import get_package_share_directory


class SimulationScript(Node):
    def __init__(self):
        super().__init__('simulation_node')
        
        commands = [# Run the Micro XRCE-DDS Agent
            "MicroXRCEAgent udp4 -p 8888"]
        self.read_swarm_config()
        i = 0
        for key, item in self.swarm_config.items():
            model = item['model']
            initial_pose_x = item["initial_pose"]['x']
            initial_pose_y = item["initial_pose"]['y']
            self.get_logger().info(f'model={model}, x={initial_pose_x}, y={initial_pose_y}')
            cmd = f'PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_{model} PX4_GZ_MODEL_POSE="{initial_pose_x},{initial_pose_y}" ~/PX4_115/build/px4_sitl_default/bin/px4 -i {i+1}'
            i = i+1
            commands.append(cmd)
            self.get_logger().info(f'{cmd}')
        # # List of commands to run
        # commands = [
        #     # Run the Micro XRCE-DDS Agent
        #     "MicroXRCEAgent udp4 -p 8888",

        #     # Run the PX4 SITL simulation
        #     # "cd ~/PX4-Autopilot && /bin/bash ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh" + query
        #     # Run the PX4 SITL simulation
        #     "PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ~/PX4_115/build/px4_sitl_default/bin/px4 -i 1",
        #     "PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ~/PX4_115/build/px4_sitl_default/bin/px4 -i 2",
        #     "PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ~/PX4_115/build/px4_sitl_default/bin/px4 -i 3"

        # ]

        # Loop through each command in the list
        for command in commands:
            # Each command is run in a new tab of the gnome-terminal
            subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])

            # Pause between each command
            time.sleep(5)
    # read plan point from trajectory.yaml
    def read_swarm_config(self):
        
        # 获取YAML文件路径
        package_name = 'px4_multi_plan'
        yaml_file_name = 'swarm_config.yaml'
        yaml_file_path = os.path.join(
            get_package_share_directory(package_name),
            # 'resource',
            yaml_file_name
        )

        try:
            # 打开并读取YAML文件
            with open(yaml_file_path, 'r') as file:
                config = yaml.safe_load(file)

            # 获取参数
            self.swarm_config = config['swarm']
            self.nb_drone = len(self.swarm_config)
            
            
        except Exception as e:
            self.get_logger().error(f"Error reading YAML file: {e}")
        



def main(args=None):
    rclpy.init(args=args)
    node = SimulationScript()
    try:
        rclpy.spin(node)
    finally:
        # subprocess.run("pkill gnome-terminal", shell=True)
        subprocess.run("exit", shell=True)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
