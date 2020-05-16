import subprocess
import time

if __name__ == "__main__":
    #Start nodes
    subprocess.call(['gnome-terminal', '--', 'python', 'simulator_node.py'])
    print("LOG: Starting Simulator Node")
    time.sleep(0.5)
    subprocess.call(['gnome-terminal', '--', 'python', 'occ_map_node.py'])
    print("LOG: Starting Occupancy Map Generator Node")
    time.sleep(0.5)
    subprocess.call(['gnome-terminal', '--', 'python', 'config_space_node.py'])
    print("LOG: Starting Configuration Space Generator Node")
    time.sleep(0.5)
    subprocess.call(['gnome-terminal', '--', 'python', 'multi_robot_coordinator_node.py'])
    print("LOG: Starting Multi-Robot Coordinator Node")
