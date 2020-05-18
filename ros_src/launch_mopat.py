import subprocess
import time

if __name__ == "__main__":
    #Start nodes
    #Simulator in a separate terminal
    subprocess.call(['gnome-terminal',
                     '--geometry=80x10+100+100',
                     '--', 'python', 'simulator_node.py'])
    print("LOG: Starting Simulator Node")
    time.sleep(0.5)
    #Others in separate tabs
    subprocess.call(['gnome-terminal',
                     '--geometry=80x10+900+100',
                     '--', 'python', 'occ_map_node.py'])
    print("LOG: Starting Occupancy Map Generator Node")
    time.sleep(0.5)
    subprocess.call(['gnome-terminal',
                     '--geometry=80x10+100+400',
                     '--', 'python', 'config_space_node.py'])
    print("LOG: Starting Configuration Space Generator Node")
    time.sleep(0.5)
    subprocess.call(['gnome-terminal',
                     '--geometry=80x10+900+400',
                     '--', 'python', 'multi_robot_coordinator_node.py'])
    print("LOG: Starting Multi-Robot Coordinator Node")
    time.sleep(0.5)
    subprocess.call(['gnome-terminal',
                     '--geometry=80x10+100+700',
                     '--', 'python', 'motion_planner_node.py'])
    print("LOG: Starting Motion Planner Node")
