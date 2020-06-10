#!/usr/bin/env python3
#MoPAT Design Lab - Guining Pertin
#Launcher script - 16-05-20

#Import libraries
import subprocess
import time

if __name__ == "__main__":
    #Start nodes
    subprocess.call(['gnome-terminal',
                     '--geometry=50x2+00+600',
                     '--title=Simulator Node',
                     '--', 'python', 'simulator_node.py'])
    print("LOG: Starting Simulator Node")
    time.sleep(0.5)

    # subprocess.call(['gnome-terminal',
    #                  '--geometry=50x2+00+600',
    #                  '--title=Simulator Node',
    #                  '--', 'python', 'formation_sim_node.py'])
    # print("LOG: Starting Formation Simulator Node")
    # time.sleep(0.5)

    subprocess.call(['gnome-terminal',
                     '--geometry=50x2+1200+50',
                     '--title=Occupancy Map Node',
                     '--', 'python', 'occ_map_node.py'])
    print("LOG: Starting Occupancy Map Generator Node")
    time.sleep(0.5)

    subprocess.call(['gnome-terminal',
                     '--title=Configuration Space Node',
                     '--geometry=50x2+1200+150',
                     '--', 'python', 'config_space_node.py'])
    print("LOG: Starting Configuration Space Generator Node")
    time.sleep(0.5)

    subprocess.call(['gnome-terminal',
                     '--geometry=50x2+1200+250',
                     '--title=Multi Robot Coordinator Node',
                     '--', 'python', 'multi_robot_coordinator_node.py'])
    print("LOG: Starting Multi-Robot Coordinator Node")
    time.sleep(0.5)

    subprocess.call(['gnome-terminal',
                     '--geometry=50x2+1200+350',
                     '--title=Motion Planner Node',
                     '--', 'python', 'motion_planner_node.py'])
    print("LOG: Starting Motion Planner Node")
    time.sleep(0.5)

    subprocess.call(['gnome-terminal',
                     '--geometry=50x2+1200+450',
                     '--title=MoPAT Plot Node',
                     '--', 'python', 'plot_node.py'])
    print("LOG: Starting MoPAT Plotter Node")
    time.sleep(0.5)
