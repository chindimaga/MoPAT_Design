import subprocess
import time

subprocess.call(['gnome-terminal', '--', 'python', 'simulator_node.py'])
time.sleep(0.5)
subprocess.call(['gnome-terminal', '--', 'python', 'occ_map_node.py'])
time.sleep(0.5)
subprocess.call(['gnome-terminal', '--', 'python', 'config_space_node.py'])
