#Guining Pertin - Started - 12-05-20
#Multi Robot Simulator Mk2
import sys
import mopat_lib

if __name__ == "__main__":
    #Change MoPAT default parameters
    mopat_lib.screen_size = (500,500)
    mopat_lib.robot_starts = {0: (200,200), 1: (50, 100)}
    mopat_lib.robot_goals = {0: (450,300), 1: (450, 100)}
    sys.exit(mopat_lib.simulation())
