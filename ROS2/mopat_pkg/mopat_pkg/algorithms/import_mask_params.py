import cv2
import numpy as np

def import_mask_params(filepath) :

    # importing mask parameters
    file = open(filepath, "r+")
    param_list = file.readlines()

    lh_b = int(param_list[0])
    ls_b = int(param_list[1])
    lv_b = int(param_list[2])

    uh_b = int(param_list[3])
    us_b = int(param_list[4])
    uv_b = int(param_list[5])

    lh_g = int(param_list[6])
    ls_g = int(param_list[7])
    lv_g = int(param_list[8])

    uh_g = int(param_list[9])
    us_g = int(param_list[10])
    uv_g = int(param_list[11])

    lh_r = int(param_list[12])
    ls_r = int(param_list[13])
    lv_r = int(param_list[14])

    uh_r = int(param_list[15])
    us_r = int(param_list[16])
    uv_r = int(param_list[17])

    # defining the lower bound and upper bound vectors

    l_b = np.array([lh_b, ls_b, lv_b])
    u_b = np.array([uh_b, us_b, uv_b])

    l_g = np.array([lh_g, ls_g, lv_g])
    u_g = np.array([uh_g, us_g, uv_g])

    l_r = np.array([lh_r, ls_r, lv_r])
    u_r = np.array([uh_r, us_r, uv_r])

    mask_params= [ l_b, u_b, l_g, u_g, l_r, u_r ]
    return mask_params

def main():
    pass
