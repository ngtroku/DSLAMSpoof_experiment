import numpy as np

import random
import json

def decide_spoofer_placement(traj_x, traj_y, traj_z):

    # load config
    with open('config_temp.json', 'r') as f:
        config = json.load(f)

    r = float(config['spoofer']['dist_from_traj'])
    theta = random.uniform(-180, 180) # degree

    spoofer_x = traj_x + (r * np.cos(np.deg2rad(theta))) # spooferのx座標
    spoofer_y = traj_y + (r * np.sin(np.deg2rad(theta))) # spooferのy座標
    spoofer_z = traj_z # spooferのz座標

    return spoofer_x, spoofer_y, spoofer_z