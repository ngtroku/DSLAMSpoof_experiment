import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from scipy.spatial import KDTree

# default modules
import random
import json

# load external files
import file_io
import spoofer
import slam
import generate_rosbag
import error_estimate

def main():
    # load config
    with open('config_temp.json', 'r') as f:
        config = json.load(f)

    ref_x, ref_y, ref_z = file_io.load_reference(config['main']['reference_file'])
    loaded_dataframe = file_io.load_reference_df(config['main']['reference_file'])

    success = 0

    for iter in range(int(config['main']['n_simulations'])):
        index = random.randint(0, ref_x.shape[0]-1)
        spoofer_x, spoofer_y, spoofer_z = spoofer.decide_spoofer_placement(ref_x[index], ref_y[index], ref_z[index]) # spoofer placement (world coordinate)

        # generate rosbag
        generate_rosbag.generate_main(spoofer_x, spoofer_y, loaded_dataframe)

        # run slam
        slam_algorithm = config['slam']['algorithm']
        bag_path = config['rosbag']['output_bag']
        lidar_topic = config['rosbag']['lidar_topic']
        save_dir = config['slam']['save_dir']
        slam.run_slam(algorithm=slam_algorithm, bag_path=bag_path, topic=lidar_topic, save_dir=save_dir)

        # evaluate
        APE = error_estimate.evo_eval_result(config['evaluation']['ground_truth'], config['evaluation']['estimated'])

        if APE >= float(config['evaluation']['threshold']):
            success += 1
        else:
            pass
    
    success_rate = success / int(config['main']['n_simulations'])
    print(f"{success}/{int(config['main']['n_simulations'])} success. Success rate is {success_rate * 100}%")

if __name__ == "__main__":
    main()