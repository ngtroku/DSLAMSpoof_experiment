import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from scipy.spatial import KDTree

# default modules
import random
import json
import time
from pathlib import Path

# load external files
import file_io
import spoofer
import slam
import generate_rosbag
import error_estimate
import post_process

def main():
    # load config
    with open('config_temp.json', 'r') as f:
        config = json.load(f)

    # --- [追加] 結果を蓄積するための辞書 ---
    results_storage = {algo: [] for algo in ['kiss_icp', 'fast_lio', 'direct_lio', 'glim']}

    ref_x, ref_y, ref_z = file_io.load_reference(config['main']['reference_file'])
    loaded_dataframe = file_io.load_reference_df(config['main']['reference_file'])

    success_kiss = 0
    success_flio = 0
    success_dlio = 0
    success_glim = 0

    # record benign
    slam_algorithm = ['glim']
    #slam_algorithm = ['direct_lio']
    bag_path = config['rosbag']['output_bag']

    lidar_topic = config['rosbag']['lidar_topic']
    imu_topic = config['rosbag']['imu_topic']

    benign_save_dir = config['slam']['benign_save_dir']
    save_dir = config['slam']['save_dir']

    # generate ground truth (修正：既に存在する場合はスキップする処理などを入れると効率的です)
    for algorithm in slam_algorithm:
        slam.run_slam(algorithm=algorithm, bag_path=config['rosbag']['input_bag'], topic=lidar_topic, imu_topic=imu_topic, save_dir=benign_save_dir, rosbag_rate=1.0)
        old_file = Path(benign_save_dir) / "temp.txt"
        new_file = old_file.with_name(f"{algorithm}_benign.txt")

        if old_file.exists():
            old_file.rename(new_file)
        else:
            print(f"Error {old_file} not found.")

    n_sims = int(config['main']['n_simulations'])
    for iter in range(n_sims):
        print(f"\n>>> Trial {iter + 1} / {n_sims}")
        
        index = random.randint(0, ref_x.shape[0]-1)
        # Spoofer位置決定
        spoofer_x, spoofer_y, spoofer_z = spoofer.decide_spoofer_placement(ref_x[index], ref_y[index], ref_z[index])

        # generate rosbag
        generate_rosbag.generate_main(spoofer_x, spoofer_y, loaded_dataframe)

        for algorithm in slam_algorithm:
            slam.run_slam(algorithm=algorithm, bag_path=bag_path, topic=lidar_topic, imu_topic=imu_topic, save_dir=save_dir)

            # evaluate
            gt_path = Path(benign_save_dir) / f"{algorithm}_benign.txt"
            APE = error_estimate.evo_eval_result(gt_path, config['evaluation']['estimated'])
            RPE = error_estimate.evo_rpe_eval_results(gt_path, config['evaluation']['estimated'])

            old_file = Path(config['evaluation']['estimated'])
            print(f"old file:{old_file}")
            new_file = old_file.with_name(f"test{iter}.txt")

            if old_file.exists():
                old_file.rename(new_file)
            else:
                print(f"Error {old_file} not found.")

            # --- [追加] データの記録 ---
            results_storage[algorithm].append({
                'iteration': iter,
                'spoofer_x': spoofer_x,
                'spoofer_y': spoofer_y,
                'spoofer_z': spoofer_z,
                'APE': APE,
                'RPE': RPE
            })

            if RPE >= float(config['evaluation']['success_threshold']):
                if algorithm == "kiss_icp": success_kiss += 1
                elif algorithm == "fast_lio": success_flio += 1
                elif algorithm == "direct_lio": success_dlio += 1
                elif algorithm == "glim": success_glim += 1

    # --- [追加] CSV書き出し ---
    print("\n" + "="*30)
    for algorithm in slam_algorithm:
        if results_storage[algorithm]:
            df = pd.DataFrame(results_storage[algorithm])
            output_csv = f"result_{algorithm}.csv"
            df.to_csv(output_csv, index=False)
            print(f"Saved: {output_csv}")

    # --- [追加] クリーンアップ (外部呼び出しのダミー) ---
    post_process.cleanup_results([benign_save_dir, save_dir]) 
    print("Cleanup completed (dummy).")
    print("="*30 + "\n")

    # 成功率の表示
    for algorithm in slam_algorithm:
        counts = {"kiss_icp": success_kiss, "fast_lio": success_flio, "direct_lio": success_dlio, "glim": success_glim}
        success_rate = counts[algorithm] / n_sims
        print(f"{algorithm}: {counts[algorithm]}/{n_sims} success. Rate: {success_rate * 100:.2f}%")

if __name__ == "__main__":
    start = time.time()
    main()
    print(f"processing time:{time.time() - start}sec")