import subprocess

def run_slam(algorithm='kiss_icp', bag_path='', topic='/velodyne_points', save_dir='', rosbag_rate='2.0', visualize=True):
    """
    Pythonの変数をroslaunchの引数として渡して実行する
    """
    if algorithm == 'kiss_icp':
        
        # 1. 使用するベースのlaunchファイルを選択
        launch_file = "slam_test_kiss.launch"
    
        # 2. 基本コマンドの構成
        cmd = [
            "roslaunch", 
            "slamspoof", 
            launch_file,
            f"bagfile:={bag_path}",
            f"topic:={topic}",
            f"name_traj_dir:={save_dir}",
            f"rosbag_rate:={rosbag_rate}",
            f"visualize:={'true' if visualize else 'false'}"
        ]

        print(f"\n--- SLAM開始 ({algorithm}) ---")
        print(f"実行コマンド: {' '.join(cmd)}")
        
        try:
            # check=Trueでエラー時に例外を投げる
            subprocess.run(cmd, check=True)
            print("--- SLAM正常終了 ---")
        except subprocess.CalledProcessError as e:
            print(f"SLAM実行中にエラーが発生しました (Exit code: {e.returncode})")
        except Exception as e:
            print(f"予期せぬエラー: {e}")

    elif algorithm == 'fast_lio':

        launch_file = "slam_test_flio.launch"

        cmd = [
            "roslaunch", 
            "slamspoof", 
            launch_file,
            f"bagfile:={bag_path}",
            f"topic:={topic}",
            f"name_traj_dir:={save_dir}",
            f"rosbag_rate:={rosbag_rate}",
            f"visualize:={'true' if visualize else 'false'}"
        ]

        try:
            # check=Trueでエラー時に例外を投げる
            subprocess.run(cmd, check=True)
            print("--- SLAM正常終了 ---")
        except subprocess.CalledProcessError as e:
            print(f"SLAM実行中にエラーが発生しました (Exit code: {e.returncode})")
        except Exception as e:
            print(f"予期せぬエラー: {e}")