from typing import List
from config.config import TASK_CONFIG, ROBOT_PORTS
import os
import cv2
import h5py
import argparse
from tqdm import tqdm
from time import sleep, time
from training.utils import pwm2pos, pwm2vel
import numpy as np

from robot import Robot
from robot_ft import Robot as RobotFT

# parse the task name via command line
parser = argparse.ArgumentParser()
parser.add_argument('--task', type=str, default='task1')
parser.add_argument('--num_episodes', type=int, default=1)
args = parser.parse_args()
task = args.task
num_episodes = args.num_episodes

cfg = TASK_CONFIG

def get_bias():
    length = len(leader.read_position())
    leader_pos = leader.read_position()
    print(f"leader_pos: {leader_pos}")
    follower_pos = follower.read_position()
    print(f"follower_pos: {follower_pos}")
    bias = [leader_pos[i] - follower_pos[i] for i in range(length)]
    print(bias)
    return bias


def capture_image(cam):
    # Capture a single frame
    _, frame = cam.read()
    # Generate a unique filename with the current date and time
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # Define your crop coordinates (top left corner and bottom right corner)
    # x1, y1 = 400, 0  # Example starting coordinates (top left of the crop rectangle)
    # x2, y2 = 1600, -1  # Example ending coordinates (bottom right of the crop rectangle)
    # # Crop the image
    # image = image[y1:y2, x1:x2]
    # Resize the image
    image = cv2.resize(image, (cfg['cam_width'], cfg['cam_height']), interpolation=cv2.INTER_AREA)

    return image


def follow_leader_pos(leader_bot: Robot, follow_bot: Robot, bias: np.array):
    pos = leader_bot.read_position()
    print("[debug]-action-raw", pos)
    action = pos - bias
    follow_bot.set_goal_pos(action)
    return action
    


if __name__ == "__main__":
    
    # init follower
    follower = RobotFT(device_name=ROBOT_PORTS['follower'], servo_ids=[1,2,3,4,5,6])
    # init leader
    leader = Robot(device_name=ROBOT_PORTS['leader'], servo_ids=[1,2,3,4,5,6])
    # get bias
    # get_bias();exit()
    
    bias = np.array([1857, 1040, 256, 1978, -152, 22])
    # init camera
    cam = cv2.VideoCapture(cfg['camera_port'])
    # Check if the camera opened successfully
    if not cam.isOpened():
        raise IOError("Cannot open camera")
    leader.set_trigger_torque(value=-200)
    
    for i in range(num_episodes):
        # bring the follower to the leader and start camera
        for i in range(200):
            follow_leader_pos(leader, follower, bias)
            _ = capture_image(cam)
        os.system('say "go"')
        # init buffers
        obs_replay = []
        action_replay = []
        for i in tqdm(range(cfg['episode_len'])):
            # observation
            qpos = follower.read_position()
            print("[debug]-qpos", qpos)
            qvel = follower.read_velocity()
            image = capture_image(cam)
            obs = {
                'qpos': pwm2pos(qpos),
                'qvel': pwm2vel(qvel),
                'images': {cn : image for cn in cfg['camera_names']}
            }
            # action (leader's position), apply action
            action = follow_leader_pos(leader, follower, bias)
            action = pwm2pos(action)
            # store data
            obs_replay.append(obs)
            action_replay.append(action)

        os.system('say "stop"')

        # disable torque
        #leader._disable_torque()
        #follower._disable_torque()

        # create a dictionary to store the data
        data_dict = {
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/action': [],
        }
        # there may be more than one camera
        for cam_name in cfg['camera_names']:
                data_dict[f'/observations/images/{cam_name}'] = []

        # store the observations and actions
        for o, a in zip(obs_replay, action_replay):
            data_dict['/observations/qpos'].append(o['qpos'])
            data_dict['/observations/qvel'].append(o['qvel'])
            data_dict['/action'].append(a)
            # store the images
            for cam_name in cfg['camera_names']:
                data_dict[f'/observations/images/{cam_name}'].append(o['images'][cam_name])

        t0 = time()
        max_timesteps = len(data_dict['/observations/qpos'])
        # create data dir if it doesn't exist
        data_dir = os.path.join(cfg['dataset_dir'], task)
        if not os.path.exists(data_dir): os.makedirs(data_dir)
        # count number of files in the directory
        idx = len([name for name in os.listdir(data_dir) if os.path.isfile(os.path.join(data_dir, name))])
        dataset_path = os.path.join(data_dir, f'episode_{idx}')
        # save the data
        with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            for cam_name in cfg['camera_names']:
                _ = image.create_dataset(cam_name, (max_timesteps, cfg['cam_height'], cfg['cam_width'], 3), dtype='uint8',
                                        chunks=(1, cfg['cam_height'], cfg['cam_width'], 3), )
            qpos = obs.create_dataset('qpos', (max_timesteps, cfg['state_dim']))
            qvel = obs.create_dataset('qvel', (max_timesteps, cfg['state_dim']))
            # image = obs.create_dataset("image", (max_timesteps, 240, 320, 3), dtype='uint8', chunks=(1, 240, 320, 3))
            action = root.create_dataset('action', (max_timesteps, cfg['action_dim']))
            
            for name, array in data_dict.items():
                root[name][...] = array
    
    leader._disable_torque()
    follower._disable_torque()
