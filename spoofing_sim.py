#!/usr/bin/env python3

import numpy as np
import rospy
import json

def cartesian2polar(x, y):
    r = (x ** 2 + y ** 2) ** 0.5
    theta = np.degrees(np.arctan2(y, x)) 
    return r, theta

def polar2cartesian(r, theta):
    x = r * np.cos(np.radians(theta))
    y = r * np.sin(np.radians(theta))
    return x, y

def removal_simulation(raw_points, mask_index):
    removed_points = np.delete(raw_points, list(mask_index[0]))
    return removed_points

def set_distance(timestamp):

    with open('config_temp.json', 'r') as f:
        config = json.load(f)

    minimum_distance = config['spoofing_simulation']['minimum_distance']
    maximum_distance = config['spoofing_simulation']['maximum_distance']
    time_cycle = config['spoofing_simulation']['time_cycle']

    f_t = (((maximum_distance - minimum_distance) / time_cycle) * (timestamp % time_cycle)) + minimum_distance
    return f_t

def noise_simulation(raw_points, largest_score_angle, spoofing_range):
    rng = np.random.default_rng() 
    with open('config.json', 'r') as f:
        config = json.load(f)

    horizontal_resolution = config['simulator']['horizontal_resolution']
    vertical_lines = config['simulator']['vertical_lines']
    spoofing_rate = config['simulator']['spoofing_rate']

    temp_min = largest_score_angle - (spoofing_range / 2) 
    temp_max = largest_score_angle + (spoofing_range / 2) 

    min, max = temp_min, temp_max

    r, theta = cartesian2polar(raw_points[:, 0], raw_points[:, 1]) 

    # theta : -180 ~ 180 (deg)
    z = raw_points[:, 2]

    mask = ((min <= theta) & (theta <= max)) 

    r_deleted = r[~mask]
    theta_deleted = theta[~mask]
    z_deleted = z[~mask]

    num_spoofed_points = int((spoofing_range / horizontal_resolution) * vertical_lines * spoofing_rate)

    r_noise = rng.uniform(0.0, 200.0, num_spoofed_points)
    theta_noise = rng.uniform(temp_min, temp_max, num_spoofed_points)
    z_noise = r_noise * np.sin(np.degrees(rng.uniform(-15.0, 15.0, num_spoofed_points)))

    # raw points
    x_remaining, y_remaining = polar2cartesian(r_deleted, theta_deleted)
    points_remaining = np.vstack((x_remaining, y_remaining, z_deleted)).T

    # spoofed points
    x_spoofed, y_spoofed = polar2cartesian(r_noise, theta_noise)
    points_spoofed = np.vstack((x_spoofed, y_spoofed, z_noise)).T
   
    return points_remaining, points_spoofed

def defenced(raw_points, largest_score_angle, spoofing_range):

    temp_min = largest_score_angle - (spoofing_range / 2) 
    temp_max = largest_score_angle + (spoofing_range / 2) 

    min, max = temp_min, temp_max

    r, theta = cartesian2polar(raw_points[:, 0], raw_points[:, 1]) 

    # theta : -180 ~ 180 (deg)
    z = raw_points[:, 2]

    mask = ((min <= theta) & (theta <= max)) 

    r_deleted = r[~mask]
    theta_deleted = theta[~mask]
    z_deleted = z[~mask]

    x_deleted, y_deleted = polar2cartesian(r_deleted, theta_deleted)

    return x_deleted, y_deleted, z_deleted

def injection_simulation(raw_points, largest_score_angle, spoofing_range, injection_dist):
    rng = np.random.default_rng()

    with open('config_temp.json', 'r') as f:
        config = json.load(f)

    temp_min = largest_score_angle - (spoofing_range / 2) 
    temp_max = largest_score_angle + (spoofing_range / 2) 

    if temp_min < 0: 
        min = 360 - temp_min
        max = temp_max

    elif temp_max > 360: 
        min = temp_min
        max = temp_max - 360

    else: 
        min = temp_min
        max = temp_max

    r, theta = cartesian2polar(raw_points[:, 0], raw_points[:, 1])
    z = raw_points[:, 2]
    mask = ((min <= theta) & (theta <= max)) 

    r_deleted = r[~mask]
    theta_deleted = theta[~mask]
    z_deleted = z[~mask]

    horizontal_resolution = 0.2
    vertical_lines = 32
    n_injection = int((spoofing_range / horizontal_resolution) * vertical_lines)  
    #vertical_angle_canditate = [-15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15] 
    vertical_angle_canditate = [-1.333, -1.0, -0.667, -0.333, 0, 0.333, 0.667, 1.0, 1.333]

    if config['spoofing_simulation']['injection_mode'] == 'wall':
        r_wall = np.full(n_injection, injection_dist)
        theta_wall = rng.uniform(temp_min, temp_max, n_injection) # Unit : degree

    elif config['spoofing_simulation']['injection_mode'] == 'corner':
        r_wall = np.full(n_injection, injection_dist)
        theta_wall = rng.uniform(temp_min, temp_max, n_injection) # Unit : degree
        rotation = float(config['spoofing_simulation']['corner_rotation'])

        theta_wall_rad = np.radians(theta_wall)
        center_angle = (np.min(theta_wall_rad) + np.max(theta_wall_rad)) / 2 + np.radians(rotation)

        theta_wall_rad = np.radians(theta_wall)
        r_wall = r_wall/(np.abs(np.cos(theta_wall_rad - center_angle)) + np.abs(np.sin(theta_wall_rad - center_angle)))

    else:
        r_wall = np.full(n_injection, injection_dist)
        theta_wall = rng.uniform(temp_min, temp_max, n_injection) # Unit : degree

    #theta_wall_rad = np.radians(theta_wall)

    #r_wall = r_wall * (1+0.2*np.sin(8*theta_wall_rad))

    vertical_angle_wall = np.random.choice(vertical_angle_canditate, size=n_injection, replace=True)
    z_wall = r_wall * np.sin(np.degrees(vertical_angle_wall))
    
    # raw points
    x_remaining, y_remaining = polar2cartesian(r_deleted, theta_deleted)
    points_remaining = np.vstack((x_remaining, y_remaining, z_deleted)).T

    # spoofed points
    x_spoofed, y_spoofed = polar2cartesian(r_wall, theta_wall)
    points_spoofed = np.vstack((x_spoofed, y_spoofed, z_wall)).T
   
    return points_remaining, points_spoofed

def decide_mask(horizontal_angle, largest_score_angle, spoofing_range):
    temp_min = largest_score_angle - (spoofing_range / 2) 
    temp_max = largest_score_angle + (spoofing_range / 2) 
    if temp_min < 0: 
        min = 360 - temp_min
        spoofing_condition = ((min <= horizontal_angle) | (horizontal_angle <= temp_max))

    elif temp_max > 360: 
        max = temp_max - 360
        spoofing_condition = ((temp_min <= horizontal_angle) | (horizontal_angle <= max))

    else: 
        spoofing_condition = ((temp_min <= horizontal_angle) & (horizontal_angle <= temp_max))
    
    return spoofing_condition

def spoof_main(pointcloud, largest_score_angle, spoofing_range): 
    angle = np.degrees(np.arctan2(pointcloud[:, 1], pointcloud[:, 0])) + 180
    mask_condition = decide_mask(angle, largest_score_angle, spoofing_range)
    mask_index = np.where(mask_condition)

    x_spoofed , y_spoofed, z_spoofed = pointcloud[:, 0], pointcloud[:, 1], pointcloud[:, 2]

    points_remaining, points_spoofed = noise_simulation(pointcloud, largest_score_angle, spoofing_range)
   
    x_remaining, y_remaining, z_remaining = points_remaining[:, 0], points_remaining[:, 1], points_remaining[:, 2]
    x_spoofed, y_spoofed, z_spoofed = points_spoofed[:, 0], points_spoofed[:, 1], points_spoofed[:, 2]

    return x_remaining, y_remaining, z_remaining, x_spoofed, y_spoofed, z_spoofed
    
def injection_main(pointcloud, largest_score_angle, spoofing_range, wall_dist):
    points_remaining, points_spoofed = injection_simulation(pointcloud, largest_score_angle, spoofing_range, wall_dist)

    x_remaining, y_remaining, z_remaining = points_remaining[:, 0], points_remaining[:, 1], points_remaining[:, 2]
    x_spoofed, y_spoofed, z_spoofed = points_spoofed[:, 0], points_spoofed[:, 1], points_spoofed[:, 2]

    return x_remaining, y_remaining, z_remaining, x_spoofed, y_spoofed, z_spoofed

def dynamic_injection_main(pointcloud, timestamp, largest_score_angle, spoofing_range):
    wall_dist = set_distance(timestamp)

    points_remaining, points_spoofed = injection_simulation(pointcloud, largest_score_angle, spoofing_range, wall_dist)
    
    x_remaining, y_remaining, z_remaining = points_remaining[:, 0], points_remaining[:, 1], points_remaining[:, 2]
    x_spoofed, y_spoofed, z_spoofed = points_spoofed[:, 0], points_spoofed[:, 1], points_spoofed[:, 2]

    return x_remaining, y_remaining, z_remaining, x_spoofed, y_spoofed, z_spoofed
    #return x_spoofed, y_spoofed, z_spoofed