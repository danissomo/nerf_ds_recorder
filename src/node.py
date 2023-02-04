#! /usr/bin/env python3
import rospy
import os
from datetime import datetime
import argparse
from robot_controller import PathGenerator, PathFolower
from image_handler import ImageHandler
import numpy as np
DATASET_CONSISTANCY= False
BASE_DIR = None

def parse_arg():
    parser = argparse.ArgumentParser(
        prog= 'nerf-ds-recorder',
        description= 'records dataset from robot for nerf learning',
        epilog='''
        Made by Daniil P. github: https://github.com/danissomo
        '''
    )
    
    parser.add_argument('-i', '--img-camera', help='camera image topic name', required=True)
    parser.add_argument('-l', '--depth-camera', help='camera depth image topic name', required=True)
    parser.add_argument('-n', '--number', default=100, help='count of pictures', type=int)
    parser.add_argument('-d', '--dir', default='./data',    help='directory for saving data')
    parser.add_argument('-t', '--trajectory', choices=['circle', 'rectangle', 'waypoints'], default='circle')
    parser.add_argument('--hfov', default=1.20428)
    parser.add_argument('--ip', help='robot ip', required=True)

    # circle settings
    parser.add_argument('-r', '--radius', type=float,  help='circle radius, if trajectory is circle', default=None)
    parser.add_argument('-p', '--position', nargs=3,  help='circle position, if trajectory is circle', default=None)
    parser.add_argument('--normal', nargs=3,  help='circle normal, if trajectory is circle', default=None)


    #something that going from roslaunch
    parser.add_argument('__name', default=None)
    parser.add_argument('__log', default=None)

    args = parser.parse_args()

    if args.trajectory == 'circle' and (args.radius is None or args.position is None or args.normal is None):
        parser.error('circle req -r and -p')
    if args.position is not None:
        args.position = [float(i) for i in args.position]  
    if args.normal is not None:
        args.normal = [float(i) for i in args.normal]  
    return args

def cancel():
    global DATASET_CONSISTANCY, BASE_DIR
    if not DATASET_CONSISTANCY and BASE_DIR is not None:
        rospy.logwarn(f'inconsistent dataset removing folder {BASE_DIR}')
        import shutil
        shutil.rmtree(BASE_DIR)


def generate_dataset(pf : PathFolower, img_handler: ImageHandler, label, hfov, samples, save_dir, path_type = 'circle', kwargs = None):
    '''
    pf - PathFolower class, that inmplements generator
    img_handler - ImageHandler class that captures sensor imges
    label - name of subdataset
    hfov - horizontal angle of cameera
    path_type  - [circle, rectangle, waypoints]
    '''
    rospy.loginfo(f'recording {label}, lenght {samples}, will be saved to {save_dir}')
    if path_type == 'circle':
        v = np.array( kwargs.normal)/np.linalg.norm( kwargs.normal) 
        p = np.array(kwargs.position)
        point = v*1.0 + p
        path = PathGenerator.Circle(kwargs.radius, kwargs.normal, kwargs.position, samples, point)
    elif path_type == 'waypoints':
        path = PathGenerator.Waypoints(samples, pf)
        point = None
    else:
        rospy.logfatal(f'{path_type} path not implemented')
        exit()
    PathGenerator.draw_path(path)
    PathGenerator.draw_path(path, f'{save_dir}/path_{label}.png')
    ds = dict()
    ds['camera_angle_x'] = hfov
    ds['frames'] = []
    rospy.loginfo('press enter to start recording')
    input()
    
    for i, tf in enumerate(pf.follow(path)):
        color, depth = img_handler.get_data()
        color.save(f'{save_dir}/{label}/r_{i}.png')
        depth.save(f'{save_dir}/{label}/r_{i}_depth.png')
        frame = dict()
        frame['file_path'] = f'{label}/r_{i}'
        frame['transform_matrix'] = tf.tolist()
        ds['frames'].append(frame)
    import json
    with open(f'{save_dir}/transforms_{label}.json', 'w') as f:
        json.dump(ds, f)


def main():
    args = parse_arg()
    rospy.init_node("ds_recorder", log_level=rospy.DEBUG)
    rospy.on_shutdown(cancel)

    

    # creating directories for dataset
    now = datetime.now()
    time_stamp = now.strftime("%m-%d-%Y_%H-%M-%S")
    base_dir = f'{args.dir}/{time_stamp}'
    test_dir = f'{args.dir}/{time_stamp}/test'
    train_dir = f'{args.dir}/{time_stamp}/train'
    val_dir = f'{args.dir}/{time_stamp}/val'
    
    if not os.path.exists(args.dir):
        rospy.loginfo(f"directory for ds is not exists, creating it on {args.dir}")
        os.makedirs(args.dir)
    os.makedirs(test_dir)
    os.makedirs(train_dir)
    os.makedirs(val_dir)
    ##

    global BASE_DIR
    BASE_DIR = base_dir

    pf = PathFolower(args.ip) 
    img_handler = ImageHandler(args.img_camera, args.depth_camera)
    for c, l in zip([args.number, args.number // 2, args.number // 2], ['train', 'test', 'val']):
        generate_dataset(pf, img_handler, l, args.hfov, c, base_dir, args.trajectory, args)

    global DATASET_CONSISTANCY
    DATASET_CONSISTANCY = True    

if __name__ == "__main__":
    main()


