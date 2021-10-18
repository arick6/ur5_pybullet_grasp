#!/usr/bin/env python3
import os

import numpy as np
import pybullet as p
from ros_np_multiarray import ros_np_trans as rnp

from tqdm import tqdm
from env import ClutteredPushGrasp
from utilities import YCBModels, Camera
import rospy
import cv2
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Point

def heuristic_demo():
    ycb_models = YCBModels(
        os.path.join('/home/iclab-isaac/ur5_pybullet_grasp/src/pybullet_ur5/data/ycb', '**', 'textured-decmp.obj'),
    )
    camera = Camera((0, -0.5, 1.5), 0.1, 5, (320, 320), 40)

    env = ClutteredPushGrasp(ycb_models, vis=True, num_objs=5, gripper_type='85')
    p.resetDebugVisualizerCamera(2.0, -270., -60., (0., 0., 0.))
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # Shadows on/off

    (rgb, depth, seg) = env.reset(camera)
    
    step_cnt = 0
    while True:
        keys = p.getKeyboardEvents()
        h_, w_ = np.unravel_index(depth.argmin(), depth.shape)
        x, y, z = camera.rgbd_2_world(w_, h_, depth[h_, w_])

        p.addUserDebugLine([x, y, 0], [x, y, z], [0, 1, 0])
        p.addUserDebugLine([x, y, z], [x, y, z+0.05], [1, 0, 0])
        #publish data to ros
        pub_rgb, pub_depth, pub_object_pos= ROS_init()
        rgb_ros_data = rnp.to_multiarray_i32(np.array(rgb, dtype=np.int32))
        depth_ros_data = rnp.to_multiarray_f32(np.array(depth, dtype=np.float32))
        rate = rospy.Rate(5)
        pub_rgb.publish(rgb_ros_data)
        pub_depth.publish(depth_ros_data)
        points = Point()
        points.x = x
        points.y = y
        points.z = z
        pub_object_pos.publish(points)
        rate.sleep()
        print("++++++++++++++++")
        (rgb, depth, seg), reward, done, info = env.step(camera,(x, y, z), 1, 'grasp')

        #switch camera with 5,6,7,8
        if 53 in keys:
            p.resetDebugVisualizerCamera(2.0, -270., -60., (0., 0., 0.))
        if 54 in keys:
            p.resetDebugVisualizerCamera(5.0, -300., -60., (0., 0., 0.))
        if 55 in keys:
            p.resetDebugVisualizerCamera(2.0, -60., -60., (0., 0., 0.))
        if 56 in keys:
            p.resetDebugVisualizerCamera(4.0, -270., -60., (0., 0., 0.))

        
        #switch rgbd image with 1,2,3,4
        if 49 in keys:
            camera = Camera((0, -0.5, 1.5), 0.1, 5, (320, 320), 40)
            env.reset_camera(camera)
        if 50 in keys:
            camera = Camera((0.0, -0.5, 1.5), 0.1, 7, (320, 320), 50)
            env.reset_camera(camera)
        if 51 in keys:
            camera = Camera((0, -0.5, 1.5), 0.2, 3, (320, 320), 40)
            env.reset_camera(camera)
        if 52 in keys:
            camera = Camera((0.1, -0.5, 1.5), 0.4, 5, (320, 320), 40)
            env.reset_camera(camera)


        print('Step %d, grasp at %.2f,%.2f,%.2f, reward %f, done %s, info %s' %
              (step_cnt, x, y, z, reward, done, info))
        step_cnt += 1
        # time.sleep(3)


def user_control_demo():
    ycb_models = YCBModels(
        os.path.join('/home/iclab-isaac/ur5_pybullet_grasp/src/pybullet_ur5/data/ycb', '**', 'textured-decmp.obj'),
    )
    camera = Camera((0, -0.5, 1.5), 0.1, 5, (320, 320), 40)

    # env = ClutteredPushGrasp(ycb_models, camera, vis=True, num_objs=5, gripper_type='85')
    env = ClutteredPushGrasp(ycb_models, vis=True, num_objs=5, gripper_type='85')

    p.resetDebugVisualizerCamera(2.0, -270., -60., (0., 0., 0.))
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # Shadows on/off
    p.addUserDebugLine([0, -0.5, 0], [0, -0.5, 1.1], [0, 1, 0])

    rgb, depth, seg = env.reset(camera)
    while True:

        # print(rgb)
        # print("----------------")
        # print(depth)
        # key control
        keys = p.getKeyboardEvents()

        #publish data to ros
        pub_rgb, pub_depth,pub_object_pos = ROS_init()
        rgb_ros_data = rnp.to_multiarray_i32(np.array(rgb, dtype=np.int32))
        depth_ros_data = rnp.to_multiarray_f32(np.array(depth, dtype=np.float32))
        rate = rospy.Rate(5)
        pub_rgb.publish(rgb_ros_data)
        pub_depth.publish(depth_ros_data)
        rate.sleep()
        if 57 in keys:
            # print(rgb)
            # print(type(rgb))
            # rgb_mat = np.ndarray((3, 320, 320), dtype=int)
            rgb_mat = rgb
            rgb_mat = np.ndarray((3, 320, 320), dtype=int)
            rgb_mat = rgb_mat.astype(np.uint8)
            rgb_mat_pic_red, rgb_mat_pic_green, rgb_mat_pic_blue = rgb_mat
            new_rgb = np.dstack([rgb_mat_pic_red, rgb_mat_pic_green, rgb_mat_pic_blue])
            # print(rgb_mat)
            cv2.imshow("WindowNameHere", new_rgb)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            # if 48 in keys:
            #     cv2.destroyWindow("WindowNameHere")
            #     pass
        if 48 in keys:
            print(depth)
            # print(type(rgb))
            # rgb_mat = np.ndarray((3, 320, 320), dtype=int)
            depth = np.ndarray((3, 320, 320), dtype=int)
            depth = depth.astype(np.uint8)
            rgb_mat_pic_red, rgb_mat_pic_green, rgb_mat_pic_blue = rgb
            new_rgb = np.dstack([rgb_mat_pic_red, rgb_mat_pic_green, rgb_mat_pic_blue])
            # print(rgb_mat)
            cv2.imshow("WindowNameHere", new_rgb)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            # if 48 in keys:
            #     cv2.destroyWindow("WindowNameHere")
            #     pass

        #switch camera with 5,6,7,8
        if 53 in keys:
            p.resetDebugVisualizerCamera(2.0, -270., -60., (0., 0., 0.))
        if 54 in keys:
            p.resetDebugVisualizerCamera(5.0, -300., -60., (0., 0., 0.))
        if 55 in keys:
            p.resetDebugVisualizerCamera(2.0, -60., -60., (0., 0., 0.))
        if 56 in keys:
            p.resetDebugVisualizerCamera(4.0, -270., -60., (0., 0., 0.))

        
        #switch rgbd image with 1,2,3,4
        if 49 in keys:
            camera = Camera((0, -0.5, 1.5), 0.1, 5, (320, 320), 40)
            rgb, depth, seg = env.reset_camera(camera)
        if 50 in keys:
            camera = Camera((0.0, -0.5, 1.5), 0.1, 7, (320, 320), 50)
            rgb, depth, seg = env.reset_camera(camera)
        if 51 in keys:
            camera = Camera((0, -0.5, 1.5), 0.2, 3, (320, 320), 40)
            rgb, depth, seg = env.reset_camera(camera)
        if 52 in keys:
            camera = Camera((0.1, -0.5, 1.5), 0.4, 5, (320, 320), 40)
            rgb, depth, seg = env.reset_camera(camera)

        # key "Z" is down and hold
        if (122 in keys) and (keys[122] == 3):
            print('Grasping...')
            if env.close_gripper(check_contact=True):
                print('Grasped!')
        # key R
        if 114 in keys:
            env.open_gripper()
        # time.sleep(1 / 120.)

def ROS_init():

    # rgb_ros_data = rnp.to_multiarray_i32(np.array(rgb_data, dtype=np.int32))
    # depth_ros_data = rnp.to_multiarray_f32(np.array(depth_data, dtype=np.float32))
    pub_rgb = rospy.Publisher('/projected_image/rgb', Int32MultiArray)
    pub_depth = rospy.Publisher('/projected_image/depth', Float32MultiArray)
    pub_object_pos = rospy.Publisher('/projected_image/pos', Point)
    rospy.init_node('pybullet_info', anonymous=True)
    # rate = rospy.Rate(5)
    # while not rospy.is_shutdown():
    #     pub_rgb.publish(rgb_ros_data)
    #     pub_depth.publish(depth_ros_data)
    #     rate.sleep()
    return pub_rgb,pub_depth,pub_object_pos


if __name__ == '__main__':
    #user_control_demo()
    heuristic_demo()
