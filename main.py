import os

import numpy as np
import pybullet as p

from tqdm import tqdm
from env import ClutteredPushGrasp
from utilities import YCBModels, Camera


def heuristic_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
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
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
    )
    camera = Camera((0, -0.5, 1.5), 0.1, 5, (320, 320), 40)

    # env = ClutteredPushGrasp(ycb_models, camera, vis=True, num_objs=5, gripper_type='85')
    env = ClutteredPushGrasp(ycb_models, vis=True, num_objs=5, gripper_type='85')

    p.resetDebugVisualizerCamera(2.0, -270., -60., (0., 0., 0.))
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)  # Shadows on/off
    p.addUserDebugLine([0, -0.5, 0], [0, -0.5, 1.1], [0, 1, 0])

    env.reset(camera)
    while True:
        env.step(camera, None, None, None, True)

        # key control
        keys = p.getKeyboardEvents()


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

        # key "Z" is down and hold
        if (122 in keys) and (keys[122] == 3):
            print('Grasping...')
            if env.close_gripper(check_contact=True):
                print('Grasped!')
        # key R
        if 114 in keys:
            env.open_gripper()
        # time.sleep(1 / 120.)


if __name__ == '__main__':
    # user_control_demo()
    heuristic_demo()
