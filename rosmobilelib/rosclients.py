from .rostools import ActionScheduler, TimeSynchronizer, ToCv

import roslibpy
import numpy as np
import math
import quaternion # numpy-quaternion
import time

from typing import Tuple, List, Dict

class CameraListener():
    def __init__(self, ros_client: roslibpy.Ros, callback: callable, *topics: str, queue_size: int=100, allow_headerless: float=0.1):
        msg_type = 'sensor_msgs/CompressedImage'
        self.client_callback = callback
        self.tps = [roslibpy.Topic(ros_client, name, msg_type) for name in topics]
        if len(self.tps) > 1:
            self.subscriber = TimeSynchronizer(self.tps, self.msg_to_cv2, queue_size, allow_headerless)
        else:
            self.subscriber = roslibpy.Topic(ros_client, self.tps[0], msg_type)
            self.subscriber.subscribe(self.msg_to_cv2)
    
    def msg_to_cv2(self, *msgs: Dict):
        ims = [ToCv.compressed_imgmsg_to_cv2(m) for m in msgs]
        self.client_callback(*ims)

## class that create and send goal of pose to move_base
class MobileClient():
    def __init__(self, ros_client: roslibpy.Ros, goal_callback: callable, odom_topic: str='/odom', map_topic: str='/map'):
        self.ros_client = ros_client
        self.result_callback = goal_callback
        self.mb_scheduler = ActionScheduler(self.ros_client, '/move_base', 'move_base_msgs/MoveBaseAction', self.result_callback)

        self.is_get_map = False
        self.map_listener = roslibpy.Topic(self.ros_client, map_topic, 'nav_msgs/OccupancyGrid')
        self.map_listener.subscribe(self._update_map)
        
        self.is_get_odom = False
        self.odom_listener = roslibpy.Topic(self.ros_client, odom_topic, 'nav_msgs/Odometry')
        self.odom_listener.subscribe(self._update_odometry)

    @property
    def is_reached(self):
        return not self.mb_scheduler.state>>1&1

    @property
    def is_freetime(self):
        return self.mb_scheduler.state&1

    def wait_for_execute_all(self, rate=0.5):
        while not (self.is_freetime and self.is_reached):
            time.sleep(rate)

    ## need to make to be changed design pattern 'update_**'
    def _update_map(self, message):
        self.map_header = message['header']
        self.map_info = message['info']
        self.map_resolution = self.map_info['resolution']
        self.map_resol_i = self.map_info['height']//2
        self.map_resol_j = self.map_info['width']//2
        self.map_data = np.array(message['data']).reshape([self.map_info['height'],self.map_info['width']]).T
        self.is_get_map = True

    def _update_odometry(self, message):
        pos = message['pose']['pose']['position']
        ori = message['pose']['pose']['orientation']
        self.position = np.array([pos['x'], pos['y'], pos['z']])
        self.orientation = np.quaternion(ori['w'], ori['x'], ori['y'], ori['z'])
        self.is_get_odom = True

    def wait_for_ready(self, timeout=10.0):
        print('wait for ROS message...', end=' ')
        sec = 0.001
        end_time = time.time() + timeout
        while True:
            if self.is_get_odom and self.is_get_map:
                print('got ready')
                break
            elif time.time()+sec >= end_time:
                self.ros_client.terminate()
                raise Exception('Timeout you can\'t get map or odometry.')
            time.sleep(sec)

    @staticmethod
    def get_relative_orientation(rel_vec: np.quaternion) -> np.quaternion:
        ## rotate angle (alpha,beta,gamma):(atan(y/z),atan(z/x),atan(x/y))
        ## don't change
        to_angle = (math.atan2(rel_vec.y, rel_vec.z), math.atan2(rel_vec.z, rel_vec.x), math.atan2(rel_vec.x,rel_vec.y))
        return quaternion.from_euler_angles(to_angle)

    @staticmethod
    def get_base_pose(base_vec: np.quaternion, base_orient: np.quaternion, rel_vec: np.quaternion, rel_orient: np.quaternion) -> Tuple[np.quaternion, np.quaternion]:
        t = (-base_orient) * rel_vec * (-base_orient).conj()
        goal_vec = base_vec+t
        goal_orient = base_orient*rel_orient
        return goal_vec, goal_orient

    def get_vec_q(x: float, y: float, z: float) -> np.quaternion:
        return np.quaternion(0,x,y,z)

    def get_rot_q(x: float, y: float, z: float) -> np.quaternion:
        return quaternion.from_euler_angles(x,y,z)

    def get_orientation_from_body(self, position: np.quaternion):
        bp = np.quaternion(0, *self.position)
        return self.get_relative_orientation(position-bp)

    def get_base_pose_from_body(self, position: np.quaternion, orientation=np.quaternion(1,0,0,0)):
        bp = np.quaternion(0, *self.position)
        return self.get_base_pose(bp, self.orientation, position, orientation)

    ## map img's (i,j) to base map's (x,y)
    def get_coordinates_from_index(self, ij: tuple) -> Tuple[float, float]:
        p = (ij[0]-self.map_resol_i)*self.map_resolution, (ij[1]-self.map_resol_j)*self.map_resolution
        print(f'ij {ij} to {p}')
        return p

    ## base map's (x,y) to base map's (i,j)
    def get_index_from_coordinates(self, xy: tuple) -> Tuple[int, int]:
        p = self.map_resol_i+int(xy[0]/self.map_resolution), self.map_resol_j+int(xy[1]/self.map_resolution)
        print(f'xy {xy} to {p}')
        return p

    def create_message_move_base_goal(self, position: np.quaternion, orientation: np.quaternion) -> roslibpy.Message:
        message = {
            'target_pose': {
                'header': self.map_header,
                'pose': {
                    'position': {
                        'x': position.x,
                        'y': position.y,
                        'z': position.z
                    },
                    'orientation': {
                        'x': orientation.x,
                        'y': orientation.y,
                        'z': orientation.z,
                        'w': orientation.w
                    }
                }
            }
        }
        return roslibpy.Message(message)

    ## set goal message that simple ahead pose
    def set_goal_relative_xy(self, x, y, angle=None, is_dynamic=False):
        rel_pos_2d = np.quaternion(0,x,y,0)
        rel_ori = quaternion.from_euler_angles(0, 0, math.atan2(x,y))
        pos, ori = self.get_base_pose_from_body(rel_pos_2d, rel_ori*self.orientation)

        def inner():
            if is_dynamic:
                dpos, dori = self.get_base_pose_from_body(rel_pos_2d, rel_ori*self.orientation)
                self._log_set(dpos)
                return self.create_message_move_base_goal(dpos, dori)
            else:
                self._log_set(pos)
                return self.create_message_move_base_goal(pos, ori)

        self.mb_scheduler.append_goal(inner)

    def set_goal(self, position: np.quaternion, orientation: np.quaternion):
        lm = lambda: self.create_message_move_base_goal(position, orientation)
        self._log_set(position)
        self.mb_scheduler.append_goal(lm)

    def _log_set(self, pos: np.quaternion):
        print(f'scheduling ({pos.x},{pos.y},{pos.z})')

    def start(self):
        self.mb_scheduler.run()

    def stop(self):
        self.mb_scheduler.cancel()

def main():
    # rc = roslibpy.Ros('localhost', port=9090)
    client = roslibpy.Ros('10.244.1.176', port=9090)
    client.on_ready(lambda: print('is ROS connected: ', client.is_connected))
    client.run()

    # topic_o = '/odometry/filtered'
    topic_o = '/odom'
    ms = MobileClient(client, lambda r: print('reached goal', r), odom_topic=topic_o)
    ms.wait_for_ready(timeout=80)
    print('map_header: ', ms.map_header)
    print('odom:', ms.position, [math.degrees(v) for v in quaternion.as_euler_angles(ms.orientation)])

    ## you can set goal any time not only after call start().
    ms.start() ## make goal appended to queue, executable
#     ms.set_goal_relative_xy(0.5, 0, True) ## set scheduler a goal that go ahead 0.5 from robot body
#     ms.set_goal_relative_xy(-0.5, 1) ## relative (x:front:-0.5, y:left:1)
    ms.set_goal(ms.get_vec_q(-0.4,-0.6,0), ms.get_rot_q(0,0,math.pi/2))
    time.sleep(30)
#     ms.set_goal_relative_xy(0.5, 0, True)
#     time.sleep(30)
    ms.stop()
    print('finish')

    client.terminate()

if __name__ == '__main__':
    main()

