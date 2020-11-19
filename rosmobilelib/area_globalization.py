import numpy as np
import collections
import math
from sklearn.cluster import DBSCAN

from typing import Tuple, List, Sequence

import time

COLOR_OBSTACLE = 100
COLOR_UNKNOWN = -1
COLOR_PASSABLE = 0

def get_test_img():
    im = np.full([500, 500], COLOR_UNKNOWN, dtype=int)
    im[20:30,20:30] = COLOR_PASSABLE
    im[21,20] = COLOR_OBSTACLE
    im[2,2:10] = COLOR_OBSTACLE
    im[4,4] = COLOR_OBSTACLE
    im[4,35] = COLOR_OBSTACLE
    im[42,7] = COLOR_OBSTACLE
    im[48, 40] = COLOR_OBSTACLE
    im[25,1] = COLOR_OBSTACLE
    im[22,26] = COLOR_OBSTACLE
    im[23,25] = COLOR_OBSTACLE
    im[24,24] = COLOR_OBSTACLE
    im[29,29] = COLOR_OBSTACLE

    im[455,455] = COLOR_OBSTACLE
    # im[451,456] = COLOR_OBSTACLE
    return im

class PointCluster():
    def __init__(self, img):
        self.img = img

    ### must be you return index because to return cluster's points is very slowly
    def get_clusters(self, eps, target, erea_start=0, erea_end=None):
        if erea_end is None: 
            erea_end = len(self.img)-1
        points = np.hstack([np.array([n]).T for n in np.where(self.img==target)])
        dbs = DBSCAN(eps=eps, min_samples=1).fit(points).labels_
        clusters = [[] for i in range(max(dbs)+1)]
        for i, v in enumerate(dbs):
            clusters[v].append(points[i])
        return clusters

class ObstacleMap():
    def __init__(self, map_img):
        self.map_img = map_img

    def trim(self, start: tuple, end: tuple) -> np.ndarray:
        return self.map_img[start[0]:end[0]+1, start[1]:end[1]+1]

class Obstacle():
    def __init__(self, ob_id, super_map: ObstacleMap, ob_threshold=0, points: List[np.ndarray]=[]):
        self.ob_id = ob_id
        self.super_map = super_map
        self.ob_threshhold = ob_threshold
        self.points = points ## have coordinates of xy
        self.convex_index, self.convex_points = get_convex_points(points)
        self.convex_area = get_hull_area(self.convex_points)
        self.convex_gravity = get_center_of_gravity(self.convex_points)
        self.minimap_start, self.minimap_end, self.minimap = self.minimapping()
        self.diameter = math.sqrt((self.minimap_end[0]+1-self.minimap_start[0])**2+(self.minimap_end[1]+1-self.minimap_start[1])**2)
        self.child: List[int] = None ## have ob_id
        self.child_threshold = None

    @property
    def have_child(self):
        return self.child is not None

    def minimapping(self) -> Tuple[int, int, np.ndarray]:
        lp = np.zeros_like(self.points)
        for i, p in enumerate(self.points):
            lp[i] = [p[0], p[1]]

        max_i = np.max(lp[:,0])
        min_i = np.min(lp[:,0])
        max_j = np.max(lp[:,1])
        min_j = np.min(lp[:,1])

        pt_min = (min_i,min_j)
        pt_max = (max_i,max_j)

        return pt_min, pt_max, self.super_map.trim(pt_min, pt_max)

    def set_child(self, threshhold, target):
        self.child_threshold = threshhold
        self.child = get_obstacles(self.minimap, threshhold, target)
    
def get_convex_points(points: Sequence[Sequence]) -> Tuple[Sequence[int], np.ndarray]:
    if len(points) <= 2:
        return [i for i in range(len(points))], np.array(points)
    angl = np.zeros([len(points)])
    angl[0] = -999
    for i in range(1, len(points)):
        angl[i] = get_declination(points[0], points[i])
        index = np.argsort(angl)

    vec = lambda m, s: [points[s][0]-points[m][0], points[s][1]-points[m][1]]

    hull = collections.deque()
    hull.append(index[0])
    hull.append(index[1])

    for i in range(2, len(index)):
        while True:
            tmpi = hull.pop()
            last = hull[-1]
            tcross = np.cross(vec(last,tmpi), vec(tmpi,index[i]))
            #print(tmpi, index[i], tcross)
            if tcross >= 0:
                hull.append(tmpi)
                hull.append(index[i])
                break

    convex_pts = np.zeros([len(hull), 2], dtype=int)
    for i, h in enumerate(hull):
        convex_pts[i] = (points[h][1], points[h][0])
    #print(convex_pts)
    return hull, convex_pts

## https://imagingsolution.net/math/calc_n_point_area/
def get_hull_area(points) -> float:
    if len(points) <= 2:
        return 0.
    t = [np.cross(points[i], points[0]) if i == len(points)-1 else np.cross(points[i], points[i+1]) for i in range(len(points))]
    return np.abs(np.sum(t))*0.5

## http://izumi-math.jp/F_Nakamura/heso/heso3.htm
def get_center_of_gravity(points) -> np.ndarray:
    if len(points) == 1:
        return points[0]
    elif len(points) == 2:
        return np.mean(points, axis=0)

    sk = np.zeros([len(points)-2])
    gk = np.zeros([len(points)-2, len(points[0])])
    for i in range(1, len(points)-1):
        sk[i-1] = np.cross(points[i]-points[0], points[i+1]-points[0])
        gk[i-1] = np.mean(np.vstack(([points[0]],points[i],points[i+1])), axis=0)

    if np.sum(np.abs(sk)) == 0:
        return np.mean(points, axis=0)
    else:
        return np.average(gk, axis=0, weights=np.abs(sk)*0.5)
    
def get_declination(p1: tuple, p2: tuple) -> float:
    return math.atan2(p2[1]-p1[1], p2[0]-p1[0])

def get_obstacles(img, threshhold, target) -> List[Obstacle]:
    pc = PointCluster(img)
    om = ObstacleMap(img)
    clusters = pc.get_clusters(threshhold, target)
    # print(clusters)
    # ob_idとかちゃんとセットしよう
    obs = [None]*len(clusters)
    for i, o in enumerate(clusters):
        obs[i] = Obstacle(ob_id=i, super_map=om, ob_threshold=threshhold, points=o)
    return obs

def main():
    st = time.time()
    img = get_test_img()
    obs = get_obstacles(img, 4, COLOR_PASSABLE)
    for o in obs:
        print(f'==\np:{o.convex_points},\narea:{o.convex_area},\ngra:{o.convex_gravity}')
    print(time.time()-st)

if __name__ == "__main__":
    main()