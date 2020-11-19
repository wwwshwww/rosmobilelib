import numpy as np
from collections import deque

from typing import Tuple, Sequence

COLOR_UNKNOWN = -1
COLOR_OBSTACLE = 100
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

class AreaChecker():
    pad_size = 1
    def __init__(self, occupancy_grid: np.ndarray, position: Tuple[int]):
        self.sig_map, self.start, self.stop = AreaChecker.significant_trim(occupancy_grid)
        print(f'sigmap:{self.sig_map.shape}, start:{self.start}, stop:{self.stop}')
        pad_tup = tuple((AreaChecker.pad_size,0) for _ in range(len(position)))
        self.padded_map = np.pad(self.sig_map, pad_tup, constant_values=(-2,0))
        self.is_in_closed, self.color_map = self._coloring(position)

    @staticmethod
    def significant_trim(occupancy_grid: np.ndarray, extra_width: int=1):
        sig = np.where(np.logical_or(occupancy_grid==COLOR_OBSTACLE,occupancy_grid==COLOR_PASSABLE))
        imin = np.min(sig, axis=1)-extra_width
        imin = np.max(np.vstack((imin, [0 for _ in range(len(imin))])), axis=0)
        imax = np.max(sig, axis=1)+extra_width
        imax = np.min(np.vstack((imax, [i-1 for i in occupancy_grid.shape])), axis=0)
        sltp = tuple(slice(imin[i], imax[i]+1) for i in range(len(imin)))
        return occupancy_grid[sltp], imin, imax
    
    ## is (position) in closed surface:
    # True: in closed curve by obstacles, False: surface of passable area touch unknown area
    def _coloring(self, position) -> Tuple[bool, np.ndarray]:
        ps = AreaChecker.pad_size
        padded_shape = self.padded_map.shape
        pos = tuple(position[i]-self.start[i] for i in range(len(position)))
        dim = len(pos)
        img_map = np.full_like(self.padded_map, -2)
        used = np.zeros_like(img_map)
        valid = lambda pp: all(tuple(0 <= pp[i] and pp[i] < padded_shape[i] for i in range(dim)))
        
        q = deque()
        pos_tup = tuple(p+ps for p in pos)
        used[pos_tup] = True
        q.append(pos_tup)
        while len(q) != 0:
            p = q.pop()
            img_map[p] = self.padded_map[p]

            pl = [None]*(dim*2)
            for i in range(dim):
                pl[i] = tuple(p[j]+(i==j) for j in range(dim))
                pl[i+1*dim] = tuple(p[j]-(i==j) for j in range(dim))
            
            for ppp in pl:
                if valid(ppp) and not used[ppp]:
                    if self.padded_map[ppp] == COLOR_PASSABLE:
                        used[ppp] = True
                        q.append(ppp)
                    elif self.padded_map[ppp] == COLOR_UNKNOWN:
                        used[ppp] = True
                        img_map[ppp] = COLOR_UNKNOWN
                    elif self.padded_map[ppp] == COLOR_OBSTACLE:
                        used[ppp] = True
                        img_map[ppp] = COLOR_OBSTACLE
        
        inf = tuple(0 for _ in range(dim))
        target = tuple(p+ps for p in pos)
        col_slice = tuple(slice(ps,padded_shape[i]) for i in range(dim))

        return not(img_map[inf]==img_map[target] or COLOR_UNKNOWN in img_map), img_map[col_slice]

class AbstMap():
    def __init__(self, occupancy_grid: np.ndarray, split_shape):
        self.pad_value = -2
        self.pad_widths, self.map_data = self.padding_for_split(occupancy_grid, split_shape, self.pad_value)
        self.split_shape = split_shape
        self.resolutions = np.array(self.map_data.shape)//split_shape
        self.resol_par_mesh = np.prod(self.resolutions)

        self.density_obstacle = self.density_mesh(target=COLOR_OBSTACLE)
        self.density_road = self.density_mesh(target=COLOR_PASSABLE)
        self.density_unknown = self.density_mesh(target=COLOR_UNKNOWN)

    @staticmethod
    def padding_for_split(img, split_shape, pad_value) -> Tuple[Tuple, np.ndarray]:
        mods = np.array(img.shape)%split_shape
        if all(mods==0):
            return tuple(0 for _ in range(len(split_shape))), np.array(img)
        elif any(mods==0):
            mods = np.array([mods[i] if not mods[i]==0 else split_shape[i] for i in range(len(split_shape))])
        pad_widths = tuple(split_shape-mods)
        ## padding at top
        return pad_widths, np.pad(img, tuple((i,0) for i in pad_widths), constant_values=(pad_value,pad_value))

    def get_abst_index(self, super_index):
        return (np.array(super_index)+self.pad_widths)//self.resolutions

    def trim(self, index: Tuple[int]):
        trim_slice = tuple(slice(n*self.resolutions[i], (n+1)*self.resolutions[i]) for i,n in enumerate(index))
        return self.map_data[trim_slice]

    def _df(self, arr, func: callable, kwargs=None, index_list=None):
        if index_list is None:
            index_list = []
        if len(index_list)==len(self.split_shape):
            ti = tuple(index_list)
            t = self.trim(ti)
            arr[ti] = func(t, **kwargs)
            return
        
        for i in range(self.split_shape[len(index_list)]):
            ti = list(index_list)
            ti.append(i)
            self._df(arr, func, kwargs, ti)
        
    def _density(self, arr: np.ndarray, val) -> float:
        return len(np.where(arr==val)[0])/self.resol_par_mesh

    def density_mesh(self, target) -> np.ndarray:
        dens = np.zeros(self.split_shape)
        self._df(dens, self._density, {'val':target})

        ## for 2D map
        # for i in range(len(dens)):
        #     for j in range(len(dens[0])):
        #         t = self.trim((i,j))
        #         dens[i,j] = len(np.where(t==target)[0])/self.resol_par_mesh

        return dens

def main():
    img = get_test_img()
    abst = AbstMap(img, (2,3))
    print(f'{abst.map_data},\n{abst.map_data.shape},\n{abst.resolutions}')
    print(f'obs: \n{abst.density_obstacle}, \nroad: \n{abst.density_road}')
    print(f'get abst index from (499,499): {abst.get_abst_index((499,499))}')

    ac = AreaChecker(img, (25,25))
    print(ac.sig_map, '\n', ac.color_map)
    print(f'is (25,25) in closed surface: {ac.is_in_closed}')
    print(np.where(ac.color_map==COLOR_PASSABLE))

if __name__ == '__main__':
    main()