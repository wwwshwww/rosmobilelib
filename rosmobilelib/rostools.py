import numpy as np
import roslibpy as rlp
import roslibpy.actionlib
from threading import Timer
from collections import deque

__all__ = ['ActionScheduler', 'TimeSynchronizer']

roslibpy.actionlib.DEFAULT_CONNECTION_TIMEOUT = 10

## FCFS (First Come First Served）
class ActionScheduler(Timer):
    STATUS_READY_TO_START = 0b01
    def __init__(self, ros_client: rlp.Ros, server_name: str, action_name: str, callback: callable, queue_size: int=100, rate: float=0.05, args: list=None, kwargs: dict=None): 
        Timer.__init__(self, rate, self.run, args, kwargs)
        self.ros_client = ros_client
        self.action_client = rlp.actionlib.ActionClient(ros_client, server_name, action_name)
        self.current_goal: rlp.actionlib.Goal = None
        self.callback = callback
        self.thread = None
        self.goal_queue = deque(maxlen=queue_size)
        self.rate = rate
        self.state = ActionScheduler.STATUS_READY_TO_START ## 01: queue is empty, 10: task is actived
        
    def _update_state(self):
        is_emp = 1 if len(self.goal_queue) == 0 else 0
        is_fin = 0
        if self.current_goal is not None:
            is_fin = 0 if self.current_goal.is_finished else 1
        self.state = is_emp|(is_fin<<1)

    def _check_task(self):
        self._update_state()
        self.thread = Timer(self.rate, self._check_task)
        self.thread.start()

        if not self.state&1 and not self.state>>1&1:
            message_closure = self.goal_queue.popleft()
            self.current_goal = rlp.actionlib.Goal(self.action_client, message_closure())
            self._update_state()
            self.current_goal.send(self._finish)

    def _finish(self, result):
        self._update_state()
        self.callback(result)

    ## need to change data type of goal to such as  <Goal, tag>, to be able to cancel
    def append_goal(self, message_closure):
        self.goal_queue.append(message_closure)
        self._update_state()

    def run(self):
        self._check_task()

    def cancel(self):
        if self.thread is not None:
            self.thread.cancel()
            self.thread.join()
            del self.thread

## make message that has subscribed synchronized in time approximate, before call a callback function
class TimeSynchronizer():
    class Subscriber():
        def __init(self, sync, listener: rlp.Topic, queue_size: int):
            self.sync = sync
            self.listener = listener
            self.queue = deque(maxlen=queue_size)
            self.listener.subscribe(self.cb)
        
        def cb(self, message):
            self.queue.append(message)
            self.sync.synchronize()

    def __init__(self, topics: list, callback: callable, queue_size: int, allow_headerless: float=None):
        self.listeners = [TimeSynchronizer.Subscriber(self, s, queue_size) for s in topics]
        self.callback = callback
        self.allow_headerless = allow_headerless
        self.queue = deque(maxlen=queue_size)

    def get_time(self, message) -> int:
        return message['header']['stamp']['secs']*(10**9)+message['header']['stamp']['nsecs']

    def synchronize(self):
        if not all([len(s.queue) for s in self.listeners]):
            return

        lis_len = len(self.listeners)
        di = {i:t for i,t in zip(range(lis_len), map(self.get_time, [l.queue[0] for l in self.listeners]))}
        cri_time_i = max(di, key=di.get)
        cri_time = di[cri_time_i]
        allow_time = cri_time + self.allow_headerless
        del di[cri_time_i]

        result = [None for x in range(lis_len)]
        result[cri_time_i] = self.listeners[cri_time_i].queue[0]

        for k in di.keys():
            # al = np.array([i for i in map(self.get_time, self.listeners[k].queue)])
            tq = deque()
            for i in range(len(self.listeners[k].queue)):
                nt = self.get_time(self.listeners[k].queue[i])
                if nt <= allow_time:
                    tq.append(nt)
                else:
                    break
            
            arr = np.abs(np.array(tq)-cri_time)
            abs_min_i = np.argmin(arr)
            result[k] = self.listeners[k].queue[abs_min_i]
            for i in range(abs_min_i+1):
                self.listeners[k].queue.popleft()
        
        self.callback(*result)