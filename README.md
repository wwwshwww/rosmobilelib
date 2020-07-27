# rosmobilelib

Enable to struct movement robotic action with ROS, the open-source robotic middleware. This library make able to run that by Python3 code on external of ROS network and workspace.

This is depended [roslibpy](https://github.com/gramaziokohler/roslibpy) that allows ROS programing without defining as ROS Node by using [rospy](http://wiki.ros.org/rospy).

## Main Features

- Manage features to set ROS robotic movement simply.
- Dynamic schedule to Goal with sending/waiting of Action.
- Provide some feature that use for programming with ROS such as coordinate transformation of **TF**.
- Give support to synchronize to single callback for needs to subscribed multi topic such as stereo camera system.

## Installation

To install rosmobilelib, use `pip`:

```
pip install rosmobilelib
```

or

```
pip install rosmobilelib --extra-index-url https://test.pypi.org/simple
```

## Documentation

Details coming soon. For now, just watch down and get through it.

### Example implementation: 

Import libraries.

```
import roslibpy as rlp
from rosmobilelib import MobileClient
```

Prepare connection with roslibpy. If you desire details see [here](https://roslibpy.readthedocs.io/en/latest/examples.html).

```
client = rlp.Ros('localhost', port=9090)
lm1 = lambda: print('is ROS connected: ', client.is_connected)
client.on_ready(lm1)
client.run()
```

Define `MobileClient` object and wait for to subscribe needs topics.

```
lm2 = lambda r: print('reached goal', r)
ms = MobileClient(client, lm2, odom_topic='/odom', map_topic='/map')
ms.wait_for_ready()
```

Use dynamic FCFS scheduler. Set goal and make able to execute goals. You can set goal any time not only after call start().

Details:

- start(), stop(): make scheduling queue executable/inexecutable

```
ms.start()

# set scheduler a goal that go ahead 0.5 from robot body
ms.set_goal_relative_xy(0.5, 0, is_dynamic=False)

# set relative pos(x:front:-0.5, y:left:1) based basis vector that decided dynamic after previous executed
ms.set_goal_relative_xy(-0.5, 1, is_dynamic=True)

# set goal directly with world frame's pose
ms.set_goal(ms.get_vec_q(-0.4,-0.6,0), ms.get_rot_q(0,0,math.pi/2))

time.sleep(60)

ms.stop()
```

There are other way to wait for time until reach goal. Exchange `time.sleep(n)` to `ms.wait_for_execute_all()`.

```
...
ms.wait_for_execute_all()
...
```
