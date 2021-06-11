
# T3D Visualiser

A tool for visualising transforms and points (basically a wrapper around the open3D visualiser).

* Free software: MIT license

Features
--------
* Uses the ROS coordinate system.
* Easy live updates of transforms and pointclouds.
* Labels for the different poses/transforms within the viewer.

Example
--------
Example usage can be found in `t3d_visualiser/t3d_visualiser.py`

```python
import time
import numpy as np
from t3d_visualiser.t3d_visualiser import Visualiser


def run(visualiser):
    """Example run function"""
    T_wc = np.identity(4)
    T_wc[:3, 3] = np.array([1, 1.5, 2])

    visualiser.add_frame("T_wc", T_wc, (1, 0, 0))
    visualiser.add_frame("T_w", np.identity(4), (1, 0, 0))
    visualiser.set_start_view(zoom=2)  # Has to be after frames are added
    visualiser.add_pointcloud("all", np.array([[0, 0, 0]]), (1, 0, 0))
    visualiser.add_pointcloud("current", np.array([[0, 0, 0]]), (0, 1, 0))

    current_point = T_wc[:3, 3] + np.array([1.5, 0, 0])
    points = [current_point]
    for _ in range(1000):
        T_wc[1, 3] += 0.02
        visualiser.update_frame("T_wc", T_wc)
        current_point = T_wc[:3, 3] + np.array([1.5, 0, 0])

        visualiser.pointclouds["all"] = np.array(points)
        visualiser.pointclouds["current"] = np.array([current_point])
        points.append(current_point)

        time.sleep(1)
    visualiser.stop()


def main():
    vis = Visualiser(run)
    vis.run()

```
