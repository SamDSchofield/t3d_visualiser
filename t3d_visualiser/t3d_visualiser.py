import threading
import time

import numpy as np
import open3d as o3d
import transforms3d

from t3d import t3d
import os
from pathlib import Path
from PIL import Image, ImageFont, ImageDraw
from matplotlib import font_manager




LOCK = threading.Lock()
OPENCV_2_OPENGL = np.array(
    [
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1],
    ]
)

def get_font():
    font = font_manager.findfont(font_manager.FontProperties(family=['sans-serif']))
    return font


def text_3d(text, pos, font, camera_pose=None, density=10):
    """
    Generate a 3D text point cloud used for visualization.
    :param text: content of the text
    :param pos: 3D xyz position of the text upper left corner
    :param direction: 3D normalized direction of where the text faces
    :param degree: in plane rotation of text
    :param font: Name of the font - change it according to your system
    :param font_size: size of the font
    :return: o3d.geoemtry.PointCloud object
    """
    font_dim = font.getsize(text)

    img = Image.new('RGB', font_dim, color=(255, 255, 255))
    draw = ImageDraw.Draw(img)
    draw.text((0, 0), text, font=font, fill=(0, 0, 0))
    img = np.asarray(img)
    img_mask = img[:, :, 0] < 128
    indices = np.indices([*img.shape[0:2], 1])[:, img_mask, 0].reshape(3, -1).T

    pcd = o3d.geometry.PointCloud()
    pcd.colors = o3d.utility.Vector3dVector(img[img_mask, :].astype(float) / 255.0)
    pcd.points = o3d.utility.Vector3dVector(indices / 100.0 / density)

    pos = t3d.apply_T(OPENCV_2_OPENGL, np.array([pos]))[0]
    if camera_pose is not None:
        tf = np.matmul(np.linalg.inv(camera_pose[:3, :3]), transforms3d.euler.euler2mat(*np.radians([0, 0, 90])))
        tf = np.matmul(tf, transforms3d.euler.euler2mat(*np.radians([180, 0, 0])))
        pcd.rotate(tf)
        pcd.translate(pos)
    return pcd


def draw_camera(pose):
    height = 0.15
    width = 0.2
    depth = 0.2
    points = np.array(
        [
            [0, 0, 0],
            [-width, height, depth],
            [width, height, depth],
            [width, -height, depth],
            [-width, -height, depth],
        ]
    ).reshape(-1, 3)

    lines = np.array(
        [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 4],
            [4, 1],
            [0, 1],
            [0, 2],
            [0, 3],
            [0, 4],
        ]
    )
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.paint_uniform_color([0, 0, 0])
    line_set.transform(np.matmul(pose, OPENCV_2_OPENGL))
    return line_set


def o3d_frame(transform=np.identity(4), colour=None):
    frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    if colour is not None:
        frame_mesh.paint_uniform_color(colour)
    frame_mesh.transform(transform)
    frame_mesh.transform(OPENCV_2_OPENGL)
    return frame_mesh


def optical_frame(transform, pose_2_cam=np.identity(4)):
    return o3d_frame(np.matmul(transform, pose_2_cam))


def o3d_pointcloud(points, colour=(1, 0, 0)):
    pointcloud_marker = o3d.geometry.PointCloud()
    if points is not None:
        pointcloud_marker.points = o3d.utility.Vector3dVector(
            t3d.apply_T(OPENCV_2_OPENGL, points)
        )

    pointcloud_marker.paint_uniform_color(colour)
    return pointcloud_marker


def draw_trajectory(trajectory):
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(
        t3d.apply_T(OPENCV_2_OPENGL, trajectory[:, 1:4])
    )
    indices = np.arange(len(trajectory))
    line_set.lines = o3d.utility.Vector2iVector(
        [(a, b) for a, b in zip(indices, indices[1:])]
    )
    return line_set


class Visualiser:
    def __init__(self, vo_function, *args, **kwargs):
        """Tool for visualising frames/poses and pointclouds

        Args:
            vo_function: main function to run the vo algorithm
            *args: arguments to pass to vo_function
        """
        self.running = True
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.frames = dict()
        self.frame_markers = dict()
        self.text = dict()
        self.pointclouds = dict()
        self.pointcloud_markers = dict()
        self.vis.add_geometry(o3d_frame())
        self.font = ImageFont.truetype(get_font(), 18 * 10)

        # threading.Thread(target=vo_function, args=[*args, self]).start()
        kwargs["visualiser"] = self
        threading.Thread(target=vo_function, args=args, kwargs=kwargs).start()


    def add_frame(self, id_, pose, colour):
        """Adds a frame to be visualised

        Args:
            id_: ID of the frame so it can be looked up/modified later
            pose: pose of the frame
            colour: colour of the frame specified as (r, g, b) [0-1]
        """
        self.frames[id_] = pose
        self.text[id_] = text_3d(id_, pose[:3, 3] + np.array([0.1, 0.1, 0.1]), self.font)
        self.vis.add_geometry(self.text[id_])
        self.frame_markers[id_] = o3d_frame(pose, colour)
        self.vis.add_geometry(self.frame_markers[id_])

    def add_pointcloud(self, id_, points, colour):
        self.pointclouds[id_] = points
        self.pointcloud_markers[id_] = (o3d_pointcloud(points, colour), colour)
        self.vis.add_geometry(self.pointcloud_markers[id_][0])

    def update_frame(self, id_, pose):
        """Update the pose of the frame. Should be used within the vo function.

        Args:
            id_: id of the frame to update
            pose: pose to set the frame to

        """
        self.frames[id_] = pose

    def _update_frame_markers(self):
        """Updates the visualisation so the markers reflect the poses
        specified by update_frame().

        Should be called from within visualiser.run
        """
        for id_, pose in self.frames.items():
            if id_ in self.frame_markers:
                self.frame_markers[id_].vertices = optical_frame(pose).vertices
                self.vis.update_geometry(self.frame_markers[id_])
                new_text = text_3d(id_, pose[:3, 3] + np.array([0.1, 0.1, 0.1]), self.font, self._camera_view_pose())
                self.text[id_].points = new_text.points
                self.vis.update_geometry(self.text[id_])

    def set_start_view(self, zoom=4):
        """Sets the direction of the camera/viewport"""
        view_ctl = self.vis.get_view_control()
        view_ctl.set_up([1, 0, 0])
        view_ctl.set_front([0, 0, -1])
        view_ctl.set_lookat([1, 0, 0])
        view_ctl.set_zoom(zoom)

    def _update_pointcloud_marker(self):
        """Updates the markers for the point clouds. Should be called from visualiser.run.
        Should be called after the markers have been added.
        """

        for id_, points in self.pointclouds.items():
            if id_ in self.pointcloud_markers:
                self.pointcloud_markers[id_][0].points = o3d_pointcloud(self.pointclouds[id_]).points
                self.pointcloud_markers[id_][0].paint_uniform_color(self.pointcloud_markers[id_][1])
                self.vis.update_geometry(self.pointcloud_markers[id_][0])

    def _camera_view_pose(self):
        return self.vis.get_view_control().convert_to_pinhole_camera_parameters().extrinsic

    def run(self):
        """Start the visualiser"""
        while self.running:
            self._update_frame_markers()
            self._update_pointcloud_marker()
            self.vis.update_renderer()
            self.vis.poll_events()
            time.sleep(0.05)
        self.vis.destroy_window()

    def stop(self):
        """Stop the visualiser"""
        self.running = False


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


if __name__ == "__main__":
    main()

