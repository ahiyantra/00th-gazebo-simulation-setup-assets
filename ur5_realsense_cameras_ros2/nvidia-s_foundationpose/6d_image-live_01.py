# "6d_image-live_01[dot]py"

from estimater import *
from datareader_online import *
import argparse
import pyrealsense2 as rs
import time
from ultralytics import YOLO
import numpy as np
import cv2
import datetime
import trimesh
import logging
import os
import imageio

def rotation_matrix_to_euler_angles(R):
    """
    Convert a rotation matrix to Euler angles (roll, pitch, yaw).
    """
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    code_dir = os.path.dirname(os.path.realpath(__file__))

    parser.add_argument('--yolo_model_path', type=str, default=f'{code_dir}/demo_data/bottle0/segmentation_weights/May_best.pt', help="Path to the Yolo segmentation weights file")
    parser.add_argument('--mesh_file', type=str, default=f'{code_dir}/demo_data/bottle0/mesh/bottle0001.ply', help="Path to the mesh file")
    parser.add_argument('--est_refine_iter', type=int, default=5, help="Number of refinement iterations for pose estimation")
    parser.add_argument('--track_refine_iter', type=int, default=2, help="Number of refinement iterations for tracking")
    parser.add_argument('--debug', type=int, default=1, help="Debug level")
    parser.add_argument('--debug_dir', type=str, default=f'{code_dir}/debug', help="Directory to save debug information")
    args = parser.parse_args()

    # Set logging format and seed for reproducibility
    set_logging_format()
    set_seed(0)

    # Load the mesh file
    mesh = trimesh.load(args.mesh_file)

    debug = args.debug
    debug_dir = args.debug_dir
    os.system(f'rm -rf {debug_dir}/* && mkdir -p {debug_dir}/track_vis {debug_dir}/ob_in_cam')

    # Calculate oriented bounds of the mesh
    to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
    bbox = np.stack([-extents / 2, extents / 2], axis=0).reshape(2, 3)

    # Initialize the ScorePredictor and PoseRefinePredictor
    scorer = ScorePredictor()
    refiner = PoseRefinePredictor()
    glctx = dr.RasterizeCudaContext()

    # Load the YOLO model using the provided model path
    model = YOLO('yolov8n.pt')  # Load an official model before the custom model
    model = YOLO(args.yolo_model_path)  # Load the custom model

    # Initialize the pose estimator
    est = FoundationPose(model_pts=mesh.vertices, model_normals=mesh.vertex_normals, mesh=mesh, scorer=scorer, refiner=refiner, debug_dir=debug_dir, debug=debug, glctx=glctx)
    logging.info("Estimator initialization done.")

    # Initialize the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)
    start_time = time.time()

    try:
        while True:
            frame_start_time = time.time()

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = frames.get_color_frame()
            aligned_depth_frame = aligned_frames.get_depth_frame()

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())[..., :3]
            depth_image = np.asanyarray(aligned_depth_frame.get_data())

            # Convert depth to meters and resize
            depth_image = depth_image / 1000.0  # Convert depth to meters
            depth_image = cv2.resize(depth_image, (640, 480), interpolation=cv2.INTER_NEAREST)
            depth_image[(depth_image < 0.1) | (depth_image > np.inf)] = 0

            color = cv2.resize(color_image, (640, 480), interpolation=cv2.INTER_NEAREST)

            # Perform pose estimation
            results = model(color_image)
            for result in results:
                if result.masks is not None:
                    mask_raw = result.masks[0].cpu().data.numpy().transpose(1, 2, 0)
                    for mask in result.masks[2:]:
                        mask_raw += mask.cpu().data.numpy().transpose(1, 2, 0)

                    mask_scaled = (mask_raw * 255).astype(np.uint8)
                    if mask_scaled is not None:
                        mask = np.array(mask_scaled).astype(bool)
                        pose = est.register(K=None, rgb=color_image, depth=depth_image, ob_mask=mask, iteration=args.est_refine_iter)
                    else:
                        raise ValueError("No valid mask found in the results.")

            if debug >= 3:
                m = mesh.copy()
                m.apply_transform(pose)
                m.export(f'{debug_dir}/model_tf.obj')
                xyz_map = depth2xyzmap(depth_image, None)
                valid = depth_image >= 0.1
                pcd = toOpen3dCloud(xyz_map[valid], color[valid])
                o3d.io.write_point_cloud(f'{debug_dir}/scene_complete.ply', pcd)
            else:
                pose = est.track_one(rgb=color, depth=depth_image, K=None, iteration=args.track_refine_iter)

            os.makedirs(f'{debug_dir}/ob_in_cam', exist_ok=True)

            if debug >= 1:
                center_pose = pose @ np.linalg.inv(to_origin)
                # Draw bounding box and pose axes
                vis = draw_posed_3d_box(None, img=color, ob_in_cam=center_pose, bbox=bbox)
                vis = draw_xyz_axis(color, ob_in_cam=center_pose, scale=0.1, K=None, thickness=3, transparency=0, is_input_rgb=True)

                # Add 6D pose text to the visualization
                fps_text = f"FPS: {1.0 / (time.time() - frame_start_time):.2f}"
                position = pose[:3, 3]
                rotation = pose[:3, :3]
                roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation)
                pose_text = f"Position: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}"
                orientation_text = f"Roll: {np.degrees(roll):.2f}, Pitch: {np.degrees(pitch):.2f}, Yaw: {np.degrees(yaw):.2f}"

                # Overlay the text on the visualization image
                cv2.putText(vis, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(vis, pose_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(vis, orientation_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                cv2.imshow('1', vis)

                # Check for any key press to end the loop
                if cv2.waitKey(1) & 0xFF != 255:  # Any key press
                    break

            if debug >= 2:
                os.makedirs(f'{debug_dir}/track_vis', exist_ok=True)
                imageio.imwrite(f'{debug_dir}/track_vis/{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.png', vis)

            frame_end_time = time.time()
            frame_processing_time = frame_end_time - frame_start_time

            # Calculate FPS
            fps = 1.0 / frame_processing_time

    finally:
        # Stop the pipeline and close all OpenCV windows
        pipeline.stop()
        cv2.destroyAllWindows()
