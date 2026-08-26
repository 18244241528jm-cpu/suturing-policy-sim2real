# Headless FoundationPose demo: same pose estimation as upstream run_demo.py, but it
# writes the pose overlay to disk instead of calling cv2.imshow(), which throws inside
# the FoundationPose Docker image and anywhere else without an attached display.
#
# Structure follows upstream run_demo.py (NVlabs/FoundationPose, commit a1b694b, NVIDIA
# Source Code License); the estimator, iteration counts and reader are all upstream. Copy
# this file next to the upstream code, since it imports estimater and datareader.
#
# See README.md in this directory for the scene layout and the install check.
import os, argparse, logging, numpy as np, trimesh, imageio, cv2
from estimater import *
from datareader import *

if __name__ == "__main__":
    code_dir = os.path.dirname(os.path.realpath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh_file", default=f"{code_dir}/demo_data/mustard0/mesh/textured_simple.obj")
    ap.add_argument("--test_scene_dir", default=f"{code_dir}/demo_data/mustard0")
    ap.add_argument("--est_refine_iter", type=int, default=5)
    ap.add_argument("--track_refine_iter", type=int, default=2)
    ap.add_argument("--max_frames", type=int, default=40)   # limit for a quick visual
    ap.add_argument("--out_dir", default=f"{code_dir}/debug")
    args = ap.parse_args()

    set_logging_format(); set_seed(0)
    os.makedirs(f"{args.out_dir}/track_vis", exist_ok=True)
    os.makedirs(f"{args.out_dir}/ob_in_cam", exist_ok=True)

    mesh = trimesh.load(args.mesh_file)
    to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
    bbox = np.stack([-extents/2, extents/2], axis=0).reshape(2,3)

    scorer = ScorePredictor(); refiner = PoseRefinePredictor()
    glctx = dr.RasterizeCudaContext()
    est = FoundationPose(model_pts=mesh.vertices, model_normals=mesh.vertex_normals,
                         mesh=mesh, scorer=scorer, refiner=refiner,
                         debug_dir=args.out_dir, debug=0, glctx=glctx)
    reader = YcbineoatReader(video_dir=args.test_scene_dir, shorter_side=None, zfar=np.inf)

    n = min(args.max_frames, len(reader.color_files))
    for i in range(n):
        color = reader.get_color(i); depth = reader.get_depth(i)
        if i == 0:
            mask = reader.get_mask(0).astype(bool)
            pose = est.register(K=reader.K, rgb=color, depth=depth, ob_mask=mask, iteration=args.est_refine_iter)
        else:
            pose = est.track_one(rgb=color, depth=depth, K=reader.K, iteration=args.track_refine_iter)
        np.savetxt(f"{args.out_dir}/ob_in_cam/{reader.id_strs[i]}.txt", pose.reshape(4,4))
        center_pose = pose @ np.linalg.inv(to_origin)
        vis = draw_posed_3d_box(reader.K, img=color, ob_in_cam=center_pose, bbox=bbox)
        vis = draw_xyz_axis(color, ob_in_cam=center_pose, scale=0.1, K=reader.K, thickness=3, transparency=0, is_input_rgb=True)
        imageio.imwrite(f"{args.out_dir}/track_vis/{reader.id_strs[i]}.png", vis)
        logging.info(f"frame {i}: saved overlay")
    print(f"DONE: wrote {n} overlays to {args.out_dir}/track_vis")
