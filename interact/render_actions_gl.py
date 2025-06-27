import numpy as np
import trimesh
import pyrender
import os
import json
import imageio
import argparse
import importlib.util
from tqdm import tqdm


def load_py_config(file_path):
    spec = importlib.util.spec_from_file_location("config", file_path)

    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    return config


def look_at_y_up(A, B, up=np.array([0, 1, 0])):
    # Forward vector (camera - looks from A to B)
    forward = B - A
    forward /= np.linalg.norm(forward)

    # Right vector (X-axis)
    right = np.cross(up, forward)
    right /= np.linalg.norm(right)

    # Recompute true up vector (Y-axis)
    up_corrected = np.cross(forward, right)
    up_corrected /= np.linalg.norm(up_corrected)

    # Create rotation matrix from right, up, forward
    R = np.column_stack([right, up_corrected, forward])  # 3x3

    # Create 4x4 matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = A  # position

    return T


# === Load Data ===
parser = argparse.ArgumentParser()
parser.add_argument('expdir', type=str)
parser.add_argument('--save_all_imgs', action='store_true')
args = parser.parse_args()

expdir = args.expdir
mesh_pathes = json.load(open(os.path.join(expdir, "mesh_pathes.json")))
actions = np.load(os.path.join(expdir, "actions.npy"))
camera_poses = np.load(os.path.join(expdir, "render_poses.npy"))
center = np.load(os.path.join(expdir, "center_of_bbox.npy"))
with open(os.path.join(args.expdir, "args.json"), 'r') as f:
    action_args = json.load(f)
# get the intrinsic parameters
# load the datapath
config_path = os.path.join(os.path.dirname(action_args["path"]), "config.py")
config = load_py_config(config_path)
datadir = config.data['datadir']
json_file = os.path.join(datadir, "transforms_train.json")
with open(json_file, "r") as f:
    data = json.load(f)
# Access individual values
width = int(data["w"])
height = int(data["h"])
fx = data["fl_x"] // 10
fy = data["fl_y"] // 10
cx = data["cx"]
cy = data["cy"]
image_resolution = (width // 4, height // 4)

# === Load Meshes ===
meshes = []
for path in mesh_pathes:
    loaded = trimesh.load(path.replace(".urdf", ".obj"))
    if isinstance(loaded, trimesh.Scene):
        mesh = loaded.dump(concatenate=True)  # merge into one Trimesh
    else:
        mesh = loaded
    meshes.append(mesh)
pyrender_meshes = []
for mesh in meshes:
    pyrender_meshes.append(pyrender.Mesh.from_trimesh(mesh))

# === Setup Renderer ===
renderer = pyrender.OffscreenRenderer(viewport_width=image_resolution[0],
                                      viewport_height=image_resolution[1])

# === Render Loop ===
scores_per_action = np.zeros(len(actions))
for j, action in tqdm(enumerate(actions)):
    all_imgs = []
    if args.save_all_imgs:
        output_dir = os.path.join(expdir, f"rendered_gl/{j:03d}")
        os.makedirs(output_dir, exist_ok=True)

    mesh_index = int(action[0])
    translation = np.array([action[1], action[2], 0.0])
    
    mesh = meshes[mesh_index].copy()
    mesh.apply_translation(translation)
    mesh_tm = pyrender.Mesh.from_trimesh(mesh)
    # Create scene
    scene = pyrender.Scene(bg_color=[0.0, 0.0, 0.0, 0.0], ambient_light=[0.5, 0.5, 0.5])
    camera = pyrender.IntrinsicsCamera(fx=fx, fy=fy, cx=image_resolution[0] // 2, cy=image_resolution[1] // 2)
    cam_node = scene.add(camera, pose=np.eye(4))
    scene.add(mesh_tm)
    for idx in range(len(meshes)):
        if not idx == mesh_index:
            scene.add(pyrender_meshes[idx])

    for i, extr in enumerate(camera_poses):
        # Convert world-to-camera to camera-to-world
        cam_pose = np.linalg.inv(extr)
        scene.set_pose(cam_node, pose=cam_pose)
        # Create camera
        # debug using visualizer
        # pyrender.Viewer(scene, use_raymond_lighting=True, run_in_thread=False)

        color, _ = renderer.render(scene)
        all_imgs.append(color)
        if args.save_all_imgs:
            imageio.imwrite(os.path.join(output_dir, f"render_{i:03d}.png"), color)
    vis = np.concatenate(all_imgs, axis=1)
    vis = vis[..., 0]
    score = vis.sum()
    scores_per_action[j] = score

renderer.delete()

# save the max value action
renderer = pyrender.OffscreenRenderer(viewport_width=image_resolution[0] * 4,
                                      viewport_height=image_resolution[1] * 4)
np.savetxt(os.path.join(args.expdir, "best_action.json"), actions[np.argmax(scores_per_action)])
# let's visualize the best value action
output_dir = os.path.join(expdir, "best_action")
os.makedirs(output_dir, exist_ok=True)
action = actions[np.argmax(scores_per_action)]
mesh_index = int(action[0])
translation = np.array([action[1], action[2], 0.0])

mesh = meshes[mesh_index].copy()
mesh.apply_translation(translation)
mesh_tm = pyrender.Mesh.from_trimesh(mesh)
# Create scene
radius = 2.0                       # Radius of light sphere
num_lights = 1024                   # Number of lights to distribute
scene = pyrender.Scene(bg_color=[0.0, 0.0, 0.0, 0.0], ambient_light=[1.0, 1.0, 1.0])
camera = pyrender.IntrinsicsCamera(fx=fx * 2, fy=fy * 2, cx=image_resolution[0] * 2 , cy=image_resolution[1] * 2)
cam_node = scene.add(camera, pose=np.eye(4))

scene.add(mesh_tm)
for idx in range(len(meshes)):
    if not idx == mesh_index:
        scene.add(pyrender_meshes[idx])
# set up the lights
# Generate spherical coordinates for light positions
phi = np.linspace(0, np.pi, int(np.sqrt(num_lights)))         # elevation
theta = np.linspace(0, 2 * np.pi, int(np.sqrt(num_lights)))   # azimuth
phi, theta = np.meshgrid(phi, theta)

# Convert spherical to Cartesian
x = center[0] + radius * np.sin(phi) * np.cos(theta)
y = center[1] + radius * np.sin(phi) * np.sin(theta)
z = center[2] + radius * np.cos(phi)

positions = np.stack([x, y, z], axis=-1).reshape(-1, 3)

# Add lights at each position
for pos in positions:
    light = pyrender.PointLight(color=np.ones(3), intensity=10.0)
    T = np.eye(4)
    T[:3, 3] = pos
    scene.add(light, pose=T)

for i, extr in enumerate(camera_poses):
    # Convert world-to-camera to camera-to-world
    cam_pose = np.linalg.inv(extr)
    scene.set_pose(cam_node, pose=cam_pose)
    # debug using visualizer
    # pyrender.Viewer(scene, use_raymond_lighting=True, run_in_thread=False)

    color, _ = renderer.render(scene)
    imageio.imwrite(os.path.join(output_dir, f"render_{i:03d}.png"), color)

renderer.delete()
