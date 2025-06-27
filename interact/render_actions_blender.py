import bpy
import numpy as np
import os
import sys
from mathutils import Matrix
import json
import argparse
import importlib.util


def load_py_config(file_path):
    spec = importlib.util.spec_from_file_location("config", file_path)
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    return config

# === Parameters ===
# arg parse
parser = argparse.ArgumentParser()
parser.add_argument("--expdir", type=str, help="Path to directory in LOGS")
args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:])

# load the mesh_paths(probably urdf)
with open(os.path.join(args.expdir, "mesh_pathes.json"), 'r') as f:
    mesh_pathes = json.load(f)
# load actions
actions = np.load(os.path.join(args.expdir, "actions.npy"))
# action generator args
with open(os.path.join(args.expdir, "args.json"), 'r') as f:
    action_args = json.load(f)
# render poses
camera_poses = np.load(os.path.join(args.expdir, "render_poses.npy"))
# center of workspace
center = np.load(os.path.join(args.expdir, "center_of_bbox.npy"))

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
fx = data["fl_x"] // 3
fy = data["fl_y"] // 3
cx = data["cx"]
cy = data["cy"]
image_resolution = (width, height)

# === Clear existing objects ===
bpy.ops.wm.read_factory_settings(use_empty=True)

# === Import OBJ ===
for mesh_path in mesh_pathes:
    # debug
    bpy.ops.import_scene.obj(filepath=mesh_path.replace(".urdf", ".obj"))

# Apply rotation to all imported mesh objects
for obj in bpy.context.scene.objects:
    if obj.type == 'MESH':
        # Set rotation in radians (X, Y, Z)
        obj.rotation_euler = (0, 0, 0)

# === Set render settings ===
scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
scene.render.image_settings.file_format = 'PNG'
scene.render.resolution_x = image_resolution[0] // 2
scene.render.resolution_y = image_resolution[1] // 2
scene.render.film_transparent = True
bpy.context.scene.eevee.taa_render_samples = 1  # default is 64
scene = bpy.context.scene
# Disable shadows, AO, bloom, etc.
# scene.eevee.use_soft_shadows = False
# scene.eevee.use_gtao = False
# scene.eevee.use_bloom = False
# scene.eevee.use_ssr = False
# # Optional: disable motion blur
# scene.render.use_motion_blur = False

# === Add camera ===
cam_data = bpy.data.cameras.new(name='Camera')
cam_obj = bpy.data.objects.new('Camera', cam_data)
scene.collection.objects.link(cam_obj)
scene.camera = cam_obj

# Set intrinsics
cam_data.type = 'PERSP'
cam_data.lens_unit = 'FOV'
cam_data.angle = 2 * np.arctan(image_resolution[0] / (2 * fx))  # horizontal FOV

# === Add light for visibility (optional) ===
light_data = bpy.data.lights.new(name='light', type='POINT')
light_obj = bpy.data.objects.new(name='light', object_data=light_data)
light_obj.location = center + np.array([0, 0, 0.5])
scene.collection.objects.link(light_obj)

# === Function to convert a 4x4 np.array to Blender Matrix ===
def np_to_matrix4x4(mat):
    return Matrix([list(row) for row in mat])

mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
# === Render each camera pose ===
for j, action in enumerate(actions):
    output_dir = os.path.join(args.expdir, "rendered/{:03d}".format(j))
    os.makedirs(output_dir, exist_ok=True)
    mesh_objects[int(action[0])].location = (action[1], action[2], 0)
    for i, extr in enumerate(camera_poses):
        # Blender uses camera-to-world, so invert the world-to-camera
        cam_to_world = np.linalg.inv(extr)
        cam_obj.matrix_world = np_to_matrix4x4(cam_to_world)

        scene.render.filepath = f"{output_dir}/render_{i:03d}.png"
        bpy.ops.render.render(write_still=True)