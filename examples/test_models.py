import os
import pybullet as p

from pybullet_tools.utils import create_mesh, set_point, read_pcd_file, disconnect, \
    wait_if_gui, mesh_from_points, get_links, get_num_links, connect, load_pybullet, \
    create_obj, WHITE, get_client, NULL_ID, set_color, get_all_links

SODA_CLOUD = 'soda.pcd'

CLOUDS = {
  'oil': 'oilBottle.pcd',
  'soup': 'soup16.pcd',
  'block': SODA_CLOUD,
  'red': SODA_CLOUD,
  'orange': SODA_CLOUD,
  'green': SODA_CLOUD,
  'blue': SODA_CLOUD,
  'purple': SODA_CLOUD,
}

SODA_MESH = 'soda.off'

MESHES = { # Cap is literally the bottle cap
    'oil': 'oil_bottle_simple_new.off',
    # oil_bottle_simple_cap, oil_bottle_simple_new, oilBottleWithCapNew, oilBottleWithCap, ...
    'soup': 'soup16.off', #
    'block': SODA_MESH,
    # soda.off, soda_original.off, soda_points.off
    'red': SODA_MESH,
    'orange': SODA_MESH,
    'green': SODA_MESH,
    'blue': SODA_MESH,
    'purple': SODA_MESH,
}

LIS_DIRECTORY = "/Users/caelan/Programs/LIS/git/lis-data/"

def test_lis(world):
    #model = 'oil'
    model = 'soup'

    point_path = os.path.join(LIS_DIRECTORY, 'clouds', CLOUDS[model])
    mesh_path = os.path.join(LIS_DIRECTORY, 'meshes', MESHES[model]) # off | ply | wrl
    #ycb_path = os.path.join(DATA_DIRECTORY, 'ycb', 'plastic_wine_cup', 'meshes', 'tsdf.stl') # stl | ply
    ycb_path = os.path.join(LIS_DIRECTORY, 'ycb', 'plastic_wine_cup',
                            'textured_meshes', 'optimized_tsdf_texture_mapped_mesh.obj') # ply

    print(point_path)
    print(mesh_path)
    print(ycb_path)

    #mesh = read_mesh_off(mesh_path, scale=0.001)
    #print(mesh)
    points = read_pcd_file(point_path)
    #print(points)
    #print(convex_hull(points))
    mesh = mesh_from_points(points)
    #print(mesh)

    body = create_mesh(mesh, color=(1, 0, 0, 1))
    #print(get_num_links(body))
    #print(mesh_from_body(body))

    #set_point(body, (1, 1, 1))
    wait_if_gui()

#####################################

YCB_DIRECTORY = "/Users/caelan/Programs/ycb_benchmarks/processed_data/"

YCB_TEMPLATE = '{:03d}_{}'
POISSON = 'poisson/'
TSDF = 'tsdf/'
OBJ_PATH = 'textured.obj'
PNG_PATH = 'textured.png'

SENSOR = POISSON # POISSON | TSDF

def get_ycb_objects():
    return {name.split('_', 1)[-1]: name for name in os.listdir(YCB_DIRECTORY)
            if os.path.isdir(os.path.join(YCB_DIRECTORY, name))}

def set_texture(body, image_path):
    #if color is not None:
    #    set_color(body, color)
    assert image_path.endswith('.png')
    texture = p.loadTexture(image_path)
    assert 0 <= texture
    p.changeVisualShape(body, linkIndex=NULL_ID, shapeIndex=NULL_ID, #rgbaColor=WHITE,
                        textureUniqueId=texture, #specularColor=(0, 0, 0),
                        physicsClientId=get_client())
    return texture

def test_ycb(world):
    name = 'mustard_bottle' # potted_meat_can | cracker_box | sugar_box | mustard_bottle | bowl
    path_from_name = get_ycb_objects()
    print(path_from_name)

    ycb_directory = os.path.join(YCB_DIRECTORY, path_from_name[name], SENSOR)
    obj_path = os.path.join(ycb_directory, OBJ_PATH)
    image_path = os.path.join(ycb_directory, PNG_PATH)
    print(obj_path)
    #body = load_pybullet(obj_path) #, fixed_base=True)
    body = create_obj(obj_path, color=WHITE)
    set_texture(body, image_path)

    for link in get_all_links(body):
        set_color(body, link=link, color=WHITE)
    wait_if_gui()

#####################################

def main():
    world = connect(use_gui=True)
    #test_lis(world)
    test_ycb(world)
    disconnect()

if __name__ == '__main__':
    main()
