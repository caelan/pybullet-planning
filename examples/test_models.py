
import os
from pybullet_tools.utils import create_mesh, set_point, read_pcd_file, disconnect, \
    wait_for_user, mesh_from_points, mesh_from_body, get_links, get_num_links, connect

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

DATA_DIRECTORY = "/Users/caelan/Programs/LIS/git/lis-data"

def main():
    world = connect(use_gui=True)

    #model = 'oil'
    model = 'soup'

    point_path = os.path.join(DATA_DIRECTORY, 'clouds', CLOUDS[model])
    mesh_path = os.path.join(DATA_DIRECTORY, 'meshes', MESHES[model]) # off | ply | wrl
    #ycb_path = os.path.join(DATA_DIRECTORY, 'ycb', 'plastic_wine_cup', 'meshes', 'tsdf.stl') # stl | ply
    ycb_path = os.path.join(DATA_DIRECTORY, 'ycb', 'plastic_wine_cup',
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
    wait_for_user()
    disconnect()

if __name__ == '__main__':
    main()
