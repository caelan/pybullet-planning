import os
from PIL import Image
import argparse
from glob import glob
from tqdm import tqdm
import open3d as o3d
import trimesh
import numpy as np


def duplicate_vertices_per_face(mesh):
    """Convert mesh to have per-face-unique vertices (flat shading)."""
    triangles = np.asarray(mesh.triangles)
    vertices = np.asarray(mesh.vertices)
    vertex_colors = np.asarray(mesh.vertex_colors)

    new_vertices = []
    new_vertex_colors = []
    new_triangles = []

    for i, tri in enumerate(triangles):
        for j in range(3):
            vid = tri[j]
            new_vertices.append(vertices[vid])
            new_vertex_colors.append(vertex_colors[vid])
        new_triangles.append([3*i, 3*i+1, 3*i+2])

    flat_mesh = o3d.geometry.TriangleMesh()
    flat_mesh.vertices = o3d.utility.Vector3dVector(np.array(new_vertices))
    flat_mesh.triangles = o3d.utility.Vector3iVector(np.array(new_triangles))
    flat_mesh.vertex_colors = o3d.utility.Vector3dVector(np.array(new_vertex_colors))
    flat_mesh.compute_vertex_normals()
    return flat_mesh

def bake_color_texture(mesh, texture_size=1024):
    """Bake per-vertex colors to a UV texture map and assign UVs to mesh."""
    # UV unwrap using Open3D's default method (XAtlas)
    mesh.compute_triangle_normals()
    mesh = duplicate_vertices_per_face(mesh)
    uvs = mesh.compute_uv_map()

    uv_coords = np.asarray(mesh.triangle_uvs)
    colors = np.asarray(mesh.vertex_colors)
    tris = np.asarray(mesh.triangles)

    tex = np.ones((texture_size, texture_size, 3), dtype=np.uint8) * 255

    for i in range(len(tris)):
        uv = uv_coords[i * 3:i * 3 + 3]
        color = (colors[tris[i]] * 255).astype(np.uint8).mean(axis=0)

        px = (uv[:, 0] * (texture_size - 1)).astype(int)
        py = ((1 - uv[:, 1]) * (texture_size - 1)).astype(int)

        rr, cc = [py[0], py[1], py[2]], [px[0], px[1], px[2]]
        ImageDraw = Image.fromarray(tex)
        draw = ImageDraw.load()
        for y in range(min(rr), max(rr)):
            for x in range(min(cc), max(cc)):
                draw[x, y] = tuple(color)

    tex_image = Image.fromarray(tex)
    return mesh, tex_image

def save_mesh_with_texture(mesh, tex_image, out_basename):
    """Save the mesh as .obj/.mtl/.png with UVs and baked texture."""
    obj_path = f"{out_basename}.obj"
    mtl_path = f"{out_basename}.mtl"
    tex_path = f"{out_basename}.png"

    o3d.io.write_triangle_mesh(obj_path, mesh, write_triangle_uvs=True)
    tex_image.save(tex_path)

    # Overwrite or write .mtl file
    with open(mtl_path, "w") as f:
        f.write("newmtl material_0\n")
        f.write(f"map_Kd {os.path.basename(tex_path)}\n")

    # Patch .obj to reference the .mtl and use material
    with open(obj_path, "r") as f:
        lines = f.readlines()

    with open(obj_path, "w") as f:
        f.write(f"mtllib {os.path.basename(mtl_path)}\nusemtl material_0\n")
        for line in lines:
            f.write(line)

    print(f"✅ Exported: {obj_path}, {mtl_path}, {tex_path}")


def load_obj_with_face_color(obj_path):
    mesh = trimesh.load(obj_path, process=False, maintain_order=True, force='mesh')
    assert mesh.visual.kind == 'face', "Expected per-face material color from MTL"
    
    # Convert to Open3D triangle mesh
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces)
    face_colors = np.asarray(mesh.visual.face_colors[:, :3]) / 255.0  # RGB

    # Duplicate vertices so each face has its own
    new_vertices = []
    new_triangles = []
    new_colors = []

    for i, tri in enumerate(faces):
        color = face_colors[i]
        idx_start = len(new_vertices)
        for vi in tri:
            new_vertices.append(vertices[vi])
            new_colors.append(color)
        new_triangles.append([idx_start, idx_start+1, idx_start+2])

    o3d_mesh = o3d.geometry.TriangleMesh()
    o3d_mesh.vertices = o3d.utility.Vector3dVector(np.array(new_vertices))
    o3d_mesh.triangles = o3d.utility.Vector3iVector(np.array(new_triangles))
    o3d_mesh.vertex_colors = o3d.utility.Vector3dVector(np.array(new_colors))
    return o3d_mesh


if __name__ == "__main__":
    input_obj = "/home/twjhlee/Downloads/종심_tosend/debug/mesh_03.obj"
    basename = "baked_mesh"

    mesh = load_obj_with_face_color(input_obj)
    o3d.visualization.draw_geometries([mesh])
    if not mesh.has_vertex_colors():
        raise ValueError("Mesh must have per-vertex colors")

    mesh_with_uvs, tex_image = bake_color_texture(mesh, texture_size=1024)
    save_mesh_with_texture(mesh_with_uvs, tex_image, basename)