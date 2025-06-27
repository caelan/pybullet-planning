import os
from PIL import Image
import argparse
from glob import glob
from tqdm import tqdm

def generate_color_texture(rgb, filename, out_dir):
    """Create a 1x1 PNG of the given RGB color (0–1 range)."""
    rgb_255 = tuple(int(round(c * 255)) for c in rgb)
    img = Image.new("RGB", (1, 1), rgb_255)
    img.save(os.path.join(filename))

def convert_mtl_with_textures(mtl_path):
    with open(mtl_path, 'r') as f:
        lines = f.readlines()

    output_lines = []
    out_dir = os.path.dirname(mtl_path)
    png_dir = mtl_path.replace(".mtl", "")
    os.makedirs(png_dir, exist_ok=True)
    current_material = None

    for i in range(len(lines)):
        line = lines[i].strip()

        if line.startswith("newmtl"):
            current_material = line.split()[1]
            output_lines.append(line)

        elif line.startswith("Kd") and current_material:
            # Parse RGB values from Kd line
            parts = line.split()
            r, g, b = map(float, parts[1:4])
            # texture_filename = f"{current_material}.png"
            texture_filename = os.path.join(png_dir, "{}.png".format(current_material))
            written_filename = os.path.join(png_dir.split('/')[-1], "{}.png".format(current_material))

            # Generate texture
            generate_color_texture((r, g, b), texture_filename, out_dir)

            # Replace Kd line with map_Kd
            output_lines.append(f"map_Kd {written_filename}")

        else:
            output_lines.append(line)

    # Write modified MTL
    with open(mtl_path, 'w') as f:
        f.write("\n".join(output_lines))


if __name__ == "__main__":
    # get all mtls within path
    parser = argparse.ArgumentParser()
    parser.add_argument('path', type=str, help="path to cluster dir with .mtl files")
    args = parser.parse_args()

    # get all mtl files
    all_mtl_files = sorted(glob(os.path.join(args.path, "*.mtl")))
    for mtl_file in tqdm(all_mtl_files):
        if "material.mtl" in mtl_file:
            continue
        convert_mtl_with_textures(mtl_file)