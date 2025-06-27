#!/usr/bin/env python3
""""Code to obtain the best action"""
import numpy as np
import os
import sys
import json
import argparse
import importlib.util
import cv2
import matplotlib.pyplot as plt
from tqdm import tqdm
from glob import glob


if __name__ == "__main__":
    # arg parser
    parser = argparse.ArgumentParser()
    parser.add_argument("expdir", type=str, help="Path to directory in LOGS")
    args = parser.parse_args()
    # get all actions
    actions = np.load(os.path.join(args.expdir, "actions.npy"))
    # get the rendered results
    all_render_dirs = sorted(glob(os.path.join(args.expdir, "rendered/*")))
    scores = []
    for render_dir in tqdm(all_render_dirs):
        all_img_names = sorted(glob(os.path.join(render_dir, "*.png")))
        # load the imgs
        all_imgs = []
        for img_name in all_img_names:
            all_imgs.append(cv2.imread(img_name))
        vis = np.concatenate(all_imgs, axis=1)
        vis = vis[..., -1]
        score = vis.sum()
        scores.append(score)
    scores_np = np.array(scores)
    print("Max action: {}".format(all_render_dirs[np.argmax(scores_np)]))
    np.savetxt(os.path.join(args.expdir, "best_action.json"), actions[np.argmax(scores_np)])