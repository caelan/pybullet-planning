import os
from glob import glob
import argparse
# import open3d as o3d
import numpy as np
# import mmcv
import pickle
from treelib import Node, Tree
# import graphviz
# import mcubes
import shutil
# import matplotlib
import json

class Node(object):
    def __init__(self, key):
        # key is the cluster idx
        self.key = key
        self.children = []
        self.parent = None

    def return_children_keys(self):
        children_idx = []
        for child in self.children:
            children_idx.append(child.key)
        return children_idx
 

class Trie(object):
    def __init__(self, grid_idx_per_cluster, logdir, size_thresh):
        self.head = Node("Head")
        self.cluster_dir = os.path.join(logdir, "clusters")
        self.grid_idx_per_cluster = grid_idx_per_cluster
        self.key_dict = {}
        self.size_thresh = size_thresh
   
    def insert(self, key):
        current_node = self.head
        # check inclusion of grid idxs to build trie
        # always add first key
        found_place = False
        while not found_place:
            if len(current_node.children) == 0:
                new_node = Node(key)
                self.key_dict[key] = new_node
                current_node.children.append(new_node)
                new_node.parent = current_node
                found_place = True
            else:
                total_included = False
                for child in current_node.children:
                    is_included, ratio_child = self.check_inclusion(key, child)
                    if is_included:
                        total_included = True
                        if ratio_child < 0.9:
                            current_node = child
                        else:
                            # total included case
                            fname = os.path.join(self.cluster_dir, "cluster_{:02d}.ply".format(key))
                            os.remove(fname)
                            fname = os.path.join(self.cluster_dir, "mesh_{:02d}.obj".format(key))
                            os.remove(fname)
                            found_place = True
                            break
                if not total_included:
                    # check if totally in parent node
                    new_node = Node(key)
                    self.key_dict[key] = new_node
                    current_node.children.append(new_node)
                    new_node.parent = current_node
                    found_place = True
    
    def prune(self):
        """Function to delete children node with no siblings"""
        original_keys = list(self.key_dict.keys())
        for key in original_keys:
            # current node
            cur_node = self.key_dict[key]
            # parent node
            parent_node = cur_node.parent
            # get number of siblings
            num_sibling = len(parent_node.children)
            if num_sibling == 1:
                # let's delete this node
                parent_node.children = []
                cur_node = None 
                del self.key_dict[key]
                fname = os.path.join(self.cluster_dir, "cluster_{:02d}.ply".format(key))
                os.remove(fname)
                fname = os.path.join(self.cluster_dir, "mesh_{:02d}.obj".format(key))
                os.remove(fname)

    def complete(self, grid_idx_per_cluster, voxel_size, xyz_min_fine, size_thresh, cluster_dir, grid_dim, interact_dir):
        """Function to make sum of children == parent"""
        original_keys = list(self.key_dict.keys())
        taken_keys = original_keys
        for key in original_keys:
            # current node
            cur_node = self.key_dict[key]
            cur_grid = grid_idx_per_cluster[cur_node.key]
            # children nodes
            children_grids = []
            for child_node in cur_node.children:
                children_grids.append(grid_idx_per_cluster[child_node.key])
            if len(children_grids) == 0:
                continue
            # total children grid
            total_children_grid = np.concatenate(children_grids, axis=0) # N x 3
            # get unique parts
            cur_grid_view = cur_grid.view([('', cur_grid.dtype)] * cur_grid.shape[1])
            child_grid_view = total_children_grid.view([('', total_children_grid.dtype)] * total_children_grid.shape[1])
            unique_set = np.setdiff1d(cur_grid_view, child_grid_view).view(cur_grid.dtype).reshape(-1, 3)
            if len(unique_set) > size_thresh:
                # create a new node
                new_key = 0
                while new_key in taken_keys:
                    new_key += 1
                new_node = Node(new_key)
                self.key_dict[new_key] = new_node
                cur_node.children.append(new_node)
                new_node.parent = cur_node
                taken_keys.append(new_key)
                # to real world coordinates
                real_points = grididx2realpos(unique_set, voxel_size, xyz_min_fine)
                # save the geometry
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(real_points)
                cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                o3d.io.write_point_cloud(os.path.join(cluster_dir, "cluster_{:02d}.ply".format(new_key)), cl)
                # marching cubes and write the mesh
                grid = np.zeros(grid_dim, dtype=bool)
                grid[unique_set[:, 0], unique_set[:, 1], unique_set[:, 2]] = True
                vertices, triangles = mcubes.marching_cubes(grid, 0.5)
                vertices = vertices * voxel_size + xyz_min_fine
                # let's save the meshes in robot coordinates
                s = np.loadtxt(os.path.join(interact_dir, "scale.txt"))
                trans = np.loadtxt(os.path.join(interact_dir, "translation.txt"))
                rot = np.loadtxt(os.path.join(interact_dir, "rotation.txt"))
                vertices_aligned = s*np.matmul(rot, vertices.transpose()) + trans.reshape(3, 1)
                mcubes.export_obj(vertices_aligned.transpose(),
                                triangles,
                                os.path.join(cluster_dir, "mesh_{:02d}.obj".format(new_key)))
    
    def get_leaf_nodes(self, node=None):
        leafs = []
        def _get_leaf_nodes( node):
            if node is not None:
                if len(node.children) == 0:
                    leafs.append(node)
                for n in node.children:
                    _get_leaf_nodes(n)
        if node is not None:
            _get_leaf_nodes(node)
        else:
            _get_leaf_nodes(self.head)
        return leafs

    def print(self):
        tree = Tree() 
        queue = list()
        visit = list()
        queue.append(self.head)

        while queue:
            node = queue.pop(0)
            if node not in visit:
                visit.append(node.key)
                queue.extend(node.children)
                if node is self.head:
                    tree.create_node(node.key, node.key)
                else:
                    tree.create_node(node.key, node.key, parent=node.parent.key)
        print(tree.show(stdout=False))
        return tree

    def return_all_child_indicies(self, key):
        """Function to return all children indicies given a node. Used in filtering by coarse mask"""
        visit = list()
        queue = list()
        node = self.key_dict[key]
        queue.append(node)
        while queue:
            thisnode = queue.pop(0)
            if thisnode.key not in visit:
                visit.append(thisnode.key)
                queue.extend(thisnode.children)
        
        return visit

    def check_inclusion(self, key, child, threshold=0.8):
        current_cluster = self.grid_idx_per_cluster[key]
        child_cluster = self.grid_idx_per_cluster[child.key]
        # let's check whether current cluster is included in child cluster
        # if current cluster is included -> add as child's child
        # if not, add another brother node
        # primitive implementation based on for loop
        total_included = 0
        for idx in range(len(current_cluster)):
            point = current_cluster[idx]
            # include = (point == child_cluster).all()
            match_x = point[0] == child_cluster[:, 0]
            match_y = point[1] == child_cluster[:, 1]
            match_z = point[2] == child_cluster[:, 2]
            include = match_x * match_y * match_z
            include = include.sum() > 0
            if include:
                total_included += 1
        # how to see if included? - some type of threshold
        ratio = total_included / len(current_cluster)
        ratio_child = total_included / len(child_cluster)
        ratio_total = max(ratio, ratio_child)
        return ratio_total > threshold, ratio_child