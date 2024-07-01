import os
from tetgen import TetGen
from pathlib import Path
import pyvista as pv
import numpy as np
import trimesh


def parse_args():
    from argparse import ArgumentParser
    parser = ArgumentParser(description='Tetrahedralize a 3D mesh')
    parser.add_argument('--input', type=str, default='', help='Input mesh file (.obj)')
    parser.add_argument('--flag', type=str, default="pq1.1/0YV", help='Flag for tetgen')
    parser.add_argument('--visualize', action='store_true', help='Visualize the mesh')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    input_path = Path(args.input)
    if not input_path.exists():
        print(f'Error: {input_path} not found')
        exit(-1)
    print(f'Input Mesh: {input_path}')


    out_without_suffix = input_path.name
    out_without_suffix = out_without_suffix[:out_without_suffix.rfind('.')]
    node_output_name = f'{out_without_suffix}_vertices.npy'
    elem_output_name = f'{out_without_suffix}_elements.npy'


    tri_mesh = trimesh.load_mesh(input_path)
    bbox_min = np.min(tri_mesh.vertices, axis=0)
    bbox_max = np.max(tri_mesh.vertices, axis=0)
    bbox_center = (bbox_min + bbox_max) / 2
    bbox_extent = bbox_max - bbox_min
    bbox_max_extent = np.max(bbox_extent)

    # Scale the tri_mesh to fit in a unit cube
    tri_mesh.apply_translation(-bbox_center)
    tri_mesh.apply_scale(1 / bbox_max_extent)

    tempfile = '.tmp.stl'
    tri_mesh.export(tempfile)
    mesh = pv.read(tempfile)
    os.remove(tempfile)
    print(mesh)

    # Run tetgen
    tg = TetGen(mesh)
    tg.make_manifold()
    # nodes, elements = tg.tetrahedralize()
    nodes, elements = tg.tetrahedralize(switches=args.flag)
    print(f'Nodes: {nodes.shape[0]}, Elements: {elements.shape[0]}')

    if args.visualize:
        tet_min = np.min(tri_mesh.vertices, axis=0)
        tet_max = np.max(tri_mesh.vertices, axis=0)
        grid = tg.grid
        for i in range(1, 10):
            ratio = 0.1 * i
            thre = (1 - ratio) * tet_min + ratio * tet_max
            mask = grid.points[:, 2] < ratio * thre[2]
            half_tet = grid.extract_points(mask)
            p = pv.Plotter()
            p.add_mesh(half_tet, color='w', show_edges=True)
            p.add_mesh(grid, opacity=0.2)
            p.show()
            p.close()


    # Save nodes and elements
    nodes = nodes * bbox_max_extent + bbox_center
    nodes = nodes.astype(np.float64)
    elements = elements.astype(np.int64)
    np.save(node_output_name, nodes)
    np.save(elem_output_name, elements)
