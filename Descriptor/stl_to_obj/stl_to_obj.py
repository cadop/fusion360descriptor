import struct
import time


def stl_to_obj(stl_file, obj_file=None, precision=None, verbose=False, scale=1.0):
    """
    Converts an STL file to an OBJ file.

    Args:
        stl_file (str): The path to the STL file.
        obj_file (str, optional): The path to the OBJ file. If not provided, a default path will be used.
        precision (int, optional): The number of decimal places to round the vertex coordinates. Default is None.
        verbose (bool, optional): If True, prints additional information during the conversion process. Default is False.

    Returns:
        None
    """
    if verbose:
        start_time = time.time()
    verts = []
    verts_idx = {}
    faces = []
    with open(stl_file, 'rb') as f:
        header = f.read(80)
        num_triangles = int.from_bytes(f.read(4), 'little')
        if verbose:
            print(f'File Header: {header}')
            print(f'Triangles: {num_triangles}')
            print(f'Vertices: {num_triangles * 3}')
        for [nx, ny, nz,
             *face_verts,
             attributes] in struct.iter_unpack('ffffffffffffH', f.read()):

            [vax, vay, vaz,
             vbx, vby, vbz,
             vcx, vcy, vcz] = map(lambda pos: round(pos, precision), face_verts) if precision else face_verts
            va = (vax, vay, vaz)
            if va not in verts_idx:
                verts.append(va)
                verts_idx[va] = len(verts) - 1
            va = verts_idx[va]

            vb = (vbx, vby, vbz)
            if vb not in verts_idx:
                verts.append(vb)
                verts_idx[vb] = len(verts) - 1
            vb = verts_idx[vb]

            vc = (vcx, vcy, vcz)
            if vc not in verts_idx:
                verts.append(vc)
                verts_idx[vc] = len(verts) - 1
            vc = verts_idx[vc]

            faces.append((va, vb, vc))

    if not obj_file:
        obj_file = f'{stl_file[:-3]}obj'
    if verbose:
        print(f'New Vertices: {len(verts)}')
        print(f'Writing to {obj_file}')
    with open(obj_file, 'w+') as f:
        for vert in verts:
            vert = tuple(v / scale for v in vert)
            f.write(f'v {vert[0]} {vert[1]} {vert[2]}\n')
        for face in faces:
            f.write(f'f {face[0] + 1} {face[1] + 1} {face[2] + 1}\n')
    if verbose:
        print(f'Conversion took {time.time() - start_time} seconds')
