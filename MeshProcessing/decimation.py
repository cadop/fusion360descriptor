import pymeshlab
import os


folder_path = "G:\OmniverseFiles\JointTests\FullBody_Simplified_description\meshes"
files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]

for file in files:
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(os.path.join(folder_path, file))
    # ms.load_new_mesh(f'{file}.stl')


    ms.meshing_decimation_quadric_edge_collapse(targetfacenum=20000, 
                                                qualitythr=1, 
                                                preserveboundary=True, 
                                                boundaryweight=1, 
                                                preservenormal=True, 
                                                preservetopology=True, 
                                                optimalplacement=True, 
                                                planarquadric=False, 
                                                planarweight=0.001, 
                                                qualityweight=False, 
                                                autoclean=True, 
                                                selected=False) 
    
    ms.meshing_decimation_quadric_edge_collapse(targetperc=0.8, 
                                                qualitythr=1, 
                                                preserveboundary=True, 
                                                boundaryweight=1, 
                                                preservenormal=True, 
                                                preservetopology=True, 
                                                optimalplacement=True, 
                                                planarquadric=False, 
                                                planarweight=0.001, 
                                                qualityweight=False, 
                                                autoclean=True, 
                                                selected=False) 

    ms.save_current_mesh(file)
    # ms.save_current_mesh(f'{file}.stl')
