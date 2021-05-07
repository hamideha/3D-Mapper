import numpy as np
import open3d as o3d

if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("FinalProjectData.xyz", format='xyz')
    print(pcd)
    lines = [] # Lines array used by Open3D to connect points
    points = [0] * 64
    planeOffset = 0
    do = 64 # Displacement offset for each plane
    
    for i in range(64):
        points[i] = i  
        
    for i in range (10): # 10 planes were measured
        for j in range(64): # Connect lines between adjacent points in the same plane
            if j == 63: # To avoid index range error for last point
                lines.append([points[63] + planeOffset, points[1] + planeOffset])
            else:
                lines.append([points[j] + planeOffset, points[j+1] + planeOffset]) # Connect line between every adjacent point
        planeOffset += 64    
            
    planeOffset = 0 # Reset the plane offset and every point 
    for i in range(64):
        points[i] = i  
    
    for i in range(9):
        for j in range(64): # Connect lines between adjacent planes
            lines.append([points[j] + planeOffset, points[j] + planeOffset + do])
        planeOffset += 64

    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))

    o3d.visualization.draw_geometries([line_set])
    
    # viewer = o3d.visualization.Visualizer() # Display axes on Open3D visualization
    # viewer.create_window()
    # viewer.add_geometry(line_set)
    # opt = viewer.get_render_option()
    # opt.show_coordinate_frame = True
    # viewer.run()
