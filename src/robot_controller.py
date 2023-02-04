from scipy.spatial.transform import Rotation
import numpy as np
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import math 
import random
import rospy



class PathFolower:
    def __init__(self, UR_IP) -> None:
        self._rtde_c = RTDEControlInterface(UR_IP)
        self._rtde_r = RTDEReceiveInterface(UR_IP)
    


    def follow(self, path):
        cam_to_gripper = np.array([
            [1, 0, 0, -0.03284863],
            [0, 1, 0, -0.08],
            [0, 0, 1, -0.08414625],
            [0, 0, 0, 1],
        ])
        for p in path:
            if p[3] is None:
                new_pose = self._rtde_r.getActualTCPPose()[3:]
                self._rtde_c.moveL(list(p)+list(new_pose), 0.1, 0.1)
            else:
                self._rtde_c.moveL(list(p), 0.1, 0.1)
            actual = self._rtde_r.getActualTCPPose()
            r_matrix = Rotation.from_rotvec(actual[3:]).as_matrix()
            t_matrix = np.eye(4)
            for i,v in enumerate(r_matrix):
                t_matrix[i,0:3] = v
                t_matrix[i, 3] = actual[i]
            t_matrix = t_matrix @ cam_to_gripper
            yield t_matrix



    



class PathGenerator:
    def Circle(r, normal, center, wp_count, look_at_point = None):
        '''
        r - circle radius
        normal - circle normal
        wp_count - count of waypoints
        '''
        normal = np.array(normal, dtype=np.float32)
        normal /= np.linalg.norm(normal)
        center = np.array(center)
        path = np.zeros((wp_count, 6))
        
        if normal[2] != 0:
            rand_x = random.random()
            rand_y = random.random()
            rand_z = (normal[0]*rand_x+normal[1]*rand_y)/(-normal[2])
        elif normal[1] != 0:
            rand_x = random.random()
            rand_z = random.random()
            rand_y = (normal[2]*rand_z+normal[0]*rand_x)/(-normal[1])
        elif normal[0] != 0:
            rand_y = random.random()
            rand_z = random.random()
            rand_x = (normal[1]*rand_y+normal[2]*rand_z)/(-normal[0])
        x_vec = np.array([rand_x, rand_y, rand_z])
        x_vec/=np.linalg.norm(x_vec)
        y_vec = np.cross(normal, x_vec)
        y_vec/=np.linalg.norm(y_vec)
        r_mat = np.array([x_vec, y_vec, normal]).T
        for i in range(wp_count):
            path[i, 0] = r*math.cos(2*i*math.pi/(wp_count-1))
            path[i, 1] = r*math.sin(2*i*math.pi/(wp_count-1))
            path[i, :3] = r_mat @ path[i, :3]
            path[i, :3] += center
            path[i, 3:] = np.array([None]*3) if look_at_point is None else PathGenerator.look_at(path[i, :3], look_at_point)
            
            
        return path



    def Waypoints(count, pf : PathFolower):
        pf._rtde_c.teachMode()
        path = []
        for i in range(count):
            rospy.loginfo('move manipulator to point and press enter')
            if not rospy.is_shutdown():
                input()
            path.append(pf._rtde_r.getActualTCPPose())
            rospy.loginfo(path[-1])
        pf._rtde_c.endTeachMode()
        return np.array(path)



    def look_at(point, view_point):
        point = np.array(point)
        cur_point = np.array( view_point)
        z_vec = ( cur_point - point)
        z_vec /= np.linalg.norm(z_vec)

        x_vec = np.cross([0, 0, 1], z_vec)
        x_vec /= np.linalg.norm(x_vec)

        y_vec = np.cross(z_vec,x_vec)
        y_vec /= np.linalg.norm(y_vec)
        m = np.array([x_vec, y_vec, z_vec]).T
        rvec = Rotation.from_matrix(m).as_rotvec()
        return rvec


    def draw_path(path, save_dir = None):
        '''
        draw point with plt
        '''
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')   
        ax.scatter(path[:, 0], path[:, 1], path[:, 2])
        vectors = []
        for p in path:
            mat = Rotation.from_rotvec(p[3:]).as_matrix()
            vectors.append(list(mat @ [0, 0, 1]))
        vectors = np.array(vectors)
        
        ax.quiver(path[:, 0], path[:, 1], path[:, 2], vectors[:, 0], vectors[:, 1], vectors[:, 2], length=0.03, normalize=True, color='red')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        if save_dir is None:
            plt.show()
        else:
            plt.savefig(save_dir)


    


