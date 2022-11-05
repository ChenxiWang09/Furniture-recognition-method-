import math

import cv2
import numpy as np
import featureextraction_ShiTomas
from cv2 import aruco as aruco
import realsense
import rotate_matrix
import move_action as ma
from time import sleep


'''
compute homogeneous transformation matrix
'''
# notice: while using different size or different number's aruco maker , plz change the parameter in the realsense file.
def coordinate_transformation(realsense_client, camera_coordinate, tgt_pos, tgt_rot, save=False):
    # camera coordinate system to tcp coordinate system
    extrinsic_matrix=realsense_client.get_extrinsic_matrix(trigger=False)
    tcp_coordinate=np.dot(np.linalg.inv(extrinsic_matrix),camera_coordinate)
    print(tcp_coordinate)
    # tcp coordinate system to world coordinate system
    # rotate vector and transformation matrix based on the aruco maker's position and pose
    x_plus=76
    y_plus=0
    z_plus=50


    trans=np.array([(tgt_pos[0]+x_plus, tgt_pos[1]+y_plus,780+z_plus)]).T
    transformation_matrix=np.hstack((tgt_rot,trans))
    transformation_matrix=np.vstack((transformation_matrix,np.array([0,0,0,1])))
    homo_transformation_matrix=np.dot(transformation_matrix,np.linalg.inv(extrinsic_matrix))

    world_coordinate = np.dot(homo_transformation_matrix,camera_coordinate)
    print(world_coordinate)
    if save==True:
        np.save("extrinsic_matrix.npy",extrinsic_matrix)
        np.save("homo_transformation_matrix.npy",homo_transformation_matrix)
        print("successfully save the homogeneous matrix!")
    return homo_transformation_matrix

if __name__ == '__main__':

    '''
    planning track
    '''
    # ma_ex = ma.ur3eusing_example()
    # n=10 # 10 different angle
    # unit = 8
    # # jnts = ma_ex.start_pos()
    # start_jnts=np.load('data/start_jnts.npy')
    # ma_ex.just_move(start_jnts)
    #
    # [jnts,pos,rot]=ma_ex.rotate_move(angle=-n/2*unit,axis='z',planning=True,save=True,move=True,data_need=True)
    # np.save('data/track_jnts_' + str(0) + '.npy', jnts)
    # for i in range(n):
    #     [jnts,pos,rot]=ma_ex.rotate_move(angle=unit,axis='z',planning=True,save=True,move=True,data_need=True)
    #     jnts=np.array(jnts)
    #     np.save('data/track_jnts_'+str(i+1)+ '.npy', jnts)
    #     print("Successfully save the path!")

    '''
    compute homogeneous transformation matrix
    '''

    # realsense_client: object = realsense.RealSense()
    # n = 11
    # homogeneous_matrixs=np.zeros((n, 4, 4))
    # homo_matrix = np.zeros((4, 4))
    #
    # ma_ex = ma.ur3eusing_example()
    # start_jnts = np.load('data/start_jnts.npy')
    # ma_ex.just_move(start_jnts)
    # # [pos, rot_rat] = ma_ex.just_move(start_jnts, data_need=True)
    # # camera_coordinate = np.hstack((realsense_client.getcenter(), np.array([1]))).T
    # # homogeneous_matrixs[i] = coordinate_transformation(realsense_client=realsense_client,
    # #                                                    camera_coordinate=camera_coordinate, tgt_pos=pos,
    # #                                                    tgt_rot=rot_rat)
    # for i in range(n):
    #     jnts = np.load('data/track_jnts_0.npy')
    #     jnts=np.load('data/track_jnts_'+str(i)+'.npy',allow_pickle=True)
    #     [pos,rot_rat]=ma_ex.just_move(jnts,data_need=True)
    #     pos=[pos[0]+math.cos(-40+i*8)*76,pos[1]+math.sin(40-8*i)*76,pos[2]]
    #     rot_rat=np.dot(rot_rat,np.array([(1,0,0),(0,0,1),(0,-1,0)]))
    #     camera_coordinate = np.hstack((realsense_client.getcenter(), np.array([1]))).T
    #     homogeneous_matrixs[i]=coordinate_transformation(realsense_client=realsense_client,camera_coordinate=camera_coordinate,tgt_pos=pos,tgt_rot=rot_rat)
    #     homo_matrix+=homogeneous_matrixs[i]
    #
    # homo_matrix=homo_matrix/n
    # np.save("homo_transformation_matrix.npy", homo_matrix)
    # print(homo_matrix)
    # print("successfully save the homogeneous matrix!")

    '''
    test x, y, z orientation error
    '''

    ma_ex = ma.ur3eusing_example()
    start_jnts = np.load('data/start_jnts.npy')
    ma_ex.just_move(start_jnts)
    realsense_client: object = realsense.RealSense()
    homo_transformation_matrix=np.load('homo_transformation_matrix.npy')
    error=0
    stick=0
    step=5
    for i in range(10):
        try:
            [jnts,pos,rot_rat]=ma_ex.axis_line_move(distance=step,axis='x',planning=True,save=True,move=True,data_need=True)
        except:
            continue
        sleep(0.5)
        camera_coordinate = np.hstack((realsense_client.getcenter(), np.array([1]))).T
        world_coordinate = np.dot(homo_transformation_matrix, camera_coordinate)
        error0 = np.array(((world_coordinate[0] - pos[0] - 76) , (world_coordinate[1] - pos[1]), (world_coordinate[2] - 830)))
        print('error0:', error0)
        error = error + math.sqrt((world_coordinate[0]-pos[0]-76)**2+(world_coordinate[1]-pos[1])**2+(world_coordinate[2]-830)**2)
        stick += 1

    ma_ex.just_move(start_jnts)
    for i in range(10):
        try:
            [jnts,pos,rot_rat]=ma_ex.axis_line_move(distance=step,axis='y',planning=True,save=True,move=True,data_need=True)
        except:
            continue
        sleep(0.5)
        camera_coordinate = np.hstack((realsense_client.getcenter(), np.array([1]))).T
        world_coordinate = np.dot(homo_transformation_matrix, camera_coordinate)
        error0 = np.array(((world_coordinate[0] - pos[0] - 76) , (world_coordinate[1] - pos[1]), (world_coordinate[2] - 830)))
        print('error0:', error0)
        error = error + math.sqrt((world_coordinate[0]-pos[0]-76)**2+(world_coordinate[1]-pos[1])**2+(world_coordinate[2]-830)**2)
        stick += 1

    ma_ex.just_move(start_jnts)
    for i in range(10):
        try:
            [jnts, pos, rot_rat] = ma_ex.axis_line_move(distance=step, axis='z', planning=True, save=True, move=True,
                                                        data_need=True)
        except:
            continue
        sleep(0.5)
        camera_coordinate = np.hstack((realsense_client.getcenter(), np.array([1]))).T
        world_coordinate = np.dot(homo_transformation_matrix, camera_coordinate)
        error0 = np.array(
            ((world_coordinate[0] - pos[0] - 76), (world_coordinate[1] - pos[1]), (world_coordinate[2] - 830)))
        print('error0:', error0)
        error = error + math.sqrt((world_coordinate[0] - pos[0] - 76) ** 2 + (world_coordinate[1] - pos[1]) ** 2 + (
                    world_coordinate[2] - 830) ** 2)
        stick += 1
    error=error/stick
    print('error:',error)




    # objrot = np.dot(rot_y, np.array([(-1, 0, 0), (0, -1, 0), (0, 0, 1)]))
    # objpos = np.array([620, 50, 930])
    # objcm = el.loadObj("calibboard.stl")
    # grasp = pickle.load(open(config.ROOT + "/graspplanner/pregrasp/calibboard_pregrasps.pkl", "rb"))[0]
    # for x in range(500, 750, 10):
    #     objrelpos, objrelrot = mp_lft.get_rel_posrot(grasp, objpos=(x, 50, 930), objrot=objrot)
    #     if objrelrot is not None:
    #         break
    # print("objrelpos:", objrelpos, "objrelrot:", objrelrot)
    # success = mp_x_lft.goto_objmat4_goal_x(grasp, objrelpos, objrelrot, rm.homobuild(objpos, objrot), objcm)
    # # camera_coordinate = np.hstack((realsense_client.getcenter(), np.array([1]))).T
    # # world_coordinate = np.dot(homo_transformation_matrix, camera_coordinate)
    # # error = world_coordinate[0] - 620
    # # print(error)
    #
    #
    # # x axis
    # x_error=0
    # times=0
    # for x in range(600,700,10):
    #     objpos=(x, 50, 930)
    #     success = mp_x_lft.goto_objmat4_goal_x(grasp, objrelpos, objrelrot, rm.homobuild(objpos, objrot), objcm)
    #     if success:
    #         camera_coordinate = np.hstack((realsense_client.getcenter(), np.array([1]))).T
    #         world_coordinate = np.dot(homo_transformation_matrix, camera_coordinate)
    #         x_error += world_coordinate[0] - x
    #         times  += 1
    # x_error = x_error / times
    #
    # # y axis
    # y_error=0
    # times=0
    # for y in range(-100,50,10):
    #     objpos = (630, y, 930)
    #     success = mp_x_lft.goto_objmat4_goal_x(grasp, objrelpos, objrelrot, rm.homobuild(objpos, objrot), objcm)
    #     if success:
    #         camera_coordinate = np.hstack((realsense_client.getcenter(), np.array([1]))).T
    #         world_coordinate = np.dot(homo_transformation_matrix, camera_coordinate)
    #         y_error = y_error + world_coordinate[1] - y
    #         times  = times + 1
    # y_error = y_error / times
    #
    # #
    # # z axis
    # z_error=0
    # times=0
    # for z in range(900,990,10):
    #     objpos = (620, 50, z)
    #     success = mp_x_lft.goto_objmat4_goal_x(grasp, objrelpos, objrelrot, rm.homobuild(objpos, objrot), objcm)
    #     if success:
    #         camera_coordinate = np.hstack((realsense_client.getcenter(), np.array([1]))).T
    #         world_coordinate = np.dot(homo_transformation_matrix, camera_coordinate)
    #         z_error += world_coordinate[1] - z
    #         times  += 1
    # z_error = z_error / times
    # print('x_error:')
    # print(x_error)
    # print('y_error:')
    # print(y_error)
    # print('z_error:')
    # print(z_error)

    '''
    use transformation matrix
    '''

    # realsense_client: object = realsense.RealSense()
    # homo_matrix= np.load('homo_transformation_matrix.npy')
    # img = realsense_client.get_rgb()
    # center = featureextraction_ShiTomas.main('black', img, trigger=True, return_center=True)
    # maex = ma.ur3eusing_example()
    # center = np.array([ (650.68834407  ,4.16613905,  850.7919856 ) ])
    # '''
    # set param
    # '''
    #
    #
    # objrot = np.array([(0, 1, 0), (1, 0, 0), (0, 0, -1)])
    # objpos = center
    # pos=maex.axis_line_move(distance=[objpos,objrot],axis='zero')
