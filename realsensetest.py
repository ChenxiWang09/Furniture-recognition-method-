import numpy as np

import realsense
import featureextraction_ShiTomas
import cv2
import cv2.aruco as aruco

realsense_client: object = realsense.RealSense()
realsense_client.waitforframe()

while True:
    img = realsense_client.get_rgb()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('img',img)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        break
center=featureextraction_ShiTomas.main('black',img,trigger=True,return_center=True)
camera_coordinate=realsense_client.get_3d_camera_coordinate(center)
camera_coordinate=np.array((camera_coordinate[0],camera_coordinate[1],camera_coordinate[2],1)).T
homogeneous_matrix=np.load('homo_transformation_matrix.npy')
world_coordinate=np.dot(np.load('homo_transformation_matrix.npy'),camera_coordinate)
print(world_coordinate)

# center=featureextraction_ShiTomas.main('black',img,trigger=True)
# camera_coordinate1=realsense_client.get_3d_camera_coordinate(center[0])
# camera_coordinate2=realsense_client.get_3d_camera_coordinate(center[1])
# camera_coordinate3=realsense_client.get_3d_camera_coordinate(center[2])
# camera_coordinate4=realsense_client.get_3d_camera_coordinate(center[3])
#
# world_coordinate1=np.dot(np.load('homo_transformation_matrix.npy'),np.array((camera_coordinate1[0],camera_coordinate1[1],camera_coordinate1[2],1)))
# world_coordinate2=np.dot(np.load('homo_transformation_matrix.npy'),np.array((camera_coordinate2[0],camera_coordinate2[1],camera_coordinate2[2],1)))
# world_coordinate3=np.dot(np.load('homo_transformation_matrix.npy'),np.array((camera_coordinate3[0],camera_coordinate3[1],camera_coordinate3[2],1)))
# world_coordinate4=np.dot(np.load('homo_transformation_matrix.npy'),np.array((camera_coordinate4[0],camera_coordinate4[1],camera_coordinate4[2],1)))
#
# print(world_coordinate1)
# print(world_coordinate2)
# print(world_coordinate3)
# print(world_coordinate4)

# realsense_client: object = realsense.RealSense()
# depth=[320,240]
# co=realsense_client.getcenter()
# print(co)
# while True:
#     # pcd = realsense_client.rgbd2pcd(toggledebug=True)
#     img = realsense_client.get_rgb()
#     cv2.imshow('img1', img)
#     key = cv2.waitKey(1)
#     if key == ord('q'):
#         break
#     parameters = aruco.DetectorParameters_create()
#     aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
#     corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
#     print(ids)
# realsense_client.get_extrinsic_matrix(trigger=True)