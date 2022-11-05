import copy
import numpy as np
import cv2

def judgement(value,backgroundcolor,threshold):
    if backgroundcolor=='white':
        if value > threshold:
            return True
        else:
            return False
    else:
        if value < threshold:
            return True
        else:
            return False

def side_detection(blackarea,point,threshold):
    y=int(point[0])
    x=int(point[1])
    n=len(blackarea)
    m=len(blackarea[0])
    count1=0
    count2=0
    count3=0
    count4=0
    judge_x=0
    judge_y=0

    for i in range(threshold, n):
        if blackarea[i][x]==1:  #left size black points detection
            if(i<y):
                count1 += 1
            elif(i>y):
                count2 += 1

        if((count1>=3) & (count2>=3)):
            judge_y=1
            break

    for i in range(m):
        if blackarea[y][i]==1:
            if(i<x):
                count3 +=1
            elif(i>x):
                count4 +=1

        if((count3>=3) & (count4>=3)):
            judge_x=1
            break

    if(judge_y & judge_x):
        return True
    else:
        return False

def colorchanging(blackarea,potlist):
    count_black=0
    count_color=0
    pointlist=[]

    for item in potlist:
        y = int(item[0])
        x = int(item[1])
        searchingsize=2
        for i in range(-searchingsize+y,searchingsize+1+y):
            for j in range(-searchingsize+x,searchingsize+1+x):
                if blackarea[i][j] ==1:
                    count_black=count_black+1
                else:
                    count_color=count_color+1
        if ((count_color> 3) &( count_black>3)) :
        # if count_color > 4:
            pointlist.append(item)

        count_color=0
        count_black=0
    return pointlist

def cornerpos(potlist):
    xsmallpoint=[0,1000]
    ysmallpoint=[1000,0]
    xbigpoint=[0,0]
    ybigpoint=[0,0]
    pot=[]
    for item in potlist:
        if item[0]>ybigpoint[0]:
            ybigpoint=item
        if item[0]<ysmallpoint[0]:
            ysmallpoint=item
        if item[1]>xbigpoint[1]:
            xbigpoint=item
        if item[1]<xsmallpoint[1]:
            xsmallpoint=item
    # sequence is botton,left,top,right
    pot.append(ybigpoint)
    pot.append(xsmallpoint)
    pot.append(ysmallpoint)
    pot.append(xbigpoint)
    return pot

def main(backgroundcolor,img,trigger=False,return_center=False):

    '''
    读取图像
    '''

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    '''
    角点检测
    '''

    corners = cv2.goodFeaturesToTrack(gray, 2000 , 0.002, 2)
    point_num=len(corners)
    img1 = copy.deepcopy(img)
    if trigger==True:
        while True:
            for i in range(point_num):
                cv2.circle(img1, (int(corners[i][0][0]), int(corners[i][0][1])), 3, (0, 0, 255), -1)
            cv2.imshow('detected_point',img1)
            key=cv2.waitKey()
            if key & 0xFF == ord('q') or key == 27:
                break

    '''
    过滤掉黑色背景外的角点
    '''

    n=len(img)
    m=len(img[0])
    blackarea=np.zeros((n,m),int)
    potlist=[]
    threshold=70
    y_space_threshold=140
    for i in range(n):
        for j in range(m):
            if judgement(gray[i][j],backgroundcolor,threshold):
                blackarea[i][j] = 1
    for item in corners:
        x, y = item.ravel()
        point = [y, x]
        if side_detection(blackarea, point, y_space_threshold):
            potlist.append(point)

    point_num=len(potlist)
    img2 = copy.deepcopy(img)
    if trigger==True:
        while True:
            for i in range(point_num):
                cv2.circle(img2, (int(potlist[i][1]), int(potlist[i][0])), 3, (0, 0, 255), -1)
            cv2.imshow('background_selected',img2)
            key=cv2.waitKey()
            if key & 0xFF == ord('q') or key == 27:
                break

    '''
    选择颜色变化边缘的点
    '''

    potlist=colorchanging(blackarea,potlist)

    point_num=len(potlist)
    img3 = copy.deepcopy(img)
    if trigger==True:
        while True:
            for i in range(point_num):
                cv2.circle(img3, (int(potlist[i][1]), int(potlist[i][0])), 3, (0, 0, 255), -1)
            cv2.imshow('color_select',img3)
            key=cv2.waitKey()
            if key & 0xFF == ord('q') or key == 27:
                break

    '''
    选择角点
    '''

    potlist=cornerpos(potlist)
    point_num = len(potlist)
    img4 = copy.deepcopy(img)
    if trigger==True:
        while True:
            for i in range(point_num):
                cv2.circle(img4, (int(potlist[i][1]), int(potlist[i][0])), 3, (0, 0, 255), -1)
            cv2.imshow('final_result',img4)
            key=cv2.waitKey()
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    potlistt=np.zeros((point_num,2))
    for i in range(point_num):
        x = potlist[i][1]
        y= potlist[i][0]
        potlistt[i][0]=x
        potlistt[i][1]=y

    '''
    return 
    '''
    if return_center == True:
        potlistt=np.array(potlistt)
        center=np.mean(potlistt,axis=0)
        return center


    return potlistt