#!/usr/bin/env python3
# license removed for brevity
#策略 機械手臂 四點來回跑
import sys
sys.path.insert(1, "/usr/local/lib/python3.5/dist-packages/")
sys.path.insert(0, '/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/')
import cv2
import threading
import time
import rospy
import os
import numpy as np
#import pyrealsense2 as rs
#import pyrealsense2 as rs
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from ROS_Socket.srv import *
#from getcalue.srv import *
#from zed_image_process.srv import *
from ROS_Socket.msg import *
import math
import enum
import Hiwin_RT605_Socket as ArmTask
from std_msgs.msg import Int32MultiArray
##----Arm state-----------
Arm_state_flag = 0
Strategy_flag = 0
Sent_data_flag = 1
##----Arm sttus enum
class Arm_status(enum.IntEnum):
    Idle = 0
    Isbusy = 1
    Error = 2
    shutdown = 6
def callback(state):
    global Arm_state_flag,Sent_data_flag
    Arm_state_flag = state.data[0]
    Sent_data_flag = state.data[1]
def arm_state_listener():

    rospy.Subscriber("chatter", Int32MultiArray, callback)
##-----------switch define------------##
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

##------------class-------
class point():
    def __init__(self,x,y,z,pitch,roll,yaw):
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
pos = point(0,36.8,11.35,-90,0,0)

class get_point():
    def __init__(self,label,x_max,x_min,y_max,y_min,down_z,center_x,center_y):
        self.label = label
        self.x_max = x_max
        self.x_min = x_min
        self.y_max = y_max
        self.y_min = y_min
        self.down_z = down_z
        self.center_x = center_x 
        self.center_y = center_y

image_point = get_point('bottle',10,0,10,0,5,0,0)

global cntr_point

class get_real():
    def __init__(self,roll,pitch,yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
realsense_point = get_real(0,0,0)

class get_depth_point():
    def __init__(self,x,y,w,h,depth):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.depth = depth
resp1 = get_depth_point(0.0,0.0,0.0,0.0,0.0)

class star_cfg():
    def __init__(self,yolo_z,down_z):
        self.yolo_z = yolo_z    # === need to change to avoid sucker hit the box ===
        self.down_z = down_z    # receive what realsense got z
star_config = star_cfg(-7.65,-26.35)
##-------------------------strategy---------------------

action = 0

cup_color = ""    #decide what's color that the cup is  

down_stop_flag = 1 #Arm stop when sucker already suck something
count_stop = 0    #decide what times need to run

pressure_info = ""


#for test
list_cnt_x = []
list_cnt_y = []
list_angle = ['0','0','0','0','0','0']
list_area = []

'''
list_cnt_x = [0, 11.3, 0, 11.3, 10.7, 0]
list_cnt_y = [37.5, 37.8, 45.4, 45.6, 53.5, 53.3]
list_angle = ['0','0','0','0','0','0']
list_area = []
'''
angle_dif = 0 #angle between two img

#for realsense
'''
cap=cv2.VideoCapture(1)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)  # 1280*720 fps = 15(在高會爆炸)
cfg = pipeline.start(config)
dev = cfg.get_device()
'''
##============================= function =========================================##

def callback_receive_sucker(data):   #read the value of pressure
    global down_stop_flag,pressure_info
    pressure_info = data.data
    down_stop_flag = int(pressure_info)
    
    
def callback_yolo_receive(data):
    global temp_label
    temp_label = data.data #receive label to temp 

def sent_sucker_signal(signal):  #let sucker enable or disable (input is on or off) 
    pub = rospy.Publisher("raspberry_sent", String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo(signal)
    pub.publish(signal)
    rate.sleep()  

def test_arduino_func():  #test pressure
    global pressure_info,action,Arm_state_flag,Sent_data_flag,down_stop_flag,count_stop
    if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        Sent_data_flag = 0
        Arm_state_flag = Arm_status.Isbusy

        for case in switch(action): #傳送指令給socket選擇手臂動作
            if case(0):      #Arm to BOX above
                print("---Arm left---")
                sent_sucker_signal("on") 
                pos.x = 15
                pos.y = 45.5
                pos.z = 1.35
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 1
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                ArmTask.Speed_Mode(1)
                ArmTask.Arm_Mode(4,1,0,30,2)#action,ra,grip,vel,both

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both

                time.sleep(3)

                break


            if case(1):  #catch items in box(down)
                print("---arm right---")
                sent_sucker_signal("off") 
                pos.x = -10
                pos.y = 45.5
                pos.z = 1.35
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 0
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both
                time.sleep(3) #please delete
                count_stop = count_stop +1
                if count_stop == 2:
                    action = 3

                break

            if case(3):
                print("---test finish---")
                sent_sucker_signal("off") 
                pos.x = 0
                pos.y = 36.8
                pos.z = 11.35
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 0
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                ArmTask.Speed_Mode(0)
                ArmTask.Arm_Mode(4,1,0,30,2)#action,ra,grip,vel,both

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both
                count_stop = count_stop +1
                break   

def take_pic():
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    images = color_image

    ret, RealSense = cap.read()
    cv2.imshow('RealSense', images)
    
    path = '/home/l/Desktop'
    cv2.imwrite(os.path.join(path,'puzzle.png'),images)

    cap.release()
    cv2.destroyAllWindows()
    pipeline.stop()

    img = cv2.imread("/home/l/Desktop/puzzle.png")
    return img

def drawMatches(img1, kp1, img2, kp2, matches):
    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    list_x1 = []
    list_y1 = []
    list_x2 = []
    list_y2 = []

    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1] = np.dstack([img1, img1, img1])

    # Place the next image to the right of it
    out[:rows2,cols1:] = np.dstack([img2, img2, img2])

    cnt = 0
    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for mat in matches:
        # Get the matching keypoints for each of the images
        img1_idx = mat.queryIdx
        img2_idx = mat.trainIdx

        # x - columns
        # y - rows
        (x1,y1) = kp1[img1_idx].pt
        (x2,y2) = kp2[img2_idx].pt
    
        list_x1.append(x1)
        list_y1.append(y1)
        list_x2.append(x2)
        list_y2.append(y2)
        #print(x2,y2)

        # Draw a small circle at both co-ordinates
        a = np.random.randint(0,256)
        b = np.random.randint(0,256)
        c = np.random.randint(0,256)

        cv2.circle(out, (int(np.round(x1)),int(np.round(y1))), 2, (a, b, c), 1)      #画圆，cv2.circle()参考官方文档
        cv2.circle(out, (int(np.round(x2)+cols1),int(np.round(y2))), 2, (a, b, c), 1)

        # Draw a line in between the two points
        cv2.line(out, (int(np.round(x1)),int(np.round(y1))), (int(np.round(x2)+cols1),int(np.round(y2))), (a, b, c), 1, lineType=cv2.LINE_AA, shift=0)  #画线，cv2.line()参考官方文档
        cnt = cnt+1

    angle_calculate(list_x1,list_y1,list_x2,list_y2)   #計算兩張圖片的角度差
    # Also return the image if you'd like a copy

def area_detection(img_x,img_y):
    if (img_x[0] > 20 and img_x[0] < 500):
        if (img_y[0] > 0 and img_y[0] < 240):
            list_area.append("1")

        elif (img_y[0] > 240 and img_y[0] < 480):
            list_area.append("3")
        
        elif (img_y[0] > 480 and img_y[0] < 720):
            list_area.append("5")
        
        else:
            list_area.append("e")

    elif (img_x[0] > 500 and img_x[0] < 830):
        if (img_y[0] > 0 and img_y[0] < 270):
            list_area.append("2")

        elif (img_y[0] > 270 and img_y[0] < 510):
            list_area.append("4")

        elif (img_y[0] > 510 and img_y[0] < 720):
            list_area.append("6")

        else:
            list_area.append("e")
    
    else:
        list_area.append("e")

def angle_calculate(x1,y1 , x2,y2):
    angle1 = np.rad2deg(np.arctan2(y1[2] - y1[1], x1[2] - x1[1]))
    #print(angle1)
    angle2 = np.rad2deg(np.arctan2(y2[2] - y2[1], x2[2] - x2[1]))
    #print(angle2)

    #if (angle1 < angle2):
    angle_diff = angle1 - angle2
    #else:
    #    angle_diff = angle1 - angle2

    #print(angle_diff)
    #list_angle.append(angle_diff)   #不準確 之後要修改
    area_detection(x1,y1)  #計算位置

def Orb_feature(img1,img2):
    # Initiate STAR detector
    orb = cv2.ORB_create(10000)

    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors.
    matches = bf.match(des1,des2)

    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)

    # Draw first 10 matches.
    img3 = drawMatches(img1,kp1,img2,kp2,matches[:10])

def angle_detect(img,mode):
    #start find edge
    # 用來給辨識程式圈選個別物件用
    img_item = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_item = img_item[0:717,400:1300] #切割 

    #用來與個別物件比較用
    img_ori = cv2.imread('/home/l/Desktop/photosss.png',0)
    img_ori = img_ori[0:717,400:1300]

    #辨識用照片
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lower = np.array([35,43,46])   #濾掉綠色
    upper = np.array([77,255,255])
    mask = cv2.inRange(hsv,lower,upper)
    ret,thresh2 = cv2.threshold(mask,0,255,cv2.THRESH_BINARY_INV)
    edge_copy = thresh2[0:717,400:1300]

    #開始尋找物件
    (_,cnts,_) = cv2.findContours(edge_copy,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for (i, c) in enumerate(cnts):

        (x, y, w, h) = cv2.boundingRect(c)
        if(mode == 1): #框出最小正方形
            cv2.rectangle(edge_copy, (x-10, y-10), (x+w+10, y+h+10), (255,0,0), 2)
            #displayIMG(edge_copy,"all_items")

            items = img_item[y-5:(y + h)+5, x-5:(x + w)+5]

        elif(mode == 2): #同上但框框會旋轉來圈出最小正方形
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            print(box)
            cv2.drawContours(edge_copy, [box], 0, (255, 0, 0), 2)
            displayIMG(edge_copy,"all_items")

            items = edged[y-5:(y + h)+5, x-5:(x + w)+5]

        if (h >130 and w > 170): #大於這個範圍的才會認定為一塊拼圖
            cnt_x = x + w/2      #原圖裁掉了400現在補回去
            cnt_y = y + h/2
            
            #座標轉換(linear)
            real_x = -12.483987 + 0.035775*cnt_x
            real_y =  56.220095  -0.033941*cnt_y +1
            
            '''
            #座標轉換(curv)
            real_x = cnt_x/(-0.01*cnt_x + 0.5)
            real_y = cnt_y/(0.01*cnt_y - 0.1) 
            '''
            list_cnt_x.append(real_x)
            list_cnt_y.append(real_y)
            
            Orb_feature(img_ori,items)
  
###===================================== strategy ==================================================================###
def Mission_strategy():
    global action,count_stop,Arm_state_flag,Sent_data_flag
    global down_stop_flag,color_now

    if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        Sent_data_flag = 0
        Arm_state_flag = Arm_status.Isbusy

        for case in switch(action): #傳送指令給socket選擇手臂動作
            if case(0):      ##======================= Arm to initial point ===================================##
                print("---Arm to initial point---")

                pos.x = 0
                pos.y = 36.8
                pos.z = 1.35
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 1
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                #ArmTask.Speed_Mode(1)
                #ArmTask.Arm_Mode(4,1,0,30,2) #speed up

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both
                break

            if case(1): #ready to take picture
                print("---Arm to img point---")

                pos.x = 0
                pos.y = 36.8
                pos.z = star_config.yolo_z
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 2
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both
                break

            if case(2): #analysizing puzzle 
                print("---analysizing puzzle---")

                if (count_stop == 0):
                    #src =  take_pic()
                    src = cv2.imread('/home/l/Desktop/photosss1.png')
                    angle_detect(src,1)
                    print("area: ",list_area)
                    print("centure: ",list_cnt_x,list_cnt_y)
                else:
                    print("area: ",list_area[count_stop])
                    print("centure: ",list_cnt_x[count_stop],list_cnt_y[count_stop])
                    print("angle: ",list_angle[count_stop])
                action = 3
                break

            if case(3):
                print("---move to puzzle up---")
                sent_sucker_signal("on") 
                pos.x = float(list_cnt_x[count_stop])
                pos.y = float(list_cnt_y[count_stop])
                pos.z = pos.z
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 4
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both
                break

            if case(4):
                print("---down to suck puzzle---")
                sent_sucker_signal("on") 

                pos.x = pos.x
                pos.y = pos.y
                pos.z = star_config.down_z
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 5
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both
                break
            
            if case(5):
                print("---arm up---")
                pos.x = pos.x
                pos.y = pos.y
                pos.z = star_config.yolo_z
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 6
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both
                break

            if case(6):
                print("---rotate the puzzle---")
                pos.x = pos.x
                pos.y = pos.y
                pos.z = pos.z
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = float(list_angle[count_stop])
                action = 7
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both 
                break

            if case(7):
                print("---analysing & move to final position---")

                if (list_area[count_stop] == "1" ):
                    pos.x = -28.5
                    pos.y = 51.8

                elif (list_area[count_stop] == "2" ):
                    pos.x = -21.5
                    pos.y = 51.3

                elif (list_area[count_stop] == "3" ):
                    pos.x = -28.2
                    pos.y = 46.3

                elif (list_area[count_stop] == "4" ):
                    pos.x = -21.5
                    pos.y = 46.3

                elif (list_area[count_stop] == "5" ):
                    pos.x = -29.5
                    pos.y = 41.3

                elif (list_area[count_stop] == "6" ):
                    pos.x = -22.2
                    pos.y = 41.3
                
                else:
                    print("error!!!")
                    pos.x = 0
                    pos.y = 40

                pos.z = pos.z
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = pos.yaw
                action = 8
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both
                break

            if case(8):
                print("---put the puzzle down---")
                pos.x = pos.x
                pos.y = pos.y
                pos.z = star_config.down_z
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = pos.yaw
                action = 9
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both
                break

            if case(9):
                print("---finish round: %d---"%(count_stop+1)) 
                action = 10  
                break


            if case(10):
                print("---finish round: %d---"%(count_stop+1)) 
                sent_sucker_signal("off") 
                sent_sucker_signal("off") 
                pos.x = pos.x
                pos.y = pos.y
                pos.z = star_config.yolo_z
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = pos.yaw
                action = 0
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,30,2)#action,ra,grip,vel,both
                count_stop  = count_stop +1
                break     
     

            if case(): # default, could also just omit condition or 'if True'
                rospy.on_shutdown(myhook)
                ArmTask.rospy.on_shutdown(myhook)


    #action: ptp line
    #ra : abs rel
    #grip 夾爪
    #vel speed
    #both : Ctrl_Mode
##-------------strategy end ------------
def myhook():
    print ("shutdown time!")

if __name__ == '__main__':
    argv = rospy.myargv()
    rospy.init_node('strategy', anonymous=True)
    GetInfoFlag = True #Test no data
    arm_state_listener()

    rospy.Subscriber("raspberry_receive",String,callback_receive_sucker)

    while 1:
        start_input = int(input('開始策略請按1 ,測試吸盤請按2 ,離開請按3 : ')) #輸入開始指令

        if start_input == 1:
            while 1:
                #time.sleep(0.05) #0710 最穩定 delay 0.1秒
                time.sleep(0.1) #0710 最穩定 delay 0.1秒
                Mission_strategy()
                #input()
                
                if count_stop == 6:
                    print(" Done! you can stop~")
                    break

        if start_input == 2:
            while 1:
                time.sleep(0.2)
                test_arduino_func()
                if count_stop == 3:
                    break

        if start_input == 3:
            break

        count_stop = 0

    ArmTask.rospy.spin()
    rospy.spin()
