"""
11.26晚：放在zero的程序，暂未添加GRN
11.27: 加入对其他机器人的信息接收和距离测算
11.28: +圆周运动
12.03: 加中介，修改了socket接收刚提的方式
12.04：加入Local GRN浓度强度计算
       加入机器人根据浓度场强度运动策略
12.12：1.修改障碍物的探寻策略，原本以机器人到障碍物的安全距离和机器人的到目标的距离之和作为是否将障碍物纳入考虑的标准，
        现在直接改成，找机器人范围内最近的机器人；2.定位GRN效果不好的原因，主要在于预测点算错;3.添加摄像头;4.添加指示灯
12.15：1.添加用摄像头找目标的方法；2.修改teammate_avoidance, 不再判断机器人当前的yaw角是不是朝向目标的±90°
12.16：1.修改摄像头找目标的方法：找两次目标，选最近的
111
"""

import time
import socket
import logging
import math
import struct
from threading import Thread
import numpy as np
import random
import uuid
import  copy
import pickle
import cv2

class Camera():
    def __init__(self):
        self.size=(120,90)
        self.radius = 0
        self.center = (0,0)
        self.aim_p=False
        self.done = False # 摄像头完成任务

        self.h_min = 100  # cv2.getTrackbarPos("Hue Min","TrackBars")
        self.h_max = 124  # cv2.getTrackbarPos("Hue Max", "TrackBars")
        self.s_min = 43  # cv2.getTrackbarPos("Sat Min", "TrackBars")
        self.s_max = 255  # cv2.getTrackbarPos("Sat Max", "TrackBars")
        self.v_min = 80  # cv2.getTrackbarPos("Val Min", "TrackBars")
        self.v_max = 255  # cv2.getTrackbarPos("Val Max", "TrackBars")

    def detector(self):
        cap = cv2.VideoCapture(0)

        while (cap.isOpened()):
            # print("cam opened")
            ret, img = cap.read()

            img = cv2.resize(img, self.size)
            frame = img.copy()
            imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # 调用回调函数，获取滑动条的值
            lower = np.array([self.h_min, self.s_min, self.v_min])
            upper = np.array([self.h_max, self.s_max, self.v_max])
            # 获得指定颜色范围内的掩码
            mask = cv2.inRange(imgHSV, lower, upper)
            # 寻找图中轮廓
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            radius=0
            center=(0,0)
            if len(cnts)>0:
                # 找到面积最大的轮廓
                c = max(cnts, key=cv2.contourArea)
                # 寻找凸包并绘制凸包（轮廓）
                #hull = cv2.convexHull(c)
                # print(len(hull))
                # length = len(hull)
                # if length > 5:
                #     # 绘制图像凸包的轮廓
                #     for i in range(length):
                #         cv2.line(img, tuple(hull[i][0]), tuple(hull[(i + 1) % length][0]), (0, 0, 255), 2)
                # cv2.imshow('finger', img)
                # 使用最小外接圆圈出面积最大的轮廓
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                # 计算轮廓的矩
                M = cv2.moments(c)
                # 计算轮廓的重心
                center = (int(M["m10"] / (M["m00"]+1e-07)), int(M["m01"] / (M["m00"]+1e-07)))
                aim_p = False
                # 只处理尺寸足够大的轮廓
                if radius > 3:
                    aim_p=True
                # print(aim_p,center)
            # 对原图图像进行按位与的操作，掩码区域保留
            #imgResult = cv2.bitwise_and(img, img, mask=mask)
                self.aim_p = aim_p
                self.radius = radius
                self.center = center

                if  self.done:
                    cap.release()

                    break
            # return self.aim_p,self.radius,self.center
        #return frame,mask,imgResult
        # return self.aim_p
        print("cam dead")

class Epuck():
    def __init__(self, robot_addr):
        self.COMMAND_PACKET_SIZE = 21  # 指令包的大小 1个字节包头0x80和20个字节数据
        self.HEADER_PACKET_SIZE = 1  # 头文件大小1
        #https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development#WiFi_2
        self.SENSORS_PACKET_SIZE = 104  # 传感器包大小
        self.MAX_NUM_CONN_TRIALS = 5  # 最大连接次数
        self.command = bytearray([0] * self.COMMAND_PACKET_SIZE)
        self.start_time = time.time()
        self.epuck_port = 1000  # TCP端口值
        self.epuck_addr = robot_addr
        self.SENS_THRESHOLD = 800
        self.avoiding_flag = False
        self.proximity = [0 for x in range(8)]
        self.Command_Init()

    ### 连接epcuk
    def Connect(self):
        trials = 0
        self.socket_error = 0  # 通信报错
        print("Try to connect to " + self.epuck_addr + ":" + str(self.epuck_port))
        while trials < self.MAX_NUM_CONN_TRIALS:
            self.epuck_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.epuck_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.epuck_socket.settimeout(10)  # 无阻塞连接超时

            try:
                self.epuck_socket.connect((self.epuck_addr,self.epuck_port))
            except socket.timeout as err:
                self.epuck_socket.close()
                logging.error("Error from Epuck Connect " + self.epuck_addr + ":")
                logging.error(err)
                trials += 1
                continue
            except socket.error as err:
                self.epuck_socket.close()
                logging.error("Error from Epuck Connect " + self.epuck_addr + ":")
                logging.error(err)
                trials += 1
                continue
            except Exception as err:
                self.epuck_socket.close()
                logging.error("Error from Epuck Connect " + self.epuck_addr + ":")
                logging.error(err)
                trials += 1
                continue
            break

        if(trials == self.MAX_NUM_CONN_TRIALS):
            print("Can't connect to " + self.epuck_addr)
            return

        print("Connected to " + self.epuck_addr)

    ### 对发送给epuck的command包进行初始化
    def Command_Init(self):
        self.command[0] = 0x80  # 设置执行器的数据包ID
        self.command[1] = 2  # 要求：仅启用传感器,
        self.command[2] = 0  # 设置：设置速度
        self.command[3] = 0  # 左马达LSB
        self.command[4] = 0  # 左马达MSB
        self.command[5] = 0  # 右马达LSB
        self.command[6] = 0  # 右马达MSB
        self.command[7] = 0x00  # 发光二极管
        self.command[8] = 0  # LED2 red
        self.command[9] = 0  # LED2 green
        self.command[10] = 0  # LED2 blue
        self.command[11] = 0  # LED4 red
        self.command[12] = 0  # LED4 green
        self.command[13] = 0  # LED4 blue
        self.command[14] = 0  # LED6 red
        self.command[15] = 0  # LED6 green
        self.command[16] = 0  # LED6 blue
        self.command[17] = 0  # LED8 red
        self.command[18] = 0  # LED8 green
        self.command[19] = 0  # LED8 blue
        self.command[20] = 0  # speaker

    ### 接受epuck的数据
    def Receive(self, s, msg_len):
        chunks = []  # 创建接受数组
        bytes_recd = 0
        while bytes_recd < msg_len:
            chunk = s.recv(min(msg_len - bytes_recd, 2048))  # 判断是否溢出2048大小
            if chunk == b'':
                raise RuntimeError("Receive error")
            chunks.append(chunk)  # 添加
            bytes_recd = bytes_recd + len(chunk)  #
        return b''.join(chunks)

    ### 向epuck发送数据
    def Send(self,s, msg, msg_len):
        total_sent = 0
        while total_sent < msg_len:
            sent = s.send(msg[total_sent:])  # 尝试发送所有
            if sent == 0:
                raise RuntimeError("Send error")
            total_sent = total_sent + sent  # 判断是否发送结束

    def Contact(self):
        # 向机器人发送命令。
        self.socket_error = 0
        prox = [0 for x in range(8)]
        try:
            self.Send(self.epuck_socket, self.command, self.COMMAND_PACKET_SIZE)  # 发送包大小
        except socket.timeout as err:
            logging.error("Error from Epuck send" + self.epuck_addr + ":")
            logging.error(err)
            self.socket_error = 1
        except socket.error as err:
            logging.error("Error from Epuck send" + self.epuck_addr + ":")
            logging.error(err)
            self.socket_error = 1
        except Exception as err:
            logging.error("Error from Epuck send" + self.epuck_addr + ":")
            logging.error(err)
            self.socket_error = 1

        # 根据完成的请求设置要接收的预期数据包数。

        try:
            header = self.Receive(self.epuck_socket, self.HEADER_PACKET_SIZE)  # 尝试接受第一个字节
            # print("header=" + str(header[0]))
        except socket.timeout as err:
            logging.error("Error from Epuck receive " + self.epuck_addr + ":")
            logging.error(err)
            self.socket_error = 1
        except socket.error as err:
            logging.error("Error from Epuck receive " + self.epuck_addr + ":")
            logging.error(err)
            self.socket_error = 1
        except Exception as err:
            logging.error("Error from Epuck receive " + self.epuck_addr + ":")
            logging.error(err)
            self.socket_error = 1
        # print("self.command:", self.command)
        if header == bytearray([2]):
            try:
                sensor = self.Receive(self.epuck_socket, self.SENSORS_PACKET_SIZE)
            except socket.timeout as err:
                logging.error("Error from Epuck receive " + self.epuck_addr + ":")
                logging.error(err)
                self.socket_error = 1
            except socket.error as err:
                logging.error("Error from Epuck receive " + self.epuck_addr + ":")
                logging.error(err)
                self.socket_error = 1
            except Exception as err:
                logging.error("Error from Epuck receive" + self.epuck_addr + ":")
                logging.error(err)
                self.socket_error = 1
            # if(self.socket_error == 1):
            #     print("socket error!")
            #     self.Contact()

            prox[0] = sensor[37] + sensor[38]*256
            prox[1] = sensor[39] + sensor[40] * 256
            prox[2] = sensor[41] + sensor[42] * 256
            prox[3] = sensor[43] + sensor[44] * 256
            prox[4] = sensor[45] + sensor[46] * 256
            prox[5] = sensor[47] + sensor[48] * 256
            prox[6] = sensor[49] + sensor[50] * 256
            prox[7] = sensor[51] + sensor[50] * 256
            return prox



    def Get_prox(self):
        contact_times = 10
        total_prox = [0 for i in range(8)]
        while (contact_times):
            prox = self.Contact()
            if contact_times > 2 and contact_times < 7:
                for i in range(8):
                    total_prox[i] += prox[i]
            # print(self.command[4], self.command[3])
            contact_times -= 1
        for a in range(8):
            self.proximity[a] = total_prox[a] / 4
        # print(self.proximity[0])
        # print(self.proximity[1])
        # print(self.proximity[6])
        # print(self.proximity[7])

    def IR_detect(self):
            # self.Contact()
        self.Get_prox()

        while (self.proximity[0]>=self.SENS_THRESHOLD) or (self.proximity[1]>=self.SENS_THRESHOLD) or (self.proximity[6]>=self.SENS_THRESHOLD) or (self.proximity[7]>=self.SENS_THRESHOLD):
            print("avoiding\n")
            self.avoiding_flag = True
            if (self.proximity[1]>=self.SENS_THRESHOLD) and (self.proximity[6]>=self.SENS_THRESHOLD):
                self.Update_Rotation(180)
            elif (self.proximity[0]>=self.SENS_THRESHOLD) or (self.proximity[1]>=self.SENS_THRESHOLD):
                self.Update_Rotation(-30)
            elif (self.proximity[6]>=self.SENS_THRESHOLD) or (self.proximity[7]>=self.SENS_THRESHOLD):
                self.Update_Rotation(30)
            elif (self.proximity[2]>=self.SENS_THRESHOLD) or (self.proximity[5]>=self.SENS_THRESHOLD) and (self.proximity[0]>=self.SENS_THRESHOLD) and (self.proximity[1]>=self.SENS_THRESHOLD)  and (self.proximity[6]>=self.SENS_THRESHOLD) and (self.proximity[7]>=self.SENS_THRESHOLD):
                self.Update_Forward(30)    
            self.Get_prox()



    def Update_Forward(self, distance):
        wheel_diameter = 41  # 轮子直径41mm
        speed = 400
        contact_times = 3
        self.start_time = time.time()
        forward_step = distance / (math.pi * wheel_diameter) * 1000  # 要走的距离除以轮子周长，再乘以轮子转一圈的步数
        while(abs(time.time() - self.start_time) < (forward_step/speed)):
            # print("GO!")
            self.command[4] = self.command[6] = int(speed // 256)
            self.command[3] = self.command[5] = int(speed % 256)
            self.Contact()

        # print("STOP!")
        self.command[4] = self.command[6] = 0
        self.command[3] = self.command[5] = 0

        while(contact_times):
            self.Contact()
            # print(self.command[4],self.command[3])
            contact_times -= 1

    def Update_Rotation(self,angel):
        wheel_diameter = 41  # 轮子直径41mm
        rotation_diameter = 53  # 自转直径53mm
        speed = 400
        contact_times = 3
        total_prox = [0 for i in range(8)]
        self.start_time = time.time()
        # rotation_step = ((angel/360)*math.pi*rotation_diameter)/(math.pi*wheel_diameter)*1000 # 自转圆弧的长度除以轮子周长，再乘以轮子转一圈的步数
        rotation_step = (abs(angel) * rotation_diameter) / (wheel_diameter * 360) * 1000 # 化简
        while (abs(time.time() - self.start_time) < (rotation_step / speed)):
            # print("SPIN!")
            if angel > 0:
                self.command[4] = int(speed // 256)
                self.command[3] = int(speed % 256)
                self.command[6] = int((65535-speed) // 256)
                self.command[5] = int((65535-speed) % 256)
            else:
                self.command[4] = int((65535-speed) // 256)
                self.command[3] = int((65535-speed)  % 256)
                self.command[6] = int(speed // 256)
                self.command[5] = int(speed % 256)
            self.Contact()

        # print("STOP!")
        self.command[4] = self.command[6] = 0
        self.command[3] = self.command[5] = 0

        while(contact_times):
            self.Contact()
            # print(self.command[4],self.command[3])
            contact_times -= 1

    def Rotation(self,dir):

        speed = 200


        # rotation_step = ((angel/360)*math.pi*rotation_diameter)/(math.pi*wheel_diameter)*1000 # 自转圆弧的长度除以轮子周长，再乘以轮子转一圈的步数

            # print("SPIN!")
        if dir > 0:
            self.command[4] = int(speed // 256)
            self.command[3] = int(speed % 256)
            self.command[6] = int((65535-speed) // 256)
            self.command[5] = int((65535-speed) % 256)
        else:
            self.command[4] = int((65535-speed) // 256)
            self.command[3] = int((65535-speed)  % 256)
            self.command[6] = int(speed // 256)
            self.command[5] = int(speed % 256)
        self.Contact()
        self.Contact()
        self.Contact()


    def Update_Circular(self,distance,direction):
        wheel_diameter = 41  # 轮子直径41mm
        speed = 600
        contact_times = 3
        outer_speed = speed
        inner_speed = outer_speed*(distance-20)/(distance+20)
        if direction == 1:
            self.command[4] = int(outer_speed // 256)
            self.command[3] = int(outer_speed % 256)
            self.command[6] = int(inner_speed // 256)
            self.command[5] = int(inner_speed % 256)
        elif direction == -1:
            self.command[4] = int(inner_speed // 256)
            self.command[3] = int(inner_speed % 256)
            self.command[6] = int(outer_speed // 256)
            self.command[5] = int(outer_speed % 256)

        self.Contact()

    def Set_RGB_LEDS(self,a,R,G,B):
        self.command[7] = a  # LED2 red
        self.command[8] = R  # LED2 red
        self.command[9] = G  # LED2 green
        self.command[10] = B  # LED2 blue
        self.command[11] = R  # LED4 red
        self.command[12] = G  # LED4 green
        self.command[13] = B  # LED4 blue
        self.command[14] = R  # LED6 red
        self.command[15] = G  # LED6 green
        self.command[16] = B  # LED6 blue
        self.command[17] = R  # LED8 red
        self.command[18] = G  # LED8 green
        self.command[19] = B  # LED8 blue
        # print(self.command)
        self.Contact()
        self.Contact()
        self.Contact()

    def Stop(self):
        self.command[4] = 0
        self.command[3] = 0
        self.command[6] = 0
        self.command[5] = 0
        self.Contact()  # 停下

class Talk_To_Motive():
    def __init__(self):
        self.motive_addr = "192.168.3.27"
        self.motive_port = 10005
        self.DETECT_TEAMMATE_RANGE = 130
        self.epuck = Epuck(zero_robot_IP[mac])
        self.rigidbody = {"ID": 0, "ax":0.0, "ay": 0.0, "yaw":0.0} # 自身刚体信息
        self.target = {"ID":0, "ax": 0.0, "ay": 0.0, "yaw":0.0} # 目标刚体信息
        self.targets = []
        self.teammate_rigidbodies = [] # teammate刚体列表
        self.obs_rigidbodies = [] # 障碍物刚体列表
        self.PREDICT_STEP = 80 # 用于预测下一步GRN模型浓度值的距离步长
        self.epuck.Connect()
        self.motive_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.OBS_DETECT_DISTANCE = 400 # 机器人检测障碍物的范围
        self.SAFE_SELF_TARGET = 250

    def Connect(self):
        try:
            print("try to connect motive")
            self.motive_socket.connect((self.motive_addr, self.motive_port))
            print("Connected to motive")
        except socket.timeout as err:
            self.motive_socket.close()
            logging.error("Error from Motive Connect " + self.motive_addr + ":")
            logging.error(err)
        except socket.error as err:
            self.motive_socket.close()
            logging.error("Error from Motive Connect " + self.motive_addr + ":")
            logging.error(err)
        except Exception as err:
            self.motive_socket.close()
            logging.error("Error from Motive Connect " + self.motive_addr + ":")
            logging.error(err)

    def Receive(self):  # to be modified
        # 获取刚体的数据
        while True:
            recv_rigidbodies = self.motive_socket.recv(1024)

            try:
                rigidbodies = pickle.loads(recv_rigidbodies)  # 解包
            except:
                self.motive_socket.send(pickle.dumps("got er"))  # 向服务端发送反馈，收到不正确的刚体长度信息
                print("pickle loads error")
                continue
            self.motive_socket.send(pickle.dumps("got it"))
            # print("rigidbodies:",rigidbodies)
            self_rigidbody = {"ID": 0, "ax": 0.0, "ay": 0.0, "yaw": 0.0}
            targets = []
            teammates = []
            obss = []
            for i in rigidbodies:
                if i["ID"] == zero_robot_ID[mac]:
                    self_rigidbody["ID"] = i["ID"]
                    self_rigidbody["ax"] = i["ax"]
                    self_rigidbody["ay"] = i["ay"]
                    self_rigidbody["yaw"] = -i["yaw"]
                elif i["ID"] == 4630 or i["ID"] == 5579 :
                    target = {"ID": i["ID"], "ax": i["ax"], "ay": i["ay"], "yaw": -i["yaw"]}
                    targets.append(target)
                elif i["ID"] > 4000 and i["ID"] < 6000:
                    teammate = {"ID": i["ID"], "ax": i["ax"], "ay": i["ay"], "yaw": -i["yaw"]}
                    teammates.append(teammate)
                else:
                    obs = {"ID": i["ID"], "ax": i["ax"], "ay": i["ay"], "yaw": -i["yaw"]}
                    obss.append(obs)

            self.rigidbody = self_rigidbody  # 自身刚体信息
            self.targets = targets  # 目标刚体信息
            self.teammate_rigidbodies = teammates  # teammate刚体列表
            self.obs_rigidbodies = obss  # 障碍物刚体列表
            # print("本机：", self.rigidbody)
            # print("target:",self.target)
            # print("teammate：", self.teammate_rigidbodies)

    def Sigmoid(self, p, theta, k):
        return 1 / (1 + np.exp(-k * (p - theta)))


    # 不考虑邻居的
    def Local_GRN(self, self_rigidbody, target_rigidbody, obs_rigidbody):
        theta = 0.22
        k = 1

        scale_in_phy = 2000 # 实机场景中的尺度范围
        scale_in_sim = 25 # 仿真场景中的尺度范围
        scale_phy_to_sim = scale_in_phy / scale_in_sim # 从实机场景到仿真场景的变换

        # 改变坐标尺度，从真实场景的2000*2000改到25*25
        self_coor_in_sim = [self_rigidbody["ax"]/scale_phy_to_sim, self_rigidbody["ay"]/scale_phy_to_sim] # get the coordinate of robot itself in simulation
        target_coor_in_sim = [target_rigidbody["ax"]/scale_phy_to_sim, target_rigidbody["ay"]/scale_phy_to_sim]  # get the coordinate of target in simulation

        self_to_target = math.sqrt(((self_coor_in_sim[0]-0.125) - target_coor_in_sim[0])**2 + ((self_coor_in_sim[1]-0.125) - target_coor_in_sim[1])**2)
        c_self_to_target = math.exp(-1 * self_to_target)

        if obs_rigidbody != None:
            obs_coor_in_sim = [obs_rigidbody["ax"] / scale_phy_to_sim, obs_rigidbody["ay"] / scale_phy_to_sim]  # get the coordinate of obstacle in simulation
            self_to_obs = math.sqrt(((self_coor_in_sim[0]-0.125) - obs_coor_in_sim[0])**2 + ((self_coor_in_sim[1]-0.125) - obs_coor_in_sim[1])**2)
            c_self_to_obs = math.exp(-1 * self_to_obs)
            # print("c_self_to_target:",(c_self_to_target),"c_self_to_obs:",(c_self_to_obs))
            # print("self_to_target_sig:",self.Sigmoid(1-c_self_to_target**2, theta, k),"self_to_obs_sig:",self.Sigmoid(c_self_to_obs**2, theta, k))
            c = self.Sigmoid(1-c_self_to_target**2, theta, k) + self.Sigmoid(c_self_to_obs**2, theta, k)
        else:
            c = self.Sigmoid(1 - c_self_to_target ** 2, theta, k)

        return c


    def Cal_self_target_distance(self,self_rigidbody, target_rigidbody):
        return math.sqrt((self_rigidbody["ax"] - target_rigidbody["ax"]) ** 2 + (
                self_rigidbody["ay"] - target_rigidbody["ay"]) ** 2)  # 目标距离

    def Cal_self_obs_distance(self, self_rigidbody, obs_rigidbodies): # should take the case of only has one rigidbody into consider?
        self_obs_distance = [0 for i in range(len(obs_rigidbodies))]
        for i in range(len(obs_rigidbodies)):
            self_obs_distance[i] = math.sqrt((self_rigidbody["ax"] - obs_rigidbodies[i]["ax"]) ** 2 + (
                self_rigidbody["ay"] - obs_rigidbodies[i]["ay"]) ** 2)  # 目标距离
        return self_obs_distance

    def Cal_target_obs_distance(self, target_rigidbody, obs_rigidbodies): # should take the case of only has one rigidbody into consider?
        target_obs_distance = [0 for i in range(len(obs_rigidbodies))]
        for i in range(len(obs_rigidbodies)):
            target_obs_distance[i] = math.sqrt((target_rigidbody["ax"] - obs_rigidbodies[i]["ax"]) ** 2 + (
                target_rigidbody["ay"] - obs_rigidbodies[i]["ay"]) ** 2)  # 目标距离
        return target_obs_distance

    def Cal_self_teammate_distance(self, self_rigidbody, teammate_rigidbodies): # should take the case of only has one rigidbody into consider?
        self_teammate_distance = []
        for i in range(len(teammate_rigidbodies)):
            self_teammate_distance[i] = math.sqrt((self_rigidbody["ax"] - teammate_rigidbodies[i]["ax"]) ** 2 + (
                self_rigidbody["ay"] - teammate_rigidbodies[i]["ay"]) ** 2)  # 目标距离
        return self_teammate_distance

    def Cal_target_teammate_distance(self, target_rigidbody, teammate_rigidbodies): # should take the case of only has one rigidbody into consider?
        target_teammate_distance = []
        for i in range(len(teammate_rigidbodies)):
            target_teammate_distance[i] = math.sqrt((target_rigidbody["ax"] - teammate_rigidbodies[i]["ax"]) ** 2 + (
                target_rigidbody["ay"] - teammate_rigidbodies[i]["ay"]) ** 2)  # 目标距离
        return target_teammate_distance

    def Cal_predict_self_target_distance(self, turn_angel, self_rigidbody, target_rigidbody):
        return math.sqrt((self_rigidbody["ax"] + math.sin(self.caclul_angle(self_rigidbody["yaw"], turn_angel) / 180 * math.pi) * self.PREDICT_STEP - target_rigidbody["ax"]) ** 2 + (
                    self_rigidbody["ay"] + math.cos(self.caclul_angle(self_rigidbody["yaw"], turn_angel) / 180 * math.pi) * self.PREDICT_STEP - target_rigidbody["ay"]) ** 2) #左转30°目标距离

    def Cal_predict_self_obs_distance(self, turn_angel, self_rigidbody, obs_rigidbodies):
        predict_self_obs_distance = []
        for i in range(len(obs_rigidbodies)):
            predict_self_obs_distance[i] = math.sqrt((self_rigidbody["ax"] + math.sin(self.caclul_angle(self_rigidbody["yaw"], turn_angel) / 180 * math.pi) * self.PREDICT_STEP - obs_rigidbodies[i]["ax"]) ** 2 + (
                    self_rigidbody["ay"] + math.cos(self.caclul_angle(self_rigidbody["yaw"], turn_angel) / 180 * math.pi) * self.PREDICT_STEP - obs_rigidbodies[i]["ay"]) ** 2)
        return predict_self_obs_distance

    def caclul_angle(self,yaw,deta_angle):
        ans_angle = yaw + deta_angle
        if ans_angle > 180:
            ans_angle = ans_angle - 360
        if ans_angle < -180:
            ans_angle = ans_angle +360
        return ans_angle

    def Predict_rigidbody(self, predict_angle, self_rigidbody):
        predict_rigidbody = copy.copy(self_rigidbody)

        if predict_angle<0:
            predict_angle+=360
        predict_rigidbody["ax"] = self_rigidbody["ax"] + math.sin(self.caclul_angle(self_rigidbody["yaw"],predict_angle) / 180 * math.pi) * self.PREDICT_STEP
        predict_rigidbody["ay"] = self_rigidbody["ay"] + math.cos(self.caclul_angle(self_rigidbody["yaw"],predict_angle) / 180 * math.pi) * self.PREDICT_STEP

        return predict_rigidbody

    def Go_with_C_grad(self):  # 根据浓度走

        while True:
            # print("in GRN")
            self.Avoiding_with_teammate() # 如果前面有邻居，先进行避碰

            this_time_obss = self.obs_rigidbodies
            this_time_rigidbody = self.rigidbody
            # self.target = self.Choose_Target(this_time_rigidbody, self.targets)
            this_time_target = self.target
            theta = (math.atan2((this_time_target["ax"] - this_time_rigidbody["ax"]),
                                (this_time_target["ay"] - this_time_rigidbody["ay"]))) * 180 / math.pi
            if theta > this_time_rigidbody["yaw"] + 10 or theta < this_time_rigidbody["yaw"] - 10:
                turn_angel = self.caclul_angle(theta - this_time_rigidbody["yaw"])
                self.epuck.Update_Rotation(turn_angel)
                this_time_rigidbody = self.rigidbody

            if self.Cal_self_target_distance(this_time_rigidbody, this_time_target) > self.SAFE_SELF_TARGET:  # 机器人没到目标附近的阈值
                ## 计算目标到所有障碍物的距离
                print("here")
                if this_time_obss != []:  # 程序一开始可能还没来得及接收到刚体的数据
                    # print("this_time_obss:",this_time_obss)
                    # print("this_time_target:",this_time_target)

                    ## Local GRN只有一个障碍物的输入，选择离机器人最近的障碍物
                    self_obss_distance = self.Cal_self_obs_distance(this_time_rigidbody,this_time_obss)
                    # print("self_obss_distance:",self_obss_distance)
                    if all(self_obss_distance[i] > self.OBS_DETECT_DISTANCE for i in range(len(self_obss_distance))):
                        chosen_obs = None
                    else:
                        chosen_obs_index = self_obss_distance.index(min(self_obss_distance))
                        chosen_obs = this_time_obss[chosen_obs_index]
                        self.epuck.Set_RGB_LEDS(0,50,50,0)
                    print("chosen_obs:",chosen_obs)


                    this_time_C = self.Local_GRN(this_time_rigidbody, this_time_target, chosen_obs)
                    print("this_time_self_C", this_time_C)
                    # 预测不同方向的下一步的浓度值
                    predict_c_angel = [-45, -30, -15, 0, 15, 30, 45]
                    predict_c = [0 for i in range(len(predict_c_angel))]  # 浓度
                    predict_c_differ = [0 for i in range(len(predict_c_angel))]

                    # print(this_time_rigidbody,this_time_target,chosen_obs)
                    for i in range(len(predict_c_angel)):  # 计算预测的三个点到目标的距离
                        predict_rigidbody = self.Predict_rigidbody(predict_c_angel[i], this_time_rigidbody)
                        # predict_self_target_distance[i] = self.Cal_predict_self_target_distance(predict_c_angel[i], this_time_rigidbody, this_time_target)
                        # pridict_self_obs_distance[i] = self.Cal_predict_self_obs_distance(predict_c_angel[i], this_time_rigidbody, detected_obss)
                        predict_c[i] = self.Local_GRN(predict_rigidbody, this_time_target, chosen_obs)
                        # print("NO.",i,"predict_rigidbody:",predict_rigidbody["ax"],predict_rigidbody["ay"],"predict_c:",predict_c[i])
                        predict_c_differ[i] = predict_c[i] - this_time_C

                    best_choice = predict_c_differ.index(min(predict_c_differ))

                    self.epuck.Update_Rotation(predict_c_angel[best_choice])
                    self.epuck.Update_Forward(self.PREDICT_STEP)
            else:
                self.epuck.Set_RGB_LEDS(0, 99, 99, 99)

    def Avoiding_with_teammate(self):
        this_time_teammate = self.teammate_rigidbodies
        this_time_rigidbody = self.rigidbody
        this_time_target = self.target
        teammate_distances = [1000 for i in range(len(this_time_teammate))]

        for i in range(len(this_time_teammate)):  ## 计算所有邻居的距离
            teammate_distances[i] = math.sqrt((this_time_rigidbody["ax"] - this_time_teammate[i]["ax"]) ** 2 + (
                    this_time_rigidbody["ay"] - this_time_teammate[i]["ay"]) ** 2)
        target_distance = math.sqrt((this_time_rigidbody["ax"] - this_time_target["ax"]) ** 2 + (
                this_time_rigidbody["ay"] - this_time_target["ay"]) ** 2)  # 目标距离

        while (not all(teammate_distances[i] > self.DETECT_TEAMMATE_RANGE for i in range(len(teammate_distances)))):
            self.epuck.Set_RGB_LEDS(0, 0, 99, 0)
            left_robot = []
            right_robot = []

            ############# 用来判断向哪边转#########################
            for i in range(len(this_time_teammate)):  # 基于自身，判断所有机器人在自身左边还是右边
                if teammate_distances[i] < self.DETECT_TEAMMATE_RANGE:
                    theta = (math.atan2((this_time_teammate[i]["ax"] - this_time_rigidbody["ax"]),
                                        (this_time_teammate[i]["ay"] - this_time_rigidbody["ay"]))) * 180 / math.pi
                    # print("theta:", theta)
                    if this_time_rigidbody["yaw"] > 90:  # 在90到180之间
                        if (this_time_rigidbody["yaw"] - 90) <= theta < this_time_rigidbody[
                            "yaw"]:  # 跟邻居的夹角在机器人的yaw-90和yaw之间，就是在机器人的左边
                            left_robot.append(this_time_teammate[i])
                        if (this_time_rigidbody["yaw"] < theta < 180) or (
                                -180 < theta < this_time_rigidbody["yaw"] - 270):
                            right_robot.append(this_time_teammate[i])
                    elif this_time_rigidbody["yaw"] < -90:
                        if this_time_rigidbody["yaw"] <= theta < this_time_rigidbody["yaw"] + 90:
                            right_robot.append(this_time_teammate[i])
                        if (this_time_rigidbody["yaw"] + 270 <= theta < 180) or (
                                -180 <= theta < this_time_rigidbody["yaw"]):
                            left_robot.append(this_time_teammate[i])
                    else:
                        if this_time_rigidbody["yaw"] - 90 <= theta < this_time_rigidbody["yaw"]:
                            left_robot.append(this_time_teammate[i])
                        if this_time_rigidbody["yaw"] <= theta < this_time_rigidbody["yaw"] + 90:
                            right_robot.append(this_time_teammate[i])

            # print("right:", right_robot)
            # print("left:", left_robot)
            if len(right_robot) > 0 or len(left_robot) > 0:  # 先看范围内有没有邻居
                target_theta = (math.atan2((this_time_target["ax"] - this_time_rigidbody["ax"]), (
                            this_time_target["ay"] - this_time_rigidbody["ay"]))) * 180 / math.pi  # 自身与目标的夹角
                if len(right_robot) > len(left_robot):  # 顺时针
                    # print("clockwise")
                    # if not ((target_theta-100) < this_time_rigidbody["yaw"] < (target_theta-80)):
                    #     self.epuck.Update_Rotation(target_theta - this_time_rigidbody["yaw"] - 90)

                    print("rotate from teammate avoidance",
                          self.caclul_angle(self.caclul_angle(target_theta, -1 * this_time_rigidbody["yaw"]), -90))
                    self.epuck.Update_Rotation(
                        self.caclul_angle(self.caclul_angle(target_theta, -1 * this_time_rigidbody["yaw"]), -90))
                    # print("target_theta:", target_theta, "yaw:", this_time_rigidbody["yaw"], "turntheangle:",target_theta - 90 - this_time_rigidbody["yaw"])
                    # print("Avoid_Update_Rotation")
                    print("circle")
                    self.epuck.Update_Circular(target_distance, 1)
                    time.sleep(0.6)

                    # print("Avoid_Update_Circular")
                else:  # 逆时针
                    # print("anti-clockwise")
                    print("rotate from teammate avoidance",
                          self.caclul_angle(self.caclul_angle(target_theta, -1 * this_time_rigidbody["yaw"]), 90))
                    self.epuck.Update_Rotation(
                        self.caclul_angle(self.caclul_angle(target_theta, -1 * this_time_rigidbody["yaw"]), 90))

                    # if not ((target_theta + 80) < this_time_rigidbody["yaw"] < (target_theta + 100) ):
                    #     # print("turn")
                    #     self.epuck.Update_Rotation(target_theta - this_time_rigidbody["yaw"] + 90)
                    print("circle")
                    self.epuck.Update_Circular(target_distance, -1)
                    time.sleep(0.6)

            # self.epuck.IR_detect()# 还是要检查障碍物避障，但是在转圈的时候遇到障碍物会怎么样还有待测试 这里detect后要直行

            # 继续计算新时刻的距离
            this_time_teammate = self.teammate_rigidbodies
            this_time_rigidbody = self.rigidbody
            this_time_target = self.target
            teammate_distances = [1000 for i in range(len(this_time_teammate))]
            for i in range(len(this_time_teammate)):
                teammate_distances[i] = math.sqrt((this_time_rigidbody["ax"] - this_time_teammate[i]["ax"]) ** 2 + (
                        this_time_rigidbody["ay"] - this_time_teammate[i]["ay"]) ** 2)
            target_distance = math.sqrt((this_time_rigidbody["ax"] - this_time_target["ax"]) ** 2 + (
                    this_time_rigidbody["ay"] - this_time_target["ay"]) ** 2)

        # 停止圆周运动
        self.epuck.command[4] = 0
        self.epuck.command[3] = 0
        self.epuck.command[6] = 0
        self.epuck.command[5] = 0
        self.epuck.Contact()

    def Choose_Target(self, this_time_rigidbody, this_time_targets):#通过判断获得围捕的目标信息
        null_target = {"ID": 0, "ax":0.0, "ay": 0.0, "yaw":0.0}
        if this_time_targets == []:
            return null_target
        elif len(this_time_targets) == 1 :
            return this_time_targets[0]
        else:
            target_distance_0 = math.sqrt((this_time_rigidbody["ax"] - this_time_targets[0]["ax"]) ** 2 + (
                    this_time_rigidbody["ay"] - this_time_targets[0]["ay"]) ** 2)
            target_distance_1 = math.sqrt((this_time_rigidbody["ax"] - this_time_targets[1]["ax"]) ** 2 + (
                    this_time_rigidbody["ay"] - this_time_targets[1]["ay"]) ** 2)
            if target_distance_0 <= target_distance_1:
                # print("take ", this_time_targets[0]["ID"], " as target")
                return this_time_targets[0]
            else:
                # print("take ", this_time_targets[1]["ID"], " as target")
                return this_time_targets[1]

    def Move(self):
        # while self.epuck.avoiding == False: #没在避障的话

        while True:
            this_time_rigidbody = self.rigidbody
            this_time_targets = self.targets
            self.target = self.Get_Target(this_time_rigidbody, this_time_targets)
            this_time_target = self.target
            # print("rigidbody:",self.rigidbody)
            target_distance = math.sqrt((this_time_rigidbody["ax"] - this_time_target["ax"]) ** 2 + (
                        this_time_rigidbody["ay"] - this_time_target["ay"]) ** 2)
            #print("target_distance:",target_distance)
            if target_distance > 250: #跟目标的距离大于250mm
                print("self:", this_time_rigidbody)
                print("distance", target_distance)
                theta = (math.atan2((this_time_target["ax"]-this_time_rigidbody["ax"]) , (this_time_target["ay"]-this_time_rigidbody["ay"])))*180/math.pi
                print("theta", theta)
                print("Yaw:", this_time_rigidbody["yaw"])
                if theta > this_time_rigidbody["yaw"]+5 or theta < this_time_rigidbody["yaw"]-5:
                    turn_angel = theta - this_time_rigidbody["yaw"]
                    print("turn_angel", turn_angel)
                    self.epuck.Update_Rotation(turn_angel)
                self.epuck.IR_detect()
                self.Avoiding_with_teammate() # 避开邻居或者避开障碍物的行为是互斥的，但是障碍物的探知范围小于邻居的距离探知，所以避障的优先级要高于避邻居
                self.epuck.Update_Forward(30)# 每一步前进100mm

    def Random_motion(self):
        print("searching target")
        random_angel = random.randint(-90,90)
        print("random_angel:",random_angel)
        self.epuck.Update_Rotation(random_angel)
        self.epuck.Update_Forward(30)
        self.epuck.IR_detect() # 检测障碍物来走30mm

    # def Search_target_by_cam(self):
    #     cam = Camera()
    #     d = Thread(target=cam.detector)
    #     d.start()
    #     while True:
    #         motive.epuck.Update_Rotation(360)
    #         if cam.aim_p == True:
    #             self.epuck.command[4] = 0
    #             self.epuck.command[3] = 0
    #             self.epuck.command[6] = 0
    #             self.epuck.command[5] = 0
    #             self.epuck.Contact()
    #             while True:
    #                 print("center",cam.center)
    #                 if cam.center[0]<50:
    #                     motive.epuck.Update_Rotation(3)
    #                 elif cam.center[0] > 70:
    #                     motive.epuck.Update_Rotation(3)
    #                 else:
    #                     targets_theta = []
    #                     print(self.targets)
    #                     for i in self.targets:
    #                         theta = (math.atan2((i["ax"] - self.rigidbody["ax"]),
    #                                 (i["ay"] - self.rigidbody["ay"]))) * 180 / math.pi
    #                         targets_theta.append(theta)
    #                     if (targets_theta[0] - self.rigidbody["yaw"]) > (targets_theta[1] - self.rigidbody["yaw"]):
    #                         self.target = self.targets[1]
    #                     else:
    #                         self.target = self.targets[0]
    #                     print("chosen target:",self.target)
    #                     cam.done = True
    #                     break
    #             break
    def Target_detect(self):
        targets_theta = []
        # print(self.targets)
        for i in self.targets:  # 获取机器人到两个目标的角度
            theta = (math.atan2((i["ax"] - self.rigidbody["ax"]),
                                (i["ay"] - self.rigidbody["ay"]))) * 180 / math.pi
            targets_theta.append(theta)
        # print("self",self.rigidbody)
        # print("targets_theta",targets_theta,"yaw",self.rigidbody["yaw"])
        if (abs(targets_theta[0] - self.rigidbody["yaw"])) > abs((targets_theta[1] - self.rigidbody["yaw"])):
            detected_target = self.targets[1]
        else:
            detected_target = self.targets[0]
        return detected_target

    def Search_target_by_cam(self):
        cam = Camera()
        d = Thread(target=cam.detector)
        d.start()
        dir = random.randint(-1,1)
        while True: # 循环1
            self.epuck.Rotation(dir) # 开始自转
            if cam.aim_p == True: # 如果找到目标
                self.epuck.Stop()
                while True: # 循环2
                    # print("center",cam.center)
                    if cam.center[0]<50:
                        self.epuck.Update_Rotation(3)
                        time.sleep(0.5)
                    elif cam.center[0] > 70:
                        self.epuck.Update_Rotation(-3)
                        time.sleep(0.5)
                    else: # 调整使目标位于图像正中心
                        first_detected_target = self.Target_detect()
                        # print("first_detected_target",first_detected_target)
                        self.epuck.Rotation(dir)  # 继续自转
                        while cam.aim_p == True: pass # 先让机器人放弃检测当前的目标
                        while True: # 循环3
                            if cam.aim_p == True:  # 再次找到目标
                                self.epuck.Stop()
                                while True: # 循环4
                                    # print("center", cam.center)
                                    if cam.center[0] < 50:
                                        self.epuck.Update_Rotation(3)
                                        time.sleep(0.5)
                                    elif cam.center[0] > 70:
                                        self.epuck.Update_Rotation(-3)
                                        time.sleep(0.5)
                                    else:  # 调整使目标位于图像正中心
                                        second_detected_target = self.Target_detect()
                                        # print("second_detected_target",second_detected_target)
                                        break # 跳出循环4
                                break # 跳出循环3
                        break # 跳出循环2
                break # 跳出循环1

        cam.done = True # 摄像头完成任务
        if first_detected_target == second_detected_target:
            self.target = first_detected_target
        else:
            first_detected_target_distance = math.sqrt((self.rigidbody["ax"] - first_detected_target["ax"]) ** 2 + (
                        self.rigidbody["ay"] - first_detected_target["ay"]) ** 2)
            second_detected_target_distance = math.sqrt((self.rigidbody["ax"] - second_detected_target["ax"]) ** 2 + (
                        self.rigidbody["ay"] - second_detected_target["ay"]) ** 2)
            if first_detected_target_distance > second_detected_target_distance:
                self.target = second_detected_target
            else:
                self.target = first_detected_target
        print("chosen target:",self.target)

        self.Go_with_C_grad()












if __name__ == '__main__':
    zero_robot_IP = {"b827ebcb0084": "192.168.3.15", "b827ebee4886": "192.168.3.12", "b827eb01634f": "192.168.3.14","b827eb8ac5fc": "192.168.3.9", "b827eb5e2365": "192.168.3.18", \
                     "b827eb77d4e8": "192.168.3.50", "b827ebb9f857": "192.168.3.47", "b827ebe71122": "192.168.3.52","b827eb67cc3b": "192.168.3.49", "b827eb0d41c7": "192.168.3.48"}  # 树莓派地址:epuck地址
    zero_robot_ID = {"b827ebcb0084": 4663, "b827ebee4886": 4665, "b827eb01634f": 4697, "b827eb8ac5fc": 4723,"b827eb5e2365": 4761, \
                     "b827eb77d4e8": 5043, "b827ebb9f857": 4994, "b827ebe71122": 4840, "b827eb67cc3b": 4853,"b827eb0d41c7": 4688}  # 树莓派地址：epuckid
    #zero_robot_IP = {"192.168.3.47":"192.168.3.10"}
    #zero_robot_ID = {"192.168.3.47": 4649}# for test
    # local_ip = socket.gethostbyname(socket.gethostname())
    mac = uuid.UUID(int=uuid.getnode()).hex[-12:]
    # mac = "b827ebcb0084"
    print("id:",zero_robot_ID[mac])
    motive = Talk_To_Motive()
    # cam = Camera()
    # motive.Random_motion() # 随机游走
    motive.Connect()
    # motive.Random_motion()
    r = Thread(target=motive.Search_target_by_cam)
    r.start()

    # m = Thread(target=motive.Go_with_C_grad)
    # m.start()
    motive.Receive()












