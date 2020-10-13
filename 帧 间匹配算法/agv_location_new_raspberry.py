#!/usr/bin/env python
#coding=utf-8

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from xml_tools import *
from std_msgs.msg import String

Range_Min=0.2
Range_Max=5.0

Search_Dist=0.5

#局部坐标到全局坐标
def GetTransformC2W(px, py, offset_x, offset_y, theta):
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)

    R = np.array([[c_theta, -s_theta], [s_theta, c_theta]])

    t = np.array([[offset_x], [offset_y]])
    Pc = np.array([[px], [py]])

    Pw = np.dot(R, Pc) + t

    return Pw

def normalize_angle(angle):
    while angle>np.pi:
        angle-=2*np.pi
    while angle<-np.pi:
        angle+=2*np.pi
    return angle

#雷达数据帧类
class RadarFrame:
    def __init__(self, timestamp, angle, ranges):
        self.timestamp = timestamp
        self.angle = angle
        self.ranges = ranges
        self.pointcloud = []

        for i in range(self.angle.__len__()):        
            x_=ranges[i]*np.cos(angle[i])
            y_=ranges[i]*np.sin(angle[i])
            
            
            self.pointcloud.append(np.array([x_, y_]))

        self.breakpoint = self.FindBreakPoint(self.pointcloud) #找出该帧的所有分割点

        #把该帧依据分割点分成一簇一簇
        self.point_cluster=[]
        cluster=[]
        iBp=0
        
        if self.breakpoint.__len__()>0:
            for i in range(self.pointcloud.__len__()):
                if i == self.breakpoint[iBp]:
                    if len(cluster) > 10: self.point_cluster.append(cluster)
                    cluster=[]
                    iBp+=1
                    if iBp == self.breakpoint.__len__():
                        if self.pointcloud.__len__() - i >10: self.point_cluster.append(self.pointcloud[i+1:])
                        break
                else:
                    cluster.append(self.pointcloud[i])
        else:
            self.point_cluster.append(self.pointcloud)
                
                
        #从每个簇中获取角点
        self.corner_landmark=[]
        for iClu in self.point_cluster:
            self.corner_landmark += self.FindCorner(iClu)

    def pi_2_pi(self, val):
        return (val + np.pi) % (2 * np.pi) - np.pi
    

		#寻找角点算法
    def FindCorner(self,cluster):
        N = 4

        if cluster.__len__() < 2 * N + 1: return []

        corner_point = []
        for iPtr in range(cluster.__len__() - N):
            if iPtr < (N+3) or iPtr > cluster.__len__() - (N+3): continue
            xl_k = 0
            xr_k = 0
            yl_k = 0
            yr_k = 0
            for j in range(N + 1):
                xl_k += cluster[iPtr - j][0]
                yl_k += cluster[iPtr - j][1]
                xr_k += cluster[iPtr + j][0]
                yr_k += cluster[iPtr + j][1]
            xl_k = xl_k / (N + 1)
            xr_k = xr_k / (N + 1)
            yl_k = yl_k / (N + 1)
            yr_k = yr_k / (N + 1)

            l_a = np.sqrt((cluster[iPtr][0] - xr_k) ** 2 + (cluster[iPtr][1] - yr_k) ** 2)
            l_b = np.sqrt((cluster[iPtr][0] - xl_k) ** 2 + (cluster[iPtr][1] - yl_k) ** 2)
            
            
            
            
            l_c = np.sqrt((xr_k - xl_k) ** 2 + (yr_k - yl_k) ** 2)

            l = (l_a + l_b + l_c) / 2
            res = np.sqrt(l * (l - l_a) * (l - l_b) * (l - l_c)) / l_a / l_b
            
            if res>0.3: corner_point.append([iPtr, res])
            

        if corner_point.__len__() == 0: return []

        D=5
        corner=[]

        overflow=0
        for iCorn in range(corner_point.__len__()):
            if iCorn==0: corner.append(corner_point[iCorn])
            else:
                
                if corner_point[iCorn][0]-corner[-1][0] > D: 
                    if overflow>3: corner.pop()
                    
                    overflow=0
                    corner.append(corner_point[iCorn])
                else:
                    if corner_point[iCorn][1] > corner[-1][1]: 
                        corner[-1]=corner_point[iCorn]
                        overflow+=1
                        
                    

        res=[]
        for iCorn in corner:
            left_p1=cluster[iCorn[0]-N]
            left_p2=cluster[iCorn[0]-2]
            right_p1=cluster[iCorn[0]+2]
            right_p2=cluster[iCorn[0]+N]
            
            th_left=np.arctan2(left_p2[1]-left_p1[1],left_p2[0]-left_p1[0])
            th_right=np.arctan2(right_p2[1]-right_p1[1],right_p2[0]-right_p1[0])
            
            if np.abs(normalize_angle(th_right-th_left)) > np.deg2rad(60):                
                res.append(cluster[iCorn[0]][0:2].tolist())
            
        n=res.__len__()
            
        if n>2:
            to_leave_corner=[]
            
            for i in range(n):
                is_effect=True
                for jCorn in res:
                    dx=res[i][0]-jCorn[0]
                    dy=res[i][1]-jCorn[1]
                    dist=np.sqrt(dx**2+dy**2)
                    if dist<Search_Dist: 
                        is_effect=False
                        break
                
                if is_effect==True: to_leave_corner.append(res[i])
                
            
            return to_leave_corner
        elif n==2:
            dx=res[0][0]-res[1][0]
            dy=res[0][1]-res[1][1]
            dist=np.sqrt(dx**2+dy**2)
            if dist<Search_Dist: res=[]
            return res
                
        else: return res

		#寻找断点
    def FindBreakPoint(self, pcl):
        summe = len(pcl)
        max_dist = 0.15  # old:0.25

        breakpoint = []
        for i in range(summe):
            delta_x = pcl[i + 1][0] - pcl[i][0]
            delta_y = pcl[i + 1][1] - pcl[i][1]
            dist = np.sqrt(delta_x ** 2 + delta_y ** 2)
            if dist > max_dist: breakpoint.append(i)
            if i == summe - 2: break

        return breakpoint
    
 #雷达数据帧类
class RadarFrameLoc:
    def __init__(self, timestamp, angle, ranges, global_landmark, agv_pose):
        self.timestamp = timestamp
        self.angle = angle
        self.ranges = ranges
        self.pointcloud = []

        for i in range(self.angle.__len__()):
        
            x_=ranges[i]*np.cos(angle[i])
            y_=ranges[i]*np.sin(angle[i])
            
            
            self.pointcloud.append(np.array([x_, y_]))

        self.breakpoint = self.FindBreakPoint(self.pointcloud) #找出该帧的所有分割点

        #把该帧依据分割点分成一簇一簇
        self.point_cluster=[]
        cluster=[]
        iBp=0
        
        if self.breakpoint.__len__()>0:
            for i in range(self.pointcloud.__len__()):
                if i == self.breakpoint[iBp]:
                    if len(cluster) > 10: self.point_cluster.append(cluster)
                    cluster=[]
                    iBp+=1
                    if iBp == self.breakpoint.__len__():
                        if self.pointcloud.__len__() - i >10: self.point_cluster.append(self.pointcloud[i+1:])
                        break
                else:
                    cluster.append(self.pointcloud[i])
        else:
            self.point_cluster.append(self.pointcloud)
                
                
        #从每个簇中获取角点
        self.corner_landmark=[]
        for iClu in self.point_cluster:
            clu_pos_loc=self.get_clu_pos(iClu)
            clu_pos_world=GetTransformC2W(clu_pos_loc[0],clu_pos_loc[1],agv_pose[0],agv_pose[1],agv_pose[2])
            
            
            for iLM in global_landmark:
                dx=clu_pos_world[0,0]-iLM[1][0]
                dy=clu_pos_world[1,0]-iLM[1][1]
                dist1=np.sqrt(dx**2+dy**2)
                
                dx=clu_pos_world[0,0]-iLM[2][0]
                dy=clu_pos_world[1,0]-iLM[2][1]
                dist2=np.sqrt(dx**2+dy**2)
                
                if dist1<Search_Dist or dist2<Search_Dist:
                    #print([dist1, dist2])
                    self.corner_landmark += self.FindCorner(iClu)
                    break
            
        #print(self.corner_landmark)
        #print("================================")  
            
    def get_clu_pos(self, cluster):
        n=cluster.__len__()
        
        x_sum=0
        y_sum=0
        for i in range(n):
            x_sum+=cluster[i][0]
            y_sum+=cluster[i][1]
        
        return [x_sum/n,y_sum/n]
        
    
    def pi_2_pi(self, val):
        return (val + np.pi) % (2 * np.pi) - np.pi
    


	#寻找断点
    def FindBreakPoint(self, pcl):
        summe = len(pcl)
        max_dist = 0.3  # old:0.25

        breakpoint = []
        for i in range(summe):
            delta_x = pcl[i + 1][0] - pcl[i][0]
            delta_y = pcl[i + 1][1] - pcl[i][1]
            dist = np.sqrt(delta_x ** 2 + delta_y ** 2)
            if dist > max_dist: breakpoint.append(i)
            if i == summe - 2: break

        return breakpoint   
    

	#寻找角点算法
    def FindCorner(self,cluster):
        N = 3

        if cluster.__len__() < 2 * N + 1: return []

        corner_point = []
        for iPtr in range(cluster.__len__() - N):
            if iPtr < (N+3) or iPtr > cluster.__len__() - (N+3): continue
            xl_k = 0
            xr_k = 0
            yl_k = 0
            yr_k = 0
            for j in range(N + 1):
                xl_k += cluster[iPtr - j][0]
                yl_k += cluster[iPtr - j][1]
                xr_k += cluster[iPtr + j][0]
                yr_k += cluster[iPtr + j][1]
            xl_k = xl_k / (N + 1)
            xr_k = xr_k / (N + 1)
            yl_k = yl_k / (N + 1)
            yr_k = yr_k / (N + 1)

            l_a = np.sqrt((cluster[iPtr][0] - xr_k) ** 2 + (cluster[iPtr][1] - yr_k) ** 2)
            l_b = np.sqrt((cluster[iPtr][0] - xl_k) ** 2 + (cluster[iPtr][1] - yl_k) ** 2)
            
            
            
            
            l_c = np.sqrt((xr_k - xl_k) ** 2 + (yr_k - yl_k) ** 2)

            l = (l_a + l_b + l_c) / 2
            res = np.sqrt(l * (l - l_a) * (l - l_b) * (l - l_c)) / l_a / l_b
            
            if res>0.3: corner_point.append([iPtr, res])
            

        if corner_point.__len__() == 0: return []

        D=3
        corner=[]

        for iCorn in range(corner_point.__len__()):
            if iCorn==0: corner.append(corner_point[iCorn])
            else:
                if corner_point[iCorn][0]-corner[-1][0] > D: corner.append(corner_point[iCorn])
                else:
                    if corner_point[iCorn][1] > corner[-1][1]: corner[-1]=corner_point[iCorn]

        res=[]
        for iCorn in corner:
            res.append(cluster[iCorn[0]][0:2].tolist())
            
        n=res.__len__()
            
        if n>2:
            to_leave_corner=[]
            
            for i in range(n):
                is_effect=True
                for jCorn in res:
                    dx=res[i][0]-jCorn[0]
                    dy=res[i][1]-jCorn[1]
                    dist=np.sqrt(dx**2+dy**2)
                    if dist<Search_Dist: 
                        is_effect=False
                        break
                
                if is_effect==True: to_leave_corner.append(res[i])
                
            
            return to_leave_corner
        elif n==2:
            dx=res[0][0]-res[1][0]
            dy=res[0][1]-res[1][1]
            dist=np.sqrt(dx**2+dy**2)
            if dist<Search_Dist: res=[]
            return res
                
        else: return res
    
SAMPLING_TIME=0.2
USE_PURE_LOC=0   
#定位算法类
class location:
    def __init__(self):
		#当前位姿
        self.pose=[0,0,0]
		#从表格中获取所有全局路标
        self.global_landmark=self.LoadLandmark("/opt/ros/indigo/share/hongrobot_2dnav")
        
        self.pose_pub=rospy.Publisher("agv_pose", Pose, queue_size=64)
        self.lm_pub=rospy.Publisher("agv_landmark", PointCloud, queue_size =64)        
        
        self.frame_landmark = []
        
        self.omega=0
        self.last_omega=self.omega
        
        self.CloseReq=0
        self.image_out=0
        
        self.timestamp=0
        self.timestamp_odom=0
        self.timestamp_pub_pose=0
        
        self.lidar_angle=[]
        self.lidar_ranges=[]


        #储存在amcl获得的当前位姿
        self.amcl_pose_x=0
        self.amcl_pose_y=0
        self.amcl_pose_yaw=0
        
        self.pose_init=0
        
        self.act_imu_dat=[0,0,0]
        self.last_imu_dat=[0,0,0]
        
        self.loc_mode=0
        
        self.nearest_obstacle=-1
        
        self.IsGetScan=0
        
        self.TurnFast=False
        
        self.lm_builder=build_landmark()
        self.lm_builder.set_build_enable(True)
        
        
    def set_pose(self,pose):
        self.pose=pose
        
    def SetTimestamp(self, time):
        self.timestamp=time    
    
    	#从xml表格中获取储存的地图中的全局路标点
    def LoadLandmark(self,path):
        xml_tool_obj = xml_tools()
        xml_file = xml_tool_obj.read_xml(path+"/landmark_data.xml")
        rootfiles = xml_tool_obj.find_nodes(xml_file, "Landmark_DATA/Landmark")
        landmark_list=[]
        
        for node in rootfiles:
            lm_id=int(node.attrib["id"])
            pose1=node.attrib["lm1_pose"].split()
            pose2=node.attrib["lm2_pose"].split()
            lm_tmp=[lm_id,[float(pose1[0]),float(pose1[1])],[float(pose2[0]),float(pose2[1])]]
            
            landmark_list.append(lm_tmp)
            
        return landmark_list
    

    
    def pi_2_pi(self, val):
        return (val + np.pi) % (2 * np.pi) - np.pi  
    
    
    	#获取雷达数据的回调函数
    def get_scan_cb(self, scan_msgs):
        angle = []    
        ranges=[]
        angle_start=scan_msgs.angle_min
    
        for i in range(len(scan_msgs.ranges)):
            if scan_msgs.ranges[i]>Range_Max: continue
            if scan_msgs.ranges[i]<Range_Min: continue
                
            
            head_angle = angle_start + i * scan_msgs.angle_increment
            angle.append(head_angle)
            ranges.append(scan_msgs.ranges[i])
                    
        self.lidar_angle=angle
        self.lidar_ranges=ranges  
        
        return
    
    def get_amcl_pose_cb(self, amcl_pose):
        self.amcl_pose_x=amcl_pose.pose.pose.position.x
        self.amcl_pose_y=amcl_pose.pose.pose.position.y
        
        siny_cosp=2*(amcl_pose.pose.pose.orientation.w*amcl_pose.pose.pose.orientation.z)
        cosy_cosp=1 - 2*(amcl_pose.pose.pose.orientation.z**2)
        yaw=np.arctan2(siny_cosp, cosy_cosp)
        self.amcl_pose_yaw=yaw
        
        if self.loc_mode!=0: return
        
        ratio=0.7
        
        self.pose[0]=self.pose[0]*(1-ratio)+self.amcl_pose_x*ratio
        self.pose[1]=self.pose[1]*(1-ratio)+self.amcl_pose_y*ratio
        self.pose[2]=self.pose[2]*(1-ratio)+self.amcl_pose_yaw*ratio

    def get_odom_cb(self, odom):
        if self.timestamp_odom==0: 
            self.timestamp_odom = rospy.get_time()
            return
        
        delta_T=(rospy.get_time() - self.timestamp_odom)/2
        self.timestamp_odom = rospy.get_time()
        
        self.act_imu_dat[0]=odom.twist.twist.linear.x*np.cos(self.pose[2])
        self.act_imu_dat[1]=odom.twist.twist.linear.x*np.sin(self.pose[2])
        self.act_imu_dat[2]=odom.twist.twist.angular.z
        
        if odom.twist.twist.angular.z>0.1: self.TurnFast=True
        else: self.TurnFast=False
        
        self.pose[0]+=(self.act_imu_dat[0]+self.last_imu_dat[0])*delta_T
        self.pose[1]+=(self.act_imu_dat[1]+self.last_imu_dat[1])*delta_T
        self.pose[2]=self.pi_2_pi((self.act_imu_dat[2]+self.last_imu_dat[2])*delta_T+self.pose[2])
        
        self.last_imu_dat=self.act_imu_dat 
        
    def landmark_match(self,lm_local,lm_global):
        dist_max=0.5
        n=lm_local.__len__()
        
        
        
        out=[]
        for i in range(lm_global.__len__()):
            lm_pair=lm_global[i]
            lm1=lm_pair[1]
            lm2=lm_pair[2]
            
            hit_dist1=-1.0
            hit1=0
            
            hit_dist2=-1.0
            hit2=0
            res=[]
            for j in range(n):
                LM_Pw=GetTransformC2W(lm_local[j][0],lm_local[j][1],self.pose[0],self.pose[1],self.pose[2])
                #print(LM_Pw)
                
                x_=lm1[0]-LM_Pw[0,0]
                y_=lm1[1]-LM_Pw[1,0]
                
                dist=np.sqrt(x_**2+y_**2)
                
                if hit_dist1<0.0: 
                    hit_dist1=dist
                    hit1=j
                else:
                    if hit_dist1>dist:
                        hit_dist1=dist
                        hit1=j
                        
                x_=lm2[0]-LM_Pw[0,0]
                y_=lm2[1]-LM_Pw[1,0]
                dist=np.sqrt(x_**2+y_**2)
                
                if hit_dist2<0.0: 
                    hit_dist2=dist
                    hit2=j
                else:
                    if hit_dist2>dist:
                        hit_dist2=dist
                        hit2=j
                
            if hit_dist1 < dist_max:
                res.append(hit1)
                
            if hit_dist2 < dist_max and hit2!=hit1:
                res.append(hit2)
                
            if res.__len__()==2:
                out.append([i,res[0],res[1]])
            
        return out
     
    def loop_normal(self):
        delta_T=rospy.get_time() - self.timestamp
        if delta_T < SAMPLING_TIME: return
        
        self.timestamp = rospy.get_time()
        
        if self.lidar_angle.__len__()==0: return
        
        frame=RadarFrame(0, self.lidar_angle, self.lidar_ranges) #建立一个雷达帧 
        
        n=frame.corner_landmark.__len__()
        
        if n==0: return
        
        if self.TurnFast==False:
            lm_list=[]
            c_theta = np.cos(self.pose[2])
            s_theta = np.sin(self.pose[2])
            
            for i in range(n):
                dist=np.sqrt(frame.corner_landmark[i][0]**2+frame.corner_landmark[i][1]**2)
                if dist>1.8: continue
            
                p_x=c_theta*(frame.corner_landmark[i][0]+0.083)-s_theta*frame.corner_landmark[i][1]+self.pose[0]
                p_y=s_theta*(frame.corner_landmark[i][0]+0.083)+c_theta*frame.corner_landmark[i][1]+self.pose[1]
                lm_list.append([p_x,p_y])
            
            
            self.lm_builder.loop(lm_list)
        
        
        lm_msg=PointCloud()
        lm_msg.header.frame_id = "agv_landmark"
        lm_msg.header.stamp=rospy.get_rostime()
        for i in range(n):
            point1=Point() 
            point1.x=frame.corner_landmark[i][0]
            point1.y=frame.corner_landmark[i][1]
            lm_msg.points.append(point1)            
        self.lm_pub.publish(lm_msg)
        
        
    
    def loop_loc(self):
        delta_T=rospy.get_time() - self.timestamp
        if delta_T < SAMPLING_TIME: return
        
        self.timestamp = rospy.get_time()
        
        if self.lidar_angle.__len__()==0: return

        frame=RadarFrameLoc(0, self.lidar_angle, self.lidar_ranges,self.global_landmark,self.pose) #建立一个雷达帧        

        
        match_res=self.landmark_match(frame.corner_landmark,self.global_landmark)
        #print(match_res)
        
        if match_res.__len__()>0:
            ref_nr1=match_res[0][1]
            ref_nr2=match_res[0][2]
            global_lm_nr=match_res[0][0]
            
            reflector1=frame.corner_landmark[ref_nr1]
            reflector2=frame.corner_landmark[ref_nr2]
            
            lm_global_left=self.global_landmark[global_lm_nr][1]
            lm_global_right=self.global_landmark[global_lm_nr][2]
            
            th_loc=self.pi_2_pi(np.pi/2.0-np.arctan2(reflector2[0]-reflector1[0],reflector1[1]-reflector2[1]))
            
                    
            c_theta = np.cos(th_loc)
            s_theta = np.sin(th_loc)
            R = np.array([[c_theta, -s_theta], [s_theta, c_theta]])
            
            r12=np.sqrt((reflector2[0]-reflector1[0])**2+(reflector2[1]-reflector1[1])**2)

            
            P12=np.array([[r12], [0.0]])
            P2=np.array([[reflector2[0]], [reflector2[1]]])
            
            
            t=P12-np.dot(R,P2)
            pose_loc=[t[0,0],t[1,0],th_loc]
            
            th_p1=np.arctan2(lm_global_right[1]-lm_global_left[1],lm_global_right[0]-lm_global_left[0])
            
            c_theta = np.cos(th_p1)
            s_theta = np.sin(th_p1)
            R = np.array([[c_theta, -s_theta], [s_theta, c_theta]])
            
            t=np.array([[lm_global_left[0]], [lm_global_left[1]]])
            Pc=np.array([[pose_loc[0]], [pose_loc[1]]])
            
            Pw=np.dot(R,Pc)+t
            th_w=self.pi_2_pi(th_p1+th_loc)
            
            pose_world=[Pw[0,0],Pw[1,0],th_w]
            
            #print(pose_loc)  
            print(pose_world)          
        
        
        
        n=match_res.__len__()
        #print(match_res)
        
        
        
        lm_msg=PointCloud()
        lm_msg.header.frame_id = "landmark"
        lm_msg.header.stamp=rospy.get_rostime()
        for i in range(n):
            point1=Point() 
            point1.x=frame.corner_landmark[match_res[i][1]][0]
            point1.y=frame.corner_landmark[match_res[i][1]][1]
            lm_msg.points.append(point1)
            point2=Point()
            point2.x=frame.corner_landmark[match_res[i][2]][0]
            point2.y=frame.corner_landmark[match_res[i][2]][1]
            lm_msg.points.append(point2)
            
        self.lm_pub.publish(lm_msg)

class build_landmark:
    def __init__(self):
        self.lm_map=[]
        self.lm_outlier=[]
        self.enable=False
        
        self.timestamp=0.0
        
        self.sampling_time=0.3
        
        self.lm_pub=rospy.Publisher("global_landmark", PointCloud, queue_size = 64)      

    def set_build_enable(self,enable):
        self.enable=enable
        
    def loop(self,lm_list):
        delta_T=rospy.get_time() - self.timestamp
        if delta_T < self.sampling_time: return
        
        self.timestamp = rospy.get_time()
        
        
        if lm_list.__len__()==0: return
        
        uneffect_max_dist=0.2
        effect_dist=0.5
        outlier_dist=0.7
        
        if self.enable==False: return
        
        
        n=self.lm_map.__len__()
        
        if n==0:
            self.lm_map+=lm_list
            for i in range(self.lm_map.__len__()):
                self.lm_outlier.append(False)
        else:
            for iLM in lm_list:
                is_exist=False
                for j in range(n):
                    if self.lm_outlier[j]==True: continue
                    
                    dx=iLM[0]-self.lm_map[j][0]
                    dy=iLM[1]-self.lm_map[j][1]
                    dist=np.sqrt(dx**2+dy**2)
                    if dist < effect_dist:
                        is_exist=True
                        if dist > uneffect_max_dist: self.lm_outlier[j]=True
                        else:
                            self.lm_map[j][0]=(self.lm_map[j][0]+iLM[0])/2.0
                            self.lm_map[j][1]=(self.lm_map[j][1]+iLM[1])/2.0
                        break
                    
                    
                if is_exist==False:
                    is_outlier=False
                    for i in range(n):
                        if self.lm_outlier[i]==True:
                            dx=iLM[0]-self.lm_map[i][0]
                            dy=iLM[1]-self.lm_map[i][1]
                            dist=np.sqrt(dx**2+dy**2)
                            if dist < outlier_dist:
                                is_outlier=True
                                break
                    
                    if is_outlier==False:
                        self.lm_map.append(iLM)
                        self.lm_outlier.append(False)
        
        
        
        
        lm_msg=PointCloud()
        lm_msg.header.frame_id = "global_landmark"
        lm_msg.header.stamp=rospy.get_rostime()
        for i in range(n):
            if self.lm_outlier[i]==True: continue
            
            point1=Point() 
            point1.x=self.lm_map[i][0]
            point1.y=self.lm_map[i][1]
            lm_msg.points.append(point1)            
        
        
        self.lm_pub.publish(lm_msg)
        
        
    

if __name__ == '__main__':
    try:
        rospy.init_node('LaserTest_location', anonymous=True)
        rospy.loginfo("start the laser test!")
        loc = location()
        rate = rospy.Rate(800)
        rospy.Subscriber("/scan", LaserScan, loc.get_scan_cb, queue_size=1000)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, loc.get_amcl_pose_cb, queue_size=32)
        rospy.Subscriber("/odom", Odometry, loc.get_odom_cb, queue_size=100)
        
        
        loc.SetTimestamp(rospy.get_time())
        
        while not rospy.is_shutdown():
            if USE_PURE_LOC==0: loc.loop_normal()
            else: loc.loop_loc()
            rate.sleep()

        print("test finished!")


    except rospy.ROSInterruptException:
        rospy.loginfo("can not start the laser test!")
        pass 


















    
    
