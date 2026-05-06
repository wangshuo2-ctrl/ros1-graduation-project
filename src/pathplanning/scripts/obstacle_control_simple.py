#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版障碍物控制GUI - 支持GUI输入坐标放置障碍物
"""

import rospy
import tkinter as tk
from tkinter import messagebox
import threading
import time
import random
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import PointStamped

class SimpleObstacleControl:
    def __init__(self, root):
        self.root = root
        self.root.title("障碍物控制面板")
        self.root.geometry("500x500")
        
        # ROS初始化
        rospy.init_node('simple_obstacle_gui', anonymous=True, disable_signals=True)
        
        # 变量
        self.obstacle_count = tk.IntVar(value=5)
        self.obstacle_radius = tk.DoubleVar(value=0.3)
        self.allow_overlap = tk.BooleanVar(value=False)
        self.x_coord = tk.DoubleVar(value=0.0)
        self.y_coord = tk.DoubleVar(value=0.0)
        self.dynamic_enabled = tk.BooleanVar(value=False)
        
        # 创建发布者
        self.click_publisher = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
        
        # 创建界面
        self.create_widgets()
        
        # 连接ROS服务
        self.connect_services()
    
    def create_widgets(self):
        """创建界面组件"""
        # 标题
        title_label = tk.Label(self.root, text="障碍物控制面板", 
                              font=("Arial", 16, "bold"))
        title_label.pack(pady=10)
        
        # 手动放置框架
        manual_frame = tk.LabelFrame(self.root, text="手动放置障碍物", padx=10, pady=10)
        manual_frame.pack(fill="x", padx=20, pady=5)
        
        # 坐标输入
        tk.Label(manual_frame, text="X坐标:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        tk.Entry(manual_frame, textvariable=self.x_coord, width=10).grid(row=0, column=1, padx=5, pady=5)
        
        tk.Label(manual_frame, text="Y坐标:").grid(row=0, column=2, sticky="w", padx=5, pady=5)
        tk.Entry(manual_frame, textvariable=self.y_coord, width=10).grid(row=0, column=3, padx=5, pady=5)
        
        tk.Button(manual_frame, text="放置障碍物", 
                 command=self.place_obstacle, width=12).grid(row=0, column=4, padx=5, pady=5)
        
        # 随机放置按钮
        tk.Button(manual_frame, text="随机放置1个", 
                 command=lambda: self.random_obstacle(1), width=12).grid(row=1, column=0, padx=5, pady=5)
        
        tk.Button(manual_frame, text="随机放置5个", 
                 command=lambda: self.random_obstacle(5), width=12).grid(row=1, column=1, padx=5, pady=5)
        
        # 参数设置框架
        param_frame = tk.LabelFrame(self.root, text="参数设置", padx=10, pady=10)
        param_frame.pack(fill="x", padx=20, pady=5)
        
        # 障碍物数量
        tk.Label(param_frame, text="障碍物数量:").grid(row=0, column=0, sticky="w", pady=5)
        count_scale = tk.Scale(param_frame, from_=1, to=20, orient="horizontal",
                              variable=self.obstacle_count, length=200)
        count_scale.grid(row=0, column=1, padx=10, pady=5)
        tk.Label(param_frame, textvariable=self.obstacle_count).grid(row=0, column=2)
        
        # 障碍物半径
        tk.Label(param_frame, text="障碍物半径(m):").grid(row=1, column=0, sticky="w", pady=5)
        radius_scale = tk.Scale(param_frame, from_=0.1, to=1.0, resolution=0.1,
                               orient="horizontal", variable=self.obstacle_radius, length=200)
        radius_scale.grid(row=1, column=1, padx=10, pady=5)
        tk.Label(param_frame, textvariable=self.obstacle_radius).grid(row=1, column=2)
        
        # 允许重叠
        tk.Checkbutton(param_frame, text="允许障碍物重叠", 
                      variable=self.allow_overlap).grid(row=2, column=0, columnspan=3, pady=5, sticky="w")
        
        # 动态障碍物开关
        tk.Checkbutton(param_frame, text="启用动态障碍物", 
                      variable=self.dynamic_enabled).grid(row=3, column=0, columnspan=3, pady=5, sticky="w")
        
        # 控制按钮框架
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=20)
        
        # 生成按钮
        self.generate_btn = tk.Button(button_frame, text="生成随机障碍物", 
                                     command=self.generate_obstacles,
                                     width=15, height=2, state="disabled")
        self.generate_btn.grid(row=0, column=0, padx=10)
        
        # 清除按钮
        self.clear_btn = tk.Button(button_frame, text="清除所有障碍物", 
                                  command=self.clear_obstacles,
                                  width=15, height=2, state="disabled")
        self.clear_btn.grid(row=0, column=1, padx=10)
        
        # 更新参数按钮
        self.update_btn = tk.Button(button_frame, text="更新参数", 
                                   command=self.update_parameters,
                                   width=15, height=2, state="disabled")
        self.update_btn.grid(row=0, column=2, padx=10)
        
        # 测试功能框架
        test_frame = tk.LabelFrame(self.root, text="测试功能", padx=10, pady=10)
        test_frame.pack(fill="x", padx=20, pady=5)
        
        # 测试按钮
        tk.Button(test_frame, text="递增测试", 
                 command=self.incremental_test, width=10).pack(side="left", padx=5, pady=5)
        tk.Button(test_frame, text="密度测试", 
                 command=self.density_test, width=10).pack(side="left", padx=5, pady=5)
        tk.Button(test_frame, text="动态测试", 
                 command=self.dynamic_test, width=10).pack(side="left", padx=5, pady=5)
        
        # 状态显示
        self.status_label = tk.Label(self.root, text="等待连接ROS服务...", 
                                    font=("Arial", 10))
        self.status_label.pack(pady=10)
    
    def connect_services(self):
        """连接ROS服务"""
        def connect():
            try:
                rospy.wait_for_service('/generate_random_obstacles', timeout=5)
                rospy.wait_for_service('/clear_random_obstacles', timeout=5)
                rospy.wait_for_service('/update_random_obstacle_parameters', timeout=5)
                
                self.generate_service = rospy.ServiceProxy('/generate_random_obstacles', Empty)
                self.clear_service = rospy.ServiceProxy('/clear_random_obstacles', Trigger)
                self.update_service = rospy.ServiceProxy('/update_random_obstacle_parameters', Empty)
                
                # 启用按钮
                self.root.after(0, self.enable_buttons)
                self.update_status("已连接到ROS服务")
                
            except Exception as e:
                self.update_status("连接失败: {}".format(e))
                self.show_error("无法连接到ROS服务")
        
        threading.Thread(target=connect, daemon=True).start()
    
    def show_error(self, message):
        """显示错误消息"""
        self.root.after(0, lambda: messagebox.showerror("错误", message))
    
    def show_info(self, message):
        """显示信息消息"""
        self.root.after(0, lambda: messagebox.showinfo("信息", message))
    
    def ask_yesno(self, title, message):
        """显示是/否对话框"""
        return messagebox.askyesno(title, message)
    
    def enable_buttons(self):
        """启用控制按钮"""
        self.generate_btn.config(state="normal")
        self.clear_btn.config(state="normal")
        self.update_btn.config(state="normal")
    
    def update_status(self, message):
        """更新状态显示"""
        self.root.after(0, lambda: self.status_label.config(text=message))
    
    def place_obstacle(self):
        """在指定位置放置障碍物"""
        def task():
            try:
                # 发布点击消息
                point_msg = PointStamped()
                point_msg.header.stamp = rospy.Time.now()
                point_msg.header.frame_id = "map"
                point_msg.point.x = self.x_coord.get()
                point_msg.point.y = self.y_coord.get()
                point_msg.point.z = 0.0
                
                self.click_publisher.publish(point_msg)
                
                self.update_status("在(%.2f, %.2f)放置障碍物" % (self.x_coord.get(), self.y_coord.get()))
                self.show_info("在(%.2f, %.2f)放置障碍物" % (self.x_coord.get(), self.y_coord.get()))
                
            except Exception as e:
                error_msg = "放置失败: {}".format(e)
                self.update_status(error_msg)
                self.show_error(error_msg)
        
        threading.Thread(target=task, daemon=True).start()
    
    def random_obstacle(self, count):
        """随机放置障碍物"""
        def task():
            try:
                for i in range(count):
                    # 生成随机坐标
                    x = random.uniform(-10, 10)
                    y = random.uniform(-10, 10)
                    
                    point_msg = PointStamped()
                    point_msg.header.stamp = rospy.Time.now()
                    point_msg.header.frame_id = "map"
                    point_msg.point.x = x
                    point_msg.point.y = y
                    point_msg.point.z = 0.0
                    
                    self.click_publisher.publish(point_msg)
                    rospy.sleep(0.1)  # 短暂延迟
                
                self.update_status("随机放置了{}个障碍物".format(count))
            except Exception as e:
                error_msg = "随机放置失败: {}".format(e)
                self.update_status(error_msg)
        
        threading.Thread(target=task, daemon=True).start()
    
    def generate_obstacles(self):
        """生成随机障碍物"""
        def task():
            try:
                # 设置参数
                rospy.set_param('/random_obstacle_publisher/obstacle_count', 
                               self.obstacle_count.get())
                rospy.set_param('/random_obstacle_publisher/obstacle_radius', 
                               self.obstacle_radius.get())
                rospy.set_param('/random_obstacle_publisher/allow_overlap', 
                               self.allow_overlap.get())
                rospy.set_param('/random_obstacle_publisher/enable_random_movement', 
                               self.dynamic_enabled.get())
                
                # 调用服务
                self.generate_service()
                
                count = self.obstacle_count.get()
                self.update_status("已生成 {} 个障碍物".format(count))
                self.show_info("已生成 {} 个障碍物".format(count))
                
            except Exception as e:
                error_msg = "生成失败: {}".format(e)
                self.update_status(error_msg)
                self.show_error("生成失败: {}".format(e))
        
        threading.Thread(target=task, daemon=True).start()
    
    def clear_obstacles(self):
        """清除所有障碍物"""
        def task():
            try:
                response = self.clear_service()
                self.update_status(response.message)
                self.show_info(response.message)
            except Exception as e:
                error_msg = "清除失败: {}".format(e)
                self.update_status(error_msg)
                self.show_error("清除失败: {}".format(e))
        
        threading.Thread(target=task, daemon=True).start()
    
    def update_parameters(self):
        """更新障碍物参数"""
        def task():
            try:
                # 设置参数
                rospy.set_param('/random_obstacle_publisher/obstacle_radius', 
                               self.obstacle_radius.get())
                rospy.set_param('/random_obstacle_publisher/allow_overlap', 
                               self.allow_overlap.get())
                rospy.set_param('/random_obstacle_publisher/enable_random_movement', 
                               self.dynamic_enabled.get())
                
                # 调用更新服务
                self.update_service()
                
                self.update_status("参数已更新")
                self.show_info("障碍物参数已更新")
                
            except Exception as e:
                error_msg = "参数更新失败: {}".format(e)
                self.update_status(error_msg)
                self.show_error(error_msg)
        
        threading.Thread(target=task, daemon=True).start()

    def incremental_test(self):
        """递增测试"""
        def task():
            for i in range(1, 6):
                rospy.set_param('/random_obstacle_publisher/obstacle_count', i)
                rospy.set_param('/random_obstacle_publisher/obstacle_radius', 0.3)
                self.generate_service()
                self.update_status("阶段 {}/5: {} 个障碍物".format(i, i))
                time.sleep(2)
            self.show_info("递增测试完成")
        
        threading.Thread(target=task, daemon=True).start()
    
    def density_test(self):
        """密度测试"""
        def task():
            for count in [5, 10, 15]:
                rospy.set_param('/random_obstacle_publisher/obstacle_count', count)
                rospy.set_param('/random_obstacle_publisher/obstacle_radius', 0.3)
                self.generate_service()
                self.update_status("密度测试: {} 个障碍物".format(count))
                time.sleep(3)
            self.show_info("密度测试完成")
        
        threading.Thread(target=task, daemon=True).start()
    
    def dynamic_test(self):
        """动态测试"""
        def task():
            rospy.set_param('/random_obstacle_publisher/enable_random_movement', True)
            rospy.set_param('/random_obstacle_publisher/obstacle_count', 8)
            rospy.set_param('/random_obstacle_publisher/obstacle_radius', 0.3)
            self.generate_service()
            self.update_status("动态测试已启动")
            self.show_info("障碍物将随机移动")
        
        threading.Thread(target=task, daemon=True).start()
    
    def on_closing(self):
        """窗口关闭事件"""
        if messagebox.askyesno("退出", "确定要退出吗？"):
            rospy.signal_shutdown("GUI关闭")
            self.root.destroy()

def main():
    root = tk.Tk()
    app = SimpleObstacleControl(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()