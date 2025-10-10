#!/usr/bin/env python3
# gui_end_effector_goal_multiarray.py

import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

TOPIC_NAME = '/goal/end_effector_pose'
SCALE_FACTOR = 1.0  # 단위 변환이 필요하면 조정 (예: mm->m: 0.001)

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('ee_goal_gui_pub_multiarray')
        self.pub = self.create_publisher(Float32MultiArray, TOPIC_NAME, 10)
        # 선택: 레이아웃 정보(2D 아님, 단순 1D 배열로 size=2)
        self.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label='xy', size=2, stride=2)],
            data_offset=0
        )

    def send_xy(self, x_val: float, y_val: float):
        msg = Float32MultiArray()
        msg.layout = self.layout  # 필요 없으면 이 줄 삭제해도 동작함
        msg.data = [float(x_val) * SCALE_FACTOR, float(y_val) * SCALE_FACTOR]
        self.pub.publish(msg)
        self.get_logger().info(f'Published Float32MultiArray: [x={msg.data[0]:.3f}, y={msg.data[1]:.3f}]')


def main():
    rclpy.init()
    node = GoalPublisher()

    # --- Tkinter GUI ---
    root = tk.Tk()
    root.title("End Effector Goal Publisher (Float32MultiArray)")

    content = ttk.Frame(root, padding=12)
    content.grid(row=0, column=0, sticky="nsew")
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)
    for i in range(3):
        content.rowconfigure(i, weight=0)
    content.columnconfigure(1, weight=1)

    # X: 0 ~ 200
    ttk.Label(content, text="X (0 ~ 200)").grid(row=0, column=0, sticky="w")
    x_var = tk.DoubleVar(value=200.0)
    x_scale = ttk.Scale(content, from_=100, to=400, orient="horizontal", variable=x_var)
    x_scale.grid(row=0, column=1, sticky="ew", padx=6)
    x_val_lbl = ttk.Label(content, text="0.0")
    x_val_lbl.grid(row=0, column=2, padx=4)

    # Y: -200 ~ 200
    ttk.Label(content, text="Y (-200 ~ 200)").grid(row=1, column=0, sticky="w")
    y_var = tk.DoubleVar(value=0.0)
    y_scale = ttk.Scale(content, from_=-200, to=200, orient="horizontal", variable=y_var)
    y_scale.grid(row=1, column=1, sticky="ew", padx=6)
    y_val_lbl = ttk.Label(content, text="-200.0")
    y_val_lbl.grid(row=1, column=2, padx=4)

    # 값 변화 라벨 업데이트
    def on_scale_change(*_):
        x_val_lbl.config(text=f"{x_var.get():.1f}")
        y_val_lbl.config(text=f"{y_var.get():.1f}")
        node.send_xy(x_var.get(), y_var.get())
    x_var.trace_add("write", on_scale_change)
    y_var.trace_add("write", on_scale_change)

    # 버튼들
    btn_frame = ttk.Frame(content)
    btn_frame.grid(row=2, column=0, columnspan=3, pady=(12, 0))

    def send_clicked():
        try:
            node.send_xy(x_var.get(), y_var.get())
        except Exception as e:
            messagebox.showerror("Publish Error", str(e))

    def quit_clicked():
        root.quit()

    send_btn = ttk.Button(btn_frame, text="Send Pose (Float32MultiArray)", command=send_clicked)
    send_btn.grid(row=0, column=0, padx=4)
    quit_btn = ttk.Button(btn_frame, text="Quit", command=quit_clicked)
    quit_btn.grid(row=0, column=1, padx=4)

    # rclpy 이벤트 펌핑
    def pump_ros():
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception as e:
            node.get_logger().warn(f"spin_once exception: {e}")
        finally:
            root.after(50, pump_ros)  # ~20Hz

    root.after(50, pump_ros)

    # 종료 처리
    def on_close():
        try:
            root.destroy()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    root.protocol("WM_DELETE_WINDOW", on_close)

    root.mainloop()
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
