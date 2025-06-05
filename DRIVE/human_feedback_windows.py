import cv2
import time
import torch
import queue
import threading
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

def save_video(frames, i):
    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter("{}.avi".format(i+1), fourcc, 20.0, (640, 360), isColor=True)

    for frame in frames:
        # Write the frame to the video file
        out.write(cv2.resize(frame, (640, 360)))
        # out.write(frame)

    # Release the VideoWriter object
    out.release()

class DataEvaluator:
    def __init__(self, master, rewards, distance_from_center, angle_from_center, steer_diff, speed, viewer_image=None, data_type='image'):
        self.master = master
        self.master.title("Data Evaluation Interface")
        self.master.geometry('1280x720')  # width x height + x_offset + y_offset

        self.feedback = tk.StringVar()

        # 数据初始化
        self.data = {
            "rewards": rewards,
            "distances": distance_from_center,
            "angles": angle_from_center,
            "steer": steer_diff,
            "speed": speed,
        }

        # 按钮框架
        self.button_frame = tk.Frame(master)
        # self.button_frame.grid(row=1, column=1, sticky="se")
        self.button_frame.place(relx=0.75, rely=0.75, anchor='se')

        if data_type == 'image':
            # 保存视频
            for i, viewer_image_ in enumerate(viewer_image):
                save_video(viewer_image_, i)

            # 配置网络布局
            self.master.grid_rowconfigure(0, weight=3)
            self.master.grid_rowconfigure(1, weight=2)
            self.master.grid_columnconfigure(0, weight=1)
            self.master.grid_columnconfigure(1, weight=1)

            # 视频画布
            self.canvas1 = tk.Canvas(master, width=640, height=360)
            self.canvas1.place(relx=0.25, rely=0.25, anchor='center')
            self.canvas2 = tk.Canvas(master, width=640, height=360)
            self.canvas2.place(relx=0.75, rely=0.25, anchor='center')

            # 数据图表
            self.fig, self.axes = plt.subplots(2, 1, figsize=(6, 4))
            self.plot_image_data()
            self.canvas_plot = FigureCanvasTkAgg(self.fig, master=self.master)
            self.canvas_plot.get_tk_widget().place(relx=0.25, rely=0.75, anchor='center')

            # 视频播放相关初始化
            self.thread1 = None
            self.thread2 = None
            self.stop_event = threading.Event()
            self.lock = threading.Lock()
            self.queue1 = queue.Queue()
            self.queue2 = queue.Queue()
            self.stop_event = threading.Event()
            self.update_frame()
            self.img_tk1 = None
            self.img_tk2 = None
            self.load_video1()
            self.load_video2()
        else:
            # 创建 Matplotlib 图表
            self.fig, self.axes = plt.subplots(5, 1, figsize=(8, 10))
            self.plot_data()
            # 将 Matplotlib 图表嵌入 Tkinter 界面
            self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # 通用按钮配置
        buttons = [
            ("Good (1)", '1'),
            ("Good (2)", '2'),
            ("Equal (3)", '3'),
            ("Incomparable (4)", '4')
        ]
        for text, cmd in buttons:
            btn = tk.Button(self.button_frame, text=text, command=lambda c=cmd: self.evaluate(c))
            # btn.pack(side=tk.LEFT, padx=5, pady=5)
            btn.pack(side=tk.TOP, padx=10, pady=5)

        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def plot_image_data(self):
        """专门为图片模式绘制reward和distance图表"""
        for i, key in enumerate(["rewards", "distances"]):
            self.axes[i].clear()
            for j in range(len(self.data[key])):
                values = self.data[key][j]
                mu = torch.mean(torch.tensor(values))
                std = torch.std(torch.tensor(values))
                label = "{} {} Mean: {:.4f} Std: {:.4f}".format(key.capitalize(), j + 1, mu, std)
                self.axes[i].plot(values, label=label)
            self.axes[i].set_title(key.capitalize())
            self.axes[i].legend()
        self.fig.tight_layout()

    def plot_data(self):
        # 绘制五个子图
        for i, (key, values) in enumerate(self.data.items()):
            self.axes[i].clear()
            for j in range(len(values)):
                mu = torch.mean(torch.tensor(values[j]))
                std = torch.std(torch.tensor(values[j]))
                label = "{} {} Mean: {:.4f} Std: {:.4f}".format(key.capitalize(), j + 1, mu, std)
                self.axes[i].plot(values[j], label=label)
            self.axes[i].set_title(key.capitalize())
            self.axes[i].legend(loc='upper right')

    def evaluate(self, choice: str) -> str:
        self.feedback.set(choice)
        self.on_closing()
        self.master.quit()  # 停止主循环

    def on_closing(self):
        plt.close('all')
        self.stop_event.set()
        self.master.destroy()

    def load_video1(self):
        self._start_video_thread("1.avi", self.queue1)

    def load_video2(self):
        self._start_video_thread("2.avi", self.queue2)

    def _start_video_thread(self, path, queue):
        if self.thread1 and self.thread1.is_alive():
            self.stop_event.set()
            self.thread1.join()
        thread = threading.Thread(target=self.play_video, args=(path, queue))
        thread.start()

    def play_video(self, video_path, frame_queue):
        cap = cv2.VideoCapture(video_path)
        fps = cap.get(cv2.CAP_PROP_FPS)
        delay = 1 / fps  # Delay to match the video FPS
        while not self.stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (400, 300))
            frame_queue.put(frame)
            time.sleep(delay)  # Add delay to control frame rate
        cap.release()

    def update_frame(self):
        """持续更新视频帧"""
        if not self.queue1.empty():
            frame = self.queue1.get()
            self._update_canvas(frame, self.canvas1, 1)

        if not self.queue2.empty():
            frame = self.queue2.get()
            self._update_canvas(frame, self.canvas2, 2)

        self.master.after(10, self.update_frame)

    def _update_canvas(self, frame, canvas, idx):
        img = Image.fromarray(frame)
        img_tk = ImageTk.PhotoImage(image=img)
        if idx == 1:
            self.img_tk1 = img_tk
        else:
            self.img_tk2 = img_tk
        canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)

def evaluate_data(seq_obs, true_rewards, seq_distance_from_center, seq_angle_from_center, seq_viewer_image=None, data_type='image'):
    root = tk.Tk()

    # 数据预处理
    seq_steer_diff = seq_obs[0][:, 65] - seq_obs[0][:, 66], seq_obs[1][:, 65] - seq_obs[1][:, 66]
    seq_speed = seq_obs[0][:, 67] * 3.6, seq_obs[1][:, 67] * 3.6

    app = DataEvaluator(root, true_rewards, seq_distance_from_center, seq_angle_from_center, seq_steer_diff, seq_speed, viewer_image=seq_viewer_image, data_type=data_type)
    root.mainloop()
    feedback_value = app.feedback.get()
    # root.destroy()

    time.sleep(0.1)
    return feedback_value

if __name__ == "__main__":
    print("User Feedback:", evaluate_data())