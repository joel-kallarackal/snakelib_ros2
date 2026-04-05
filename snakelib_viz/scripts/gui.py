import cv2
import tkinter as tk
from tkinter import Label, Button, Frame
from PIL import Image, ImageTk
import threading


class CameraApp:
    def __init__(self, window, window_title):
        self.window = window
        self.window.title(window_title)
        self.window.configure(bg="#121212")  # Dark mode base
        self.window.geometry("1000x650")

        # ---------- HEADER ----------
        self.header = Frame(window, bg="#1f1f1f", height=100)
        self.header.pack(fill="x")
        self.title = Label(
            self.header,
            text="Snake UI",
            font=("Helvetica", 20, "bold"),
            fg="#ffffff",
            bg="#1f1f1f"
        )
        self.title.pack(pady=20)

        # ---------- MAIN LAYOUT ----------
        self.main_frame = Frame(window, bg="#121212")
        self.main_frame.pack(fill="both", expand=True, padx=0, pady=0)

        # LEFT: VIDEO FEED
        self.video_card = Frame(self.main_frame, bg="#1e1e1e", bd=0, relief="ridge")
        self.video_card.pack(side="left", padx=10, pady=10)

        self.label = Label(self.video_card, bg="#1e1e1e")
        self.label.pack(padx=0, pady=0)

        # RIGHT: CONTROL PANEL
        self.control_panel = Frame(self.main_frame, bg="#1e1e1e", width=300, bd=2, relief="ridge")
        self.control_panel.pack(side="right", fill="y", padx=0, pady=0)

        # LED BUTTON
        self.led_on = False
        self.toggle_button = Button(
            self.control_panel,
            text="Turn LED ON",
            width=50,
            height=2,
            font=("Helvetica", 14, "bold"),
            fg="#ffffff",
            bg="#007acc",
            activebackground="#005f99",
            activeforeground="#ffffff",
            relief="flat",
            command=self.toggle_led
        )
        self.toggle_button.pack(pady=0, padx=0)

        self.extra_label = Label(
            self.control_panel,
            text="Status",
            font=("Helvetica", 15, "bold"),
            fg="#bbbbbb",
            bg="#1e1e1e",
            anchor="w"   # left align text
        )
        self.extra_label.pack(pady=10, fill="x", padx=5)

        # STATUS LABEL
        self.status_label = Label(
            self.control_panel,
            text="LED : OFF",
            font=("Helvetica", 12, "bold"),
            fg="#ff4d4d",
            bg="#1e1e1e",
            anchor="w"   # left align text
        )
        self.status_label.pack(pady=0, fill="x", padx=10)

        # ---------- CAMERA THREAD ----------
        url = "http://192.168.8.132:8080/?action=stream"
        self.cap = cv2.VideoCapture(url)
        self.running = True
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

        # ---------- WINDOW CLOSE ----------
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)

    def toggle_led(self):
        self.led_on = not self.led_on
        if self.led_on:
            print("LED turned ON")
            self.toggle_button.config(text="Turn LED OFF", bg="#d9534f", activebackground="#b52b27")
            self.status_label.config(text="LED : ON", fg="#32cd32")
        else:
            print("LED turned OFF")
            self.toggle_button.config(text="Turn LED ON", bg="#007acc", activebackground="#005f99")
            self.status_label.config(text="LED : OFF", fg="#ff4d4d")

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                # frame = cv2.resize(frame, (640, 480))
                frame = cv2.resize(frame, (960, 720))
                cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(cv2image)
                imgtk = ImageTk.PhotoImage(image=img)

                self.label.imgtk = imgtk
                self.label.configure(image=imgtk)

            self.label.after(15)

    def on_closing(self):
        self.running = False
        self.cap.release()
        self.window.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = CameraApp(root, "Camera Dashboard")
    root.mainloop()
