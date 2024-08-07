import tkinter as tk
from PIL import Image, ImageTk
import time
import numpy as np

class GUI:
    def __init__(self):
        SAMPLE_PATH = "/home/Ilia.Nechaev/Downloads/1671761356_kalix-club-p-dora-na-rabochii-stol-oboi-3.jpg"

        self.node = None
        # Создание основного окна
        self.root = tk.Tk()
        self.assets_dir = None
        self.root.title("ROS2 Interface")
        # self.image_frame = tk.Frame(self.root)
        self.image_frame = tk.Canvas(self.root, width=1280, height=960)
        self.image_frame.grid(row=0, column=0)
        
        self.control_frame = tk.Frame(self.root)
        self.control_frame.grid(row=0, column=1)
        
        image = Image.new("RGB", (1280, 960), (255, 255, 255))
        
        image = image.resize((1280, 960), Image.BILINEAR)
        self.photo = ImageTk.PhotoImage(image)

        self.image_on_canvas = self.image_frame.create_image(0, 0, image=self.photo, anchor='nw')
        
        self.status_canvas = tk.Canvas(self.control_frame, width=180, height=180)
        self.status_canvas.grid(row=0, column=0)
        
        self.circle = self.status_canvas.create_oval(10, 10, 170, 170, fill="red")
        self.status_text = self.status_canvas.create_text(90, 90, text="status", fill="black")
        
        self.ok_button = tk.Button(self.control_frame, text="OK", state='disabled', height=5, width=10, command=self.calibrate)
        self.ok_button.grid(row=2, column=0)

        self.root.bind()
        
        self.empty_label = tk.Canvas(self.control_frame, width=180, height=180)
        self.empty_label.grid(row=1, column=0)

        self.checkbox_frame = tk.Frame(self.root)
        self.checkbox_frame.grid(row=0, column=2)
        self.small_tables = []
        self.small_tables_titles = []
        titles = ["Rotate left", "Straight", "Rotate right"]
        for k in range(3):
            frame = tk.Frame(self.checkbox_frame)
            frame.grid(row=k, column=0)

            label = tk.Label(frame, text=titles[k], font=("Arial", 24))
            self.small_tables_titles.append(label)
            label.grid(row=0, column=0, columnspan=3)
            self.small_tables.append([])
            for i in range(3):
                self.small_tables[k].append([])
                for j in range(3):
                    check = tk.Label(frame, text="X", font=("Arial", 20))
                    check.grid(row=i + 1, column=j)
                    self.small_tables[k][i].append(check)
        self.big_table_frame = tk.Frame(self.root)
        self.big_table_frame.grid(row=0, column=3)
        self.big_table_title = tk.Label(self.big_table_frame, text="Big table", font=("Arial", 24))
        self.big_table_title.grid(row=0, column=0, columnspan=3)
        self.big_table = []
        for i in range(3):
            check = tk.Label(self.big_table_frame, text="X", font=("Arial", 20))
            check.grid(row=1, column=i)
            self.big_table.append(check)
        print("Init finished")

    def update_placeholder(self):
        if self.assets_dir is not None:
            image = Image.open(self.assets_dir + "/placeholder.jpg")
            image = image.resize((1280, 960), Image.BILINEAR)
            self.photo = ImageTk.PhotoImage(image)
            self.image_frame.itemconfig(self.image_on_canvas, image=self.photo)

    def start(self):
        self.root.mainloop()

    def update_image(self, image: Image):
        photo = ImageTk.PhotoImage(image.resize((1280, 960), Image.BILINEAR))
        self.image_frame.itemconfig(self.image_on_canvas, image=photo)
        self.photo = photo
        self.image_frame.update_idletasks()

    def update_counter(self, counter: int):
        if counter == 1:
            self.status_canvas.itemconfig(self.circle, fill="blue")
            return
        if counter == 0:
            self.status_canvas.itemconfig(self.circle, fill="red")
            return
        self.status_canvas.itemconfig(self.circle, fill="green")

    def update_tables(self, table: np.ndarray):
        for i in range(table.shape[0]):
            for j in range(table.shape[1]):
                for k in range(table.shape[2]):
                    if table[i, j, k] == 0:
                        self.small_tables[i][j][k].config(text="X")
                    else:
                        self.small_tables[i][j][k].config(text="✓", fg="green")

    def update_big_table(self, table: np.ndarray):
        for i in range(table.shape[0]):
            if table[i] == 0:
                self.big_table[i].config(text="X")
            else:
                self.big_table[i].config(text="✓", fg="green")

    def enable_button(self):
        self.ok_button.config(state="normal")

    def calibrate(self):
        if self.node is not None:
            self.node.set_intrinsics()
            self.ok_button.config(state="disabled")
            self.root.after(1000, self.node.validate_service)

