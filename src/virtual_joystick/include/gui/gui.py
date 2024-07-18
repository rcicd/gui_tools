import tkinter as tk
from PIL import Image, ImageTk


class GUI:
    def __init__(self, callback, assets_dir="assets"):
        self.root = tk.Tk()
        self.frame = tk.Frame(self.root)
        self.frame.grid(row=0, column=0)

        self.up_image = tk.PhotoImage(file=f"{assets_dir}/up.png")
        self.up_active_image = tk.PhotoImage(file=f"{assets_dir}/up_active.png")
        self.up_button = tk.Label(self.frame, image=self.up_image)
        self.up_button.grid(row=0, column=1)

        self.down_image = tk.PhotoImage(file=f"{assets_dir}/down.png")
        self.down_active_image = tk.PhotoImage(file=f"{assets_dir}/down_active.png")
        self.down_button = tk.Label(self.frame, image=self.down_image)
        self.down_button.grid(row=2, column=1)

        image = Image.open(f"{assets_dir}/logo.png")
        image = image.resize((327, 327))
        self.center_image = ImageTk.PhotoImage(image)
        self.center_label = tk.Label(self.frame, image=self.center_image)
        self.center_label.grid(row=1, column=1)

        self.left_image = tk.PhotoImage(file=f"{assets_dir}/left.png")
        self.left_active_image = tk.PhotoImage(file=f"{assets_dir}/left_active.png")
        self.left_button = tk.Label(self.frame, image=self.left_image)
        self.left_button.grid(row=1, column=0)

        self.right_image = tk.PhotoImage(file=f"{assets_dir}/right.png")
        self.right_active_image = tk.PhotoImage(file=f"{assets_dir}/right_active.png")
        self.right_button = tk.Label(self.frame, image=self.right_image)
        self.right_button.grid(row=1, column=2)

        self.slider = tk.Scale(
            self.root,
            from_=1,
            to=0,
            resolution=0.01,
            orient=tk.VERTICAL,
            command=lambda event: self.slider_changed()
        )
        self.slider.set(1)
        self.slider.grid(row=0, column=1, sticky='ns')

        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.resizable(False, False)

        self.root.title("Duckiebot and RaccoonScout Virtual Joystick")

        self.root.bind('<Up>', lambda event: self.up())
        self.root.bind('<Down>', lambda event: self.down())
        self.root.bind('<Left>', lambda event: self.left())
        self.root.bind('<Right>', lambda event: self.right())

        self.root.bind('<KeyRelease-Up>', lambda event: self.reset_up())
        self.root.bind('<KeyRelease-Down>', lambda event: self.reset_down())
        self.root.bind('<KeyRelease-Left>', lambda event: self.reset_left())
        self.root.bind('<KeyRelease-Right>', lambda event: self.reset_right())

        self.root.bind('<s>', lambda event: self.stop())

        self.buttons_states = [0, 0, 0, 0, 0]

        self.callback = callback
        self.buttons_states[4] = ord('s')
        self.callback(self.buttons_states, self.slider.get())

        self.root.mainloop()

    def up(self):
        print("Up")
        self.up_button.config(image=self.up_active_image)
        self.buttons_states[0] = 1
        self.callback(self.buttons_states, self.slider.get())

    def down(self):
        print("Down")
        self.down_button.config(image=self.down_active_image)
        self.buttons_states[2] = 1
        self.callback(self.buttons_states, self.slider.get())

    def left(self):
        print("Left")
        self.left_button.config(image=self.left_active_image)
        self.buttons_states[3] = 1
        self.callback(self.buttons_states, self.slider.get())

    def right(self):
        print("Right")
        self.right_button.config(image=self.right_active_image)
        self.buttons_states[1] = 1
        self.callback(self.buttons_states, self.slider.get())
    def stop(self):
        print("Stop")
        self.buttons_states[4] = ord('s')
        self.callback(self.buttons_states, self.slider.get())

    def reset_up(self):
        print("Up release")
        self.up_button.config(image=self.up_image)
        self.buttons_states[0] = 0
        self.callback(self.buttons_states, self.slider.get())

    def reset_down(self):
        print("Down release")
        self.down_button.config(image=self.down_image)
        self.buttons_states[2] = 0
        self.callback(self.buttons_states, self.slider.get())

    def reset_left(self):
        print("Left release")
        self.left_button.config(image=self.left_image)
        self.buttons_states[3] = 0
        self.callback(self.buttons_states, self.slider.get())

    def reset_right(self):
        print("Right release")
        self.right_button.config(image=self.right_image)
        self.buttons_states[1] = 0
        self.callback(self.buttons_states, self.slider.get())

    def slider_changed(self):
        print(f"Slider in state: {self.slider.get()}")

    def on_shutdown(self):
        self.root.destroy()
