import tkinter as tk
from PIL import Image, ImageTk


class MovementStatesCallback:
    def __init__(self, state, gui):
        self.state = state
        self.gui = gui

    def callback(self, event):
        self.gui.movement_states_change(self.state)


class ToggleStatesCallback:
    def __init__(self, state, gui):
        self.state = state
        self.gui = gui

    def callback(self, event):
        self.gui.other_state_change(self.state)


class GUI:
    def __init__(self, callback, assets_dir="assets", movement_states=None, toggle_states=None, logger=None):
        self.root = tk.Tk()
        self.frame = tk.Frame(self.root)
        self.frame.grid(row=0, column=0)
        self.logger = logger

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

        self.tools_frame = tk.Frame(self.root)
        self.tools_frame.grid(row=0, column=2, sticky='ns', padx=20)

        # List here all driving states. Only one of them can be active. In the array of buttons states ascii code of
        # symbol will be written on the 4th position
        # self.movement_states = {'a': 'autonomous', 's': 'manual'}
        self.movement_states = movement_states
        self.movement_states['s'] = "manual"
        self.movement_callbacks = []
        # Bind key for autonomous driving
        for key in self.movement_states.keys():
            self.movement_callbacks.append(MovementStatesCallback(key, self))
            self.root.bind(f'<{key}>', self.movement_callbacks[-1].callback)
        # self.root.bind('<s>', lambda event: self.movement_states_change('s'))
        # self.root.bind('<a>', lambda event: self.movement_states_change('a'))

        # List here all states which can be toggled, first element of tuple is description, second is position
        # in array of buttons states which will be sent to bot, so don't overwrite already existing states
        self.toggle_states = toggle_states
        self.toggle_callbacks = []
        for key in self.toggle_states.keys():
            self.toggle_callbacks.append(ToggleStatesCallback(key, self))
            self.root.bind(f'<{key}>', self.toggle_callbacks[-1].callback)
        # self.root.bind('<o>', lambda event: self.other_state_change('o'))

        self.movement_labels = {}
        self.other_labels = {}

        # Movement section
        movement_section = tk.LabelFrame(self.tools_frame, text="Movement", padx=10, pady=10, font=("Helvetica", 16))
        movement_section.pack(fill="both", expand="yes", padx=10, pady=10)

        for key, value in self.movement_states.items():
            if key == 's':
                label = tk.Label(movement_section, text=f"{key} : {value}", fg="green", font=("Helvetica", 14), anchor="w")
            else:
                label = tk.Label(movement_section, text=f"{key} : {value}", fg="black", font=("Helvetica", 14), anchor="w")
            label.pack(fill="x", pady=5)
            self.movement_labels[key] = label

        # Other functions section
        other_section = tk.LabelFrame(self.tools_frame, text="Other Functions", padx=10, pady=10, font=("Helvetica", 16))
        other_section.pack(fill="both", expand="yes", padx=10, pady=10)

        for key, value in self.toggle_states.items():
            label = tk.Label(other_section, text=f"{key} : {value[0]}", fg="black", font=("Helvetica", 14), anchor="w")
            label.pack(fill="x", pady=5)
            self.other_labels[key] = label

        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.resizable(False, False)

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

        self.buttons_states = [0, 0, 0, 0, 0]

        self.callback = callback
        self.buttons_states[4] = ord('s')
        self.callback(self.buttons_states, self.slider.get())
        self.root.mainloop()

    def up(self):
        if self.logger is not None:
            self.logger.info("Up")
        self.up_button.config(image=self.up_active_image)
        self.buttons_states[0] = 1
        self.callback(self.buttons_states, self.slider.get())

    def down(self):
        if self.logger is not None:
            self.logger.info("Down")
        self.down_button.config(image=self.down_active_image)
        self.buttons_states[2] = 1
        self.callback(self.buttons_states, self.slider.get())

    def left(self):
        if self.logger is not None:
            self.logger.info("Left")
        self.left_button.config(image=self.left_active_image)
        self.buttons_states[3] = 1
        self.callback(self.buttons_states, self.slider.get())

    def right(self):
        if self.logger is not None:
            self.logger.info("Right")
        self.right_button.config(image=self.right_active_image)
        self.buttons_states[1] = 1
        self.callback(self.buttons_states, self.slider.get())

    def movement_states_change(self, state):
        if self.logger is not None:
            self.logger.info(self.movement_states[state])
        self.buttons_states[4] = ord(state)
        self.callback(self.buttons_states, self.slider.get())
        self.toggle_movement_state(state)

    def other_state_change(self, state):
        if self.logger is not None:
            self.logger.info(self.toggle_states[state][0])
        if len(self.buttons_states) <= self.toggle_states[state][1]:
            self.buttons_states.extend([0]*(self.toggle_states[state][1] - len(self.buttons_states) + 1))
        self.buttons_states[self.toggle_states[state][1]] = int(self.other_labels.get(state).cget("fg") == "black")
        self.callback(self.buttons_states, self.slider.get())
        self.toggle_other_state(state)

    def reset_up(self):
        if self.logger is not None:
            self.logger.info("Up released")
        self.up_button.config(image=self.up_image)
        self.buttons_states[0] = 0
        self.callback(self.buttons_states, self.slider.get())

    def reset_down(self):
        if self.logger is not None:
            self.logger.info("Down released")
        self.down_button.config(image=self.down_image)
        self.buttons_states[2] = 0
        self.callback(self.buttons_states, self.slider.get())

    def reset_left(self):
        if self.logger is not None:
            self.logger.info("Left released")
        self.left_button.config(image=self.left_image)
        self.buttons_states[3] = 0
        self.callback(self.buttons_states, self.slider.get())

    def reset_right(self):
        if self.logger is not None:
            self.logger.info("Right released")
        self.right_button.config(image=self.right_image)
        self.buttons_states[1] = 0
        self.callback(self.buttons_states, self.slider.get())

    def slider_changed(self):
        if self.logger is not None:
            self.logger.info(f"Slider in state: {self.slider.get()}")

    def toggle_movement_state(self, key):
        # Reset all movement labels
        for k, label in self.movement_labels.items():
            label.config(fg="black")
        # Set the selected movement label to green
        self.movement_labels[key].config(fg="green")
        if self.logger is not None:
            self.logger.info(f"Movement state {key} toggled")

    def toggle_other_state(self, key):
        label = self.other_labels.get(key)
        if label:
            current_color = label.cget("fg")
            new_color = "green" if current_color == "black" else "black"
            label.config(fg=new_color)
            if self.logger is not None:
                self.logger.info(f"Other state {key} toggled to {new_color}")

    def on_shutdown(self):
        self.root.destroy()
