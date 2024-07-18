import tkinter as tk

def up():
    print("Up")
    up_button.config(image=up_active_image)

# def down():
#     print("Вниз")
#     down_button.config(image=down_active_image)
#
# def left():
#     print("Влево")
#     left_button.config(image=left_active_image)
#
# def right():
#     print("Вправо")
#     right_button.config(image=right_active_image)

# def stop(event):
#     print("Стоп")

def reset_up(event):
    print("Up release")
    up_button.config(image=up_image)

# def reset_down(event):
#     down_button.config(image=down_image)
#
# def reset_left(event):
#     left_button.config(image=left_image)
#
# def reset_right(event):
#     right_button.config(image=right_image)
#
# def slider_changed(event):
#     print(f"Ползунок в состоянии {slider.get()}")

root = tk.Tk()

frame = tk.Frame(root)
frame.grid(row=0, column=0)

up_image = tk.PhotoImage(file="assets/up.png")
up_active_image = tk.PhotoImage(file="assets/up_active.png")
up_button = tk.Label(frame, image=up_image)
up_button.grid(row=0, column=1)

# down_image = tk.PhotoImage(file="assets/down.png")
# down_active_image = tk.PhotoImage(file="assets/down_active.png")
# down_button = tk.Button(frame, image=down_image, command=down)
# down_button.grid(row=2, column=1)
#
# left_image = tk.PhotoImage(file="assets/left.png")
# left_active_image = tk.PhotoImage(file="assets/left_active.png")
# left_button = tk.Button(frame, image=left_image, command=left)
# left_button.grid(row=1, column=0)
#
# right_image = tk.PhotoImage(file="assets/right.png")
# right_active_image = tk.PhotoImage(file="assets/right_active.png")
# right_button = tk.Button(frame, image=right_image, command=right)
# right_button.grid(row=1, column=2)
#
# slider = tk.Scale(root, from_=0, to=1, resolution=0.01, orient=tk.VERTICAL, command=slider_changed)
# slider.grid(row=0, column=1, sticky='ns')

root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(1, weight=1)

root.bind('<Up>', lambda event: up())
# root.bind('<Down>', lambda event: down())
# root.bind('<Left>', lambda event: left())
# root.bind('<Right>', lambda event: right())

root.bind('<KeyRelease-Up>', reset_up)
# root.bind('<KeyRelease-Down>', reset_down)
# root.bind('<KeyRelease-Left>', reset_left)
# root.bind('<KeyRelease-Right>', reset_right)

# root.bind('<s>', stop)

root.mainloop()
