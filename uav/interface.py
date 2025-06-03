# uav/interface.py
import tkinter as tk
from threading import Thread

exit_flag = [False]

def launch_control_gui(param_refs):
    def on_stop():
        exit_flag[0] = True

    def on_reset():
        param_refs['reset_flag'][0] = True

    def update_labels():
        l_val.set(f"{param_refs['L'][0]:.2f}")
        c_val.set(f"{param_refs['C'][0]:.2f}")
        r_val.set(f"{param_refs['R'][0]:.2f}")
        state_val.set(param_refs['state'][0])
        root.after(200, update_labels)

    root = tk.Tk()
    root.title("UAV Controller")
    root.geometry("300x350")

    l_val = tk.StringVar(); c_val = tk.StringVar(); r_val = tk.StringVar(); state_val = tk.StringVar()

    tk.Label(root, text="Brake Threshold Base").pack()
    brake_slider = tk.Scale(root, from_=10, to=100, orient='horizontal',
                            command=lambda v: param_refs['brake'][0].__setitem__(0, float(v)))
    brake_slider.set(param_refs['brake'][0][0])
    brake_slider.pack()

    tk.Label(root, text="Dodge Threshold Base").pack()
    dodge_slider = tk.Scale(root, from_=1, to=20, orient='horizontal',
                            command=lambda v: param_refs['dodge'][0].__setitem__(0, float(v)))
    dodge_slider.set(param_refs['dodge'][0][0])
    dodge_slider.pack()

    tk.Button(root, text="Reset Simulation", command=on_reset).pack(pady=5)
    tk.Button(root, text="Stop UAV", command=on_stop, bg='red', fg='white').pack(pady=5)

    tk.Label(root, text="Flow Magnitudes").pack()
    tk.Label(root, textvariable=l_val).pack()
    tk.Label(root, textvariable=c_val).pack()
    tk.Label(root, textvariable=r_val).pack()

    tk.Label(root, text="Current State:").pack()
    tk.Label(root, textvariable=state_val).pack()

    update_labels()
    root.mainloop()

def start_gui(param_refs=None):
    if param_refs is None:
        Thread(target=gui_exit, daemon=True).start()
    else:
        Thread(target=lambda: launch_control_gui(param_refs), daemon=True).start()

def gui_exit():
    root = tk.Tk()
    root.title("Stop UAV")
    root.geometry("200x100")
    btn = tk.Button(root, text="STOP", font=("Arial", 20), command=lambda: exit_flag.__setitem__(0, True))
    btn.pack(expand=True)
    root.mainloop()
