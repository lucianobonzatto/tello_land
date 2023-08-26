import rospy
import rospkg 
import tkinter as tk
from tkinter import Label, Entry, Button, Radiobutton, StringVar
from uav_land.msg import controllers_gain

class ControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Configuração de Controladores")
        self.entries = {}
        self.par_pub = rospy.Publisher('/PID/parameters', controllers_gain, queue_size=10)
        self.gains = controllers_gain()
        self.package_path = rospkg.RosPack().get_path("uav_land")

        self.controller_mode = StringVar()
        self.controller_mode.set("PD")

        self.create_widgets()
        self.load_gains()

    def create_widgets(self):
        
        pd_radio = Radiobutton(self.root, text="PD", variable=self.controller_mode, value="PD")
        pd_radio.grid(row=0, column=0, padx=10, pady=5, sticky="w")
        pid_radio = Radiobutton(self.root, text="PID", variable=self.controller_mode, value="PID")
        pid_radio.grid(row=0, column=10, padx=10, pady=5, sticky="w")

        cascade_radio = Radiobutton(self.root, text="Cascade", variable=self.controller_mode, value="Cascade")
        cascade_radio.grid(row=10, column=0, padx=10, pady=5, sticky="w")
        parallel_radio = Radiobutton(self.root, text="Parallel", variable=self.controller_mode, value="Parallel")
        parallel_radio.grid(row=10, column=10, padx=10, pady=5, sticky="w")
        
        self.create_gain_entries("PD", ["P", "D"], 2, 0)
        self.create_gain_entries("PID", ["P", "I", "D"], 2, 11)
        self.create_gain_entries("Cascade", ["pdP", "pdD", "piP", "piI"], 10, 0)
        self.create_gain_entries("Parallel", ["pdP", "pdD", "piP", "piI"], 10, 11)

        self.submit_button = Button(self.root, text="Aplicar", command=self.apply_gains)
        self.submit_button.grid(row=18, column=0, columnspan=15, padx=10, pady=10)

        self.save_button = Button(self.root, text="save", command=self.save_gains)
        self.save_button.grid(row=18, column=2, columnspan=15, padx=10, pady=10)

    def create_gain_entries(self, controller_type, gains_Text, row_start, column_start):
        dimensions = ["x", "y", "z", "yaw"]

        for i, gain in enumerate(gains_Text):
            label = Label(self.root, text=gain)
            label.grid(row=row_start + i + 1, column=column_start, sticky="e")

        for j, dimension in enumerate(dimensions):
            label_dimension = Label(self.root, text=dimension)
            label_dimension.grid(row=row_start, column= column_start + j + 1)

            for i, gain in enumerate(gains_Text):
                entry = Entry(self.root, width=5)
                entry.insert(0, "0.0")
                entry.grid(row=row_start + i + 1, column= column_start + j + 1, padx=5, pady=5)
                self.entries[f"{controller_type}_{gain}_{dimension}"] = entry

    def save_gains(self):
        file_name = self.package_path + "/parameters/gains.txt"
        print(file_name)

        with open(file_name, 'w') as file:
            for key, entry in self.entries.items():
                value = entry.get()
                file.write(f"{key}: {value}\n")
            file.write(f"controller_mode: {self.controller_mode.get()}\n")

    def load_gains(self):
        file_name = self.package_path + "/parameters/gains.txt"
        try:
            with open(file_name, 'r') as file:
                lines = file.readlines()
                for line in lines:
                    key, value = line.strip().split(":")
                    if key == "controller_mode":
                        self.controller_mode.set(value.strip())
                        continue
                    self.entries[key].delete(0, "end")
                    self.entries[key].insert(0, value.strip())
        except FileNotFoundError:
            print("Gains file not found. Using default values.")
            pass

    def apply_gains(self):
        self.gains.pd_ctrl.x.p_gain   = float(self.get_entry("PD", "P", "x"  ).get())
        self.gains.pd_ctrl.x.d_gain   = float(self.get_entry("PD", "D", "x"  ).get())
        self.gains.pd_ctrl.y.p_gain   = float(self.get_entry("PD", "P", "y"  ).get())
        self.gains.pd_ctrl.y.d_gain   = float(self.get_entry("PD", "D", "y"  ).get())
        self.gains.pd_ctrl.z.p_gain   = float(self.get_entry("PD", "P", "z"  ).get())
        self.gains.pd_ctrl.z.d_gain   = float(self.get_entry("PD", "D", "z"  ).get())
        self.gains.pd_ctrl.yaw.p_gain = float(self.get_entry("PD", "P", "yaw").get())
        self.gains.pd_ctrl.yaw.d_gain = float(self.get_entry("PD", "D", "yaw").get())

        self.gains.pid_ctrl.x.p_gain   = float(self.get_entry("PID", "P", "x"  ).get())
        self.gains.pid_ctrl.x.i_gain   = float(self.get_entry("PID", "I", "x"  ).get())
        self.gains.pid_ctrl.x.d_gain   = float(self.get_entry("PID", "D", "x"  ).get())
        self.gains.pid_ctrl.y.p_gain   = float(self.get_entry("PID", "P", "y"  ).get())
        self.gains.pid_ctrl.y.i_gain   = float(self.get_entry("PID", "I", "y"  ).get())
        self.gains.pid_ctrl.y.d_gain   = float(self.get_entry("PID", "D", "y"  ).get())
        self.gains.pid_ctrl.z.p_gain   = float(self.get_entry("PID", "P", "z"  ).get())
        self.gains.pid_ctrl.z.i_gain   = float(self.get_entry("PID", "I", "z"  ).get())
        self.gains.pid_ctrl.z.d_gain   = float(self.get_entry("PID", "D", "z"  ).get())
        self.gains.pid_ctrl.yaw.p_gain = float(self.get_entry("PID", "P", "yaw").get())
        self.gains.pid_ctrl.yaw.i_gain = float(self.get_entry("PID", "I", "yaw").get())
        self.gains.pid_ctrl.yaw.d_gain = float(self.get_entry("PID", "D", "yaw").get())

        self.gains.cascade_ctrl.x.pd_ctrl.p_gain = float(self.get_entry("Cascade", "pdP", "x").get())
        self.gains.cascade_ctrl.x.pd_ctrl.d_gain = float(self.get_entry("Cascade", "pdD", "x").get())
        self.gains.cascade_ctrl.x.pi_ctrl.p_gain = float(self.get_entry("Cascade", "piP", "x").get())
        self.gains.cascade_ctrl.x.pi_ctrl.i_gain = float(self.get_entry("Cascade", "piI", "x").get())

        self.gains.cascade_ctrl.y.pd_ctrl.p_gain = float(self.get_entry("Cascade", "pdP", "y").get())
        self.gains.cascade_ctrl.y.pd_ctrl.d_gain = float(self.get_entry("Cascade", "pdD", "y").get())
        self.gains.cascade_ctrl.y.pi_ctrl.p_gain = float(self.get_entry("Cascade", "piP", "y").get())
        self.gains.cascade_ctrl.y.pi_ctrl.i_gain = float(self.get_entry("Cascade", "piI", "y").get())

        self.gains.cascade_ctrl.z.pd_ctrl.p_gain = float(self.get_entry("Cascade", "pdP", "z").get())
        self.gains.cascade_ctrl.z.pd_ctrl.d_gain = float(self.get_entry("Cascade", "pdD", "z").get())
        self.gains.cascade_ctrl.z.pi_ctrl.p_gain = float(self.get_entry("Cascade", "piP", "z").get())
        self.gains.cascade_ctrl.z.pi_ctrl.i_gain = float(self.get_entry("Cascade", "piI", "z").get())

        self.gains.cascade_ctrl.yaw.pd_ctrl.p_gain = float(self.get_entry("Cascade", "pdP", "yaw").get())
        self.gains.cascade_ctrl.yaw.pd_ctrl.d_gain = float(self.get_entry("Cascade", "pdD", "yaw").get())
        self.gains.cascade_ctrl.yaw.pi_ctrl.p_gain = float(self.get_entry("Cascade", "piP", "yaw").get())
        self.gains.cascade_ctrl.yaw.pi_ctrl.i_gain = float(self.get_entry("Cascade", "piI", "yaw").get())

        self.gains.paralel_ctrl.x.pd_ctrl.p_gain = float(self.get_entry("Parallel", "pdP", "x").get())
        self.gains.paralel_ctrl.x.pd_ctrl.d_gain = float(self.get_entry("Parallel", "pdD", "x").get())
        self.gains.paralel_ctrl.x.pi_ctrl.p_gain = float(self.get_entry("Parallel", "piP", "x").get())
        self.gains.paralel_ctrl.x.pi_ctrl.i_gain = float(self.get_entry("Parallel", "piI", "x").get())

        self.gains.paralel_ctrl.y.pd_ctrl.p_gain = float(self.get_entry("Parallel", "pdP", "y").get())
        self.gains.paralel_ctrl.y.pd_ctrl.d_gain = float(self.get_entry("Parallel", "pdD", "y").get())
        self.gains.paralel_ctrl.y.pi_ctrl.p_gain = float(self.get_entry("Parallel", "piP", "y").get())
        self.gains.paralel_ctrl.y.pi_ctrl.i_gain = float(self.get_entry("Parallel", "piI", "y").get())

        self.gains.paralel_ctrl.z.pd_ctrl.p_gain = float(self.get_entry("Parallel", "pdP", "z").get())
        self.gains.paralel_ctrl.z.pd_ctrl.d_gain = float(self.get_entry("Parallel", "pdD", "z").get())
        self.gains.paralel_ctrl.z.pi_ctrl.p_gain = float(self.get_entry("Parallel", "piP", "z").get())
        self.gains.paralel_ctrl.z.pi_ctrl.i_gain = float(self.get_entry("Parallel", "piI", "z").get())

        self.gains.paralel_ctrl.yaw.pd_ctrl.p_gain = float(self.get_entry("Parallel", "pdP", "yaw").get())
        self.gains.paralel_ctrl.yaw.pd_ctrl.d_gain = float(self.get_entry("Parallel", "pdD", "yaw").get())
        self.gains.paralel_ctrl.yaw.pi_ctrl.p_gain = float(self.get_entry("Parallel", "piP", "yaw").get())
        self.gains.paralel_ctrl.yaw.pi_ctrl.i_gain = float(self.get_entry("Parallel", "piI", "yaw").get())

        self.gains.mode = 0
        if self.controller_mode.get() == "PD":
            self.gains.mode = 1
        if self.controller_mode.get() == "PID":
            self.gains.mode = 2
        elif self.controller_mode.get() == "Cascade":
            self.gains.mode = 3
        elif self.controller_mode.get() == "Parallel":
            self.gains.mode = 4


        self.par_pub.publish(self.gains)

    def get_entry(self, controller_type, gain, dimension):
        return self.entries[f"{controller_type}_{gain}_{dimension}"]

def main():
    teste = controllers_gain()

    rospy.init_node('controller_gui_node', anonymous=True)
    root = tk.Tk()
    app = ControllerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
