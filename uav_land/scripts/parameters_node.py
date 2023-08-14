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
        cascade_radio = Radiobutton(self.root, text="Cascade", variable=self.controller_mode, value="cascade")
        cascade_radio.grid(row=5, column=0, padx=10, pady=5, sticky="w")
        parallel_radio = Radiobutton(self.root, text="Parallel", variable=self.controller_mode, value="parallel")
        parallel_radio.grid(row=11, column=0, padx=10, pady=5, sticky="w")

        self.create_gain_entries("PD", ["P", "D"], 2)
        self.create_gain_entries("Cascade", ["pdP", "pdD", "piP", "piI"], 6)
        self.create_gain_entries("Parallel", ["pdP", "pdD", "piP", "piI"], 12)

        self.submit_button = Button(self.root, text="Aplicar", command=self.apply_gains)
        self.submit_button.grid(row=17, column=0, columnspan=15, padx=10, pady=10)

        self.save_button = Button(self.root, text="save", command=self.save_gains)
        self.save_button.grid(row=17, column=2, columnspan=15, padx=10, pady=10)

    def create_gain_entries(self, controller_type, gains_Text, row_start):
        dimensions = ["x", "y", "z", "yaw"]

        for i, gain in enumerate(gains_Text):
            label = Label(self.root, text=gain)
            label.grid(row=row_start + i + 1, column=0, sticky="e")

        for j, dimension in enumerate(dimensions):
            label_dimension = Label(self.root, text=dimension)
            label_dimension.grid(row=row_start, column=j + 1)

            for i, gain in enumerate(gains_Text):
                entry = Entry(self.root, width=5)
                entry.insert(0, "0.0")
                entry.grid(row=row_start + i + 1, column=j + 1, padx=5, pady=5)
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

        self.gains.cascade_ctrl.x.pd_ctrl.p_gain = float(self.get_entry("PDPI cascade", "pdP", "x").get())
        self.gains.cascade_ctrl.x.pd_ctrl.d_gain = float(self.get_entry("PDPI cascade", "pdD", "x").get())
        self.gains.cascade_ctrl.x.pi_ctrl.p_gain = float(self.get_entry("PDPI cascade", "piP", "x").get())
        self.gains.cascade_ctrl.x.pi_ctrl.i_gain = float(self.get_entry("PDPI cascade", "piI", "x").get())

        self.gains.cascade_ctrl.y.pd_ctrl.p_gain = float(self.get_entry("PDPI cascade", "pdP", "y").get())
        self.gains.cascade_ctrl.y.pd_ctrl.d_gain = float(self.get_entry("PDPI cascade", "pdD", "y").get())
        self.gains.cascade_ctrl.y.pi_ctrl.p_gain = float(self.get_entry("PDPI cascade", "piP", "y").get())
        self.gains.cascade_ctrl.y.pi_ctrl.i_gain = float(self.get_entry("PDPI cascade", "piI", "y").get())

        self.gains.cascade_ctrl.z.pd_ctrl.p_gain = float(self.get_entry("PDPI cascade", "pdP", "z").get())
        self.gains.cascade_ctrl.z.pd_ctrl.d_gain = float(self.get_entry("PDPI cascade", "pdD", "z").get())
        self.gains.cascade_ctrl.z.pi_ctrl.p_gain = float(self.get_entry("PDPI cascade", "piP", "z").get())
        self.gains.cascade_ctrl.z.pi_ctrl.i_gain = float(self.get_entry("PDPI cascade", "piI", "z").get())

        self.gains.cascade_ctrl.yaw.pd_ctrl.p_gain = float(self.get_entry("PDPI cascade", "pdP", "yaw").get())
        self.gains.cascade_ctrl.yaw.pd_ctrl.d_gain = float(self.get_entry("PDPI cascade", "pdD", "yaw").get())
        self.gains.cascade_ctrl.yaw.pi_ctrl.p_gain = float(self.get_entry("PDPI cascade", "piP", "yaw").get())
        self.gains.cascade_ctrl.yaw.pi_ctrl.i_gain = float(self.get_entry("PDPI cascade", "piI", "yaw").get())

        self.gains.paralel_ctrl.x.pd_ctrl.p_gain = float(self.get_entry("PDPI paralel", "pdP", "x").get())
        self.gains.paralel_ctrl.x.pd_ctrl.d_gain = float(self.get_entry("PDPI paralel", "pdD", "x").get())
        self.gains.paralel_ctrl.x.pi_ctrl.p_gain = float(self.get_entry("PDPI paralel", "piP", "x").get())
        self.gains.paralel_ctrl.x.pi_ctrl.i_gain = float(self.get_entry("PDPI paralel", "piI", "x").get())

        self.gains.paralel_ctrl.y.pd_ctrl.p_gain = float(self.get_entry("PDPI paralel", "pdP", "y").get())
        self.gains.paralel_ctrl.y.pd_ctrl.d_gain = float(self.get_entry("PDPI paralel", "pdD", "y").get())
        self.gains.paralel_ctrl.y.pi_ctrl.p_gain = float(self.get_entry("PDPI paralel", "piP", "y").get())
        self.gains.paralel_ctrl.y.pi_ctrl.i_gain = float(self.get_entry("PDPI paralel", "piI", "y").get())

        self.gains.paralel_ctrl.z.pd_ctrl.p_gain = float(self.get_entry("PDPI paralel", "pdP", "z").get())
        self.gains.paralel_ctrl.z.pd_ctrl.d_gain = float(self.get_entry("PDPI paralel", "pdD", "z").get())
        self.gains.paralel_ctrl.z.pi_ctrl.p_gain = float(self.get_entry("PDPI paralel", "piP", "z").get())
        self.gains.paralel_ctrl.z.pi_ctrl.i_gain = float(self.get_entry("PDPI paralel", "piI", "z").get())

        self.gains.paralel_ctrl.yaw.pd_ctrl.p_gain = float(self.get_entry("PDPI paralel", "pdP", "yaw").get())
        self.gains.paralel_ctrl.yaw.pd_ctrl.d_gain = float(self.get_entry("PDPI paralel", "pdD", "yaw").get())
        self.gains.paralel_ctrl.yaw.pi_ctrl.p_gain = float(self.get_entry("PDPI paralel", "piP", "yaw").get())
        self.gains.paralel_ctrl.yaw.pi_ctrl.i_gain = float(self.get_entry("PDPI paralel", "piI", "yaw").get())

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
