import rospy
import tkinter as tk
from tkinter import Label, Entry, Button
from uav_land.msg import controllers_gain

class ControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Configuração de Controladores")
        self.entries = {}
        self.par_pub = rospy.Publisher('/PID/parameters', controllers_gain, queue_size=10)
        self.gains = controllers_gain()

        self.create_widgets()

    def create_widgets(self):
        self.label_pd = Label(self.root, text="PD")
        self.label_pd.grid(row=0, column=0, columnspan=1, padx=10, pady=10)
        self.create_pd_gain_entries("PD", 1)

        self.label_pdpi1 = Label(self.root, text="PDPI cascade")
        self.label_pdpi1.grid(row=4, column=0, columnspan=1, padx=10, pady=10)
        self.create_pdpi_gain_entries("PDPI cascade", 5)

        self.label_pdpi2 = Label(self.root, text="PDPI paralel")
        self.label_pdpi2.grid(row=10, column=0, columnspan=1, padx=10, pady=10)
        self.create_pdpi_gain_entries("PDPI paralel", 11)

        self.submit_button = Button(self.root, text="Aplicar", command=self.apply_gains)
        self.submit_button.grid(row=16, column=0, columnspan=15, padx=10, pady=10)

    def create_pd_gain_entries(self, controller_type, row_start):
        dimensions = ["x", "y", "z", "yaw"]

        for j, dimension in enumerate(dimensions):
            label_dimension = Label(self.root, text=dimension)
            label_dimension.grid(row=row_start, column=j * 4 + 1, padx=10, pady=5)

            for i, gain in enumerate(["P", "D"]):
                label = Label(self.root, text=gain)
                label.grid(row=row_start + i + 1, column=j * 4, padx=10, pady=5)

                entry = Entry(self.root, width=5)
                entry.insert(0, "0.0")
                entry.grid(row=row_start + i + 1, column=j * 4 + 1, padx=5, pady=5)
                self.entries[f"{controller_type}_{gain}_{dimension}"] = entry
                
    def create_pdpi_gain_entries(self, controller_type, row_start):
        dimensions = ["x", "y", "z", "yaw"]

        for j, dimension in enumerate(dimensions):
            label_dimension = Label(self.root, text=dimension)
            label_dimension.grid(row=row_start, column=j * 4 + 1, padx=10, pady=5)

            for i, gain in enumerate(["pdP", "pdD", "piP", "piI"]):
                label = Label(self.root, text=gain)
                label.grid(row=row_start + i + 1, column=j * 4, padx=10, pady=5)

                entry = Entry(self.root, width=5)
                entry.insert(0, "0.0")
                entry.grid(row=row_start + i + 1, column=j * 4 + 1, padx=5, pady=5)
                self.entries[f"{controller_type}_{gain}_{dimension}"] = entry

    def apply_gains(self):
        dimensions = ["x", "y", "z", "yaw"]

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

        print("-----")
        print(self.gains)

        # self.par_pub.pub(self.gains)
        self.par_pub.publish(self.gains)
        

        # for controller_type in ["PD", "PDPI cascade", "PDPI paralel"]:
        #     print(controller_type)
        #     for gain in ["P", "I", "D"]:
        #         for dimension in dimensions:
                    # entry = self.get_entry(controller_type, gain, dimension)
        #             gain_value = float(entry.get())
        #             rospy.loginfo(f"Ganho {gain} ({controller_type}) {dimension}: {gain_value}")

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
