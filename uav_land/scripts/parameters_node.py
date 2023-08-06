#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import PySimpleGUI as sg

file_path = "vlue.txt"
rospy.init_node('array_publisher', anonymous=True)
pub = rospy.Publisher('/PID/parameters', Float32MultiArray, queue_size=10)
rate = rospy.Rate(1)

def publish_array(array):
    msg = Float32MultiArray(data=array)
    pub.publish(msg)
    rate.sleep()

array = [0,0,0,
         0,0,0,
         0,0,0,
         0,0,0]

with open(file_path, 'r') as file:
        array = [float(value) for value in file.read().split()]

# Define a layout for the GUI
layout = [
    [
        sg.Text('     X', font=('Helvetica', 16), justification='center'),
        sg.Text('         Y', font=('Helvetica', 16), justification='center'),
        sg.Text('         Z', font=('Helvetica', 16), justification='center'),
        sg.Text('         YAW', font=('Helvetica', 16), justification='center')
    ],
    [
        sg.Text('P:', font=('Helvetica', 12)), sg.InputText(key='P_X', size=(7, None), default_text=array[0]),
        sg.Text(' ', font=('Helvetica', 12)), sg.InputText(key='P_Y', size=(7, None), default_text=array[3]),
        sg.Text(' ', font=('Helvetica', 12)), sg.InputText(key='P_Z', size=(7, None), default_text=array[6]),
        sg.Text(' ', font=('Helvetica', 12)), sg.InputText(key='P_YAW', size=(7, None), default_text=array[9])
    ],
    [
        sg.Text('I:  ', font=('Helvetica', 12)), sg.InputText(key='I_X', size=(7, None), default_text=array[1]),
        sg.Text(' ', font=('Helvetica', 12)), sg.InputText(key='I_Y', size=(7, None), default_text=array[4]),
        sg.Text(' ', font=('Helvetica', 12)), sg.InputText(key='I_Z', size=(7, None), default_text=array[7]),
        sg.Text(' ', font=('Helvetica', 12)), sg.InputText(key='I_YAW', size=(7, None), default_text=array[10]),
    ],
    [
        sg.Text('D:', font=('Helvetica', 12)), sg.InputText(key='D_X', size=(7, None), default_text=array[2]),
        sg.Text(' ', font=('Helvetica', 12)), sg.InputText(key='D_Y', size=(7, None), default_text=array[5]),
        sg.Text(' ', font=('Helvetica', 12)), sg.InputText(key='D_Z', size=(7, None), default_text=array[8]),
        sg.Text(' ', font=('Helvetica', 12)), sg.InputText(key='D_YAW', size=(7, None), default_text=array[11]),
    ],
    [sg.Button('set', size=(10, 1)), sg.Button('save', size=(10, 1))]
]

# Create the window
window = sg.Window('Array Publisher', layout)

while not rospy.is_shutdown():
    event, values = window.read()

    if event == sg.WINDOW_CLOSED:
        break

    if event == 'save':
        with open(file_path, 'w') as file:
            file.write(' '.join(str(value) for value in array))

    if event == 'set':
        array = [float(values['P_X']),float(values['I_X']),float(values['D_X']),
                 float(values['P_Y']),float(values['I_Y']),float(values['D_Y']),
                 float(values['P_Z']),float(values['I_Z']),float(values['D_Z']),
                 float(values['P_YAW']),float(values['I_YAW']),float(values['D_YAW']),
                 ]

        try:
            publish_array(array)
        except rospy.ROSInterruptException:
            pass

# Close the GUI window
window.close()
