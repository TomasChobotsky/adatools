#!/usr/bin/env python

from adatools import config_generator as cg
from adatools import plotting_tools as pt


my_conf1_robot = cg.get_robot_config_1(link1=0.145, link1_offset=0.0,
                                       link2=0.310, link2_offset=0.0,
                                       link3=0.275, link3_offset=0.0,
                                       link4=0.100, link4_offset=0.0)

# Plot the robot on base plate
robot_plot = my_conf1_robot.plot(my_conf1_robot.q, backend='pyplot')
pt.plot_baseplate(robot_plot)
# robot_plot.hold()

# Jog the robot on base plate
robot_teach = my_conf1_robot.teach(my_conf1_robot.q, backend='pyplot', block=False)
pt.plot_baseplate(robot_teach)
robot_teach.hold()