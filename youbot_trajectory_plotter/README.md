# youbot_trajectory_plotter

catkin_make and run the node with ```rosrun youbot_trajectory_plotter youbot_trajectory_plotter```. The node listens to the topic which you can edit in "src/youbot_trajectory_plotter.py".

In test without real youbot, run the node fake publisher with ```rosrun youbot_trajecotry_plotter fake_publisher.py```. 

**NB: It's a bit buggy with matplotlib figure**, i.e., when a figure a plotted, the node cannot be stopped with "ctrl+c". Solution may be from threading.

-> find out the process with ```ps aux | grep "youbot_trajectory_plotter```, and kill with ```kill -9 XXXXX``` (where XXXXX is the process id).
