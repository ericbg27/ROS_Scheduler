# ROS_Scheduler
EDF message scheduler module for Robot Operating System (ROS)

+ Entirely made in C++.
+ Test module implemented to test functionality.
+ User must define the following scheduler parameters:
  * _frequency (Hz):=
  * _timeout (ms):=
+ For modules:
  * _frequency (Hz):=
  * _deadline (μs):=
  * _wce (ms):=
  * __name:= (optional if module number is an one to one relationship with executable number, mandatory if not)
+ Scheduler must be launched before. It doesn't schedule modules declared beforehand.
+ Module basic implementation is located in moduletemplate folder. Add code as necessary but be sure not to erase the code already given.
