# ROS_Scheduler
EDF message scheduler module for Robot Operating System (ROS)

+ Entirely made in C++
+ Test module implemented to test functionality
+ User must define the following scheduler parameters:
  * _frequency:=
  * _timeout:=
+ For modules:
  * _frequency:=
  * _deadline:=
  * _wce:=
  * _name:= (optional if module number is an one to one relationship with executable number, mandatory if not)
+ Scheduler must be launched before. It doesn't schedule modules declared before
+ Some code must be implmented in modules, these codes are in the Test Module module
