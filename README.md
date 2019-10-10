# ROS_Scheduler
EDF message scheduler module for Robot Operating System (ROS)

+ Entirely made in C++.
+ Test module implemented to test functionality.
+ User must define the following scheduler parameters:
  * _frequency:= (Hz)
  * _timeout:= (ms)
+ For modules:
  * _frequency:= (Hz)
  * _deadline:= (ns)
  * _wce:=  (μs)
  * __name:= (optional if module number is an one to one relationship with executable number, mandatory if not)
+ Scheduler must be launched before. It doesn't schedule modules declared beforehand.
+ Module basic implementation is located in moduletemplate folder. Add code as necessary but be sure not to erase the code already given.
+ Multithread version improved functionality
+ Keep ROS master node alive during execution
+ check_frequency parameter for modules define the real frequency in which thread wakes up, to check if it is scheduled. Frequency defines the frequency in which the thread will execute it's main functionality.
  *By default check_frequency is defined 2/(deadline/1000000), adjusting it to x will assign the value x/(deadline/1000000)
