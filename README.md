		GraspIt! HelloWorld Plugin

In the plugin directory
```
export GRASPIT_PLUGIN_DIR=/path_to_plugin_dir/lib 
```

in Graspit, run 

```
./bin/graspit -p libthenameoftheplugin
```


Introduction
------------
This plugin serves as an introduction to writing plugins for the GraspIt! grasping simulator.
It should be used as an example and to test your system configuration.

Disclaimer
----------
This is code that has been, and is currently being used, for research. There 
are still many unfinished pieces and plenty of ways to crash the system.  It's
by no means bullet proof. Please see the Introduction in the User Manual for 
more details.



Plugin Contests
---------------
helloWorldPlugin.pro
helloWorldPlugin.h
helloWorldPlugin.cpp

To compile, run qmake helloWorldPlugin.pro to generate a build file, then compile the build file. 



Troubleshooting
---------------
Ensure that you have compiled the plugin and GraspIt! using the same configuration mode [Release/Debug] or you will 
have segmentation errors due to a mismatch between the standard libraries on Windows.

On Windows, if your library uses the graspitGui GraspItGui class singleton to access the ivmanager or world, make sure to link against $(GRASPIT)/bin/graspit.lib. On Linux, there is no need to link against GraspIt!.

If you change the name of your graspit binary in your compilation environment, the name of the lib file will also change, and you will need to
recompile your plugin.

