# Gazebo
Here are some gazebo projects I used to make some simulated data.

Worldfiles/ 
contain the files that are run with gazebo. They contain the specification of the current simulation. Cameraspawned creates a new world by spawning new objects. Camera_turned, spawns all objects but all the ones that are not needed are turned upside down.
$gzserver name_world.world --verbose
in a new window
$gzclient

Worldplugin/
plugins that load the different objects / surroundings / modelplugins according to the world file.
