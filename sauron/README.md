# Sauron

Oversees the arm, and controls it from the shadows (if the shadows is a custom
RViz GUI)

For now its a pannel in RViz that is used to control the arm's unique stuff, and
for now (as of 11/24) just the homing button for the stepper motors.

Thank you to [this code here](https://github.com/BruceChanJianLe/rviz2-panel/tree/master) for most (all) of the code for this 


## Rough Description of files
`start.launch.py`
: It's important to note that _there is nothing special about the launchfile!_
When the package is compiled, is runs this command at the bottom of
`CMakeLists.txt`, which installs the plugin to RViz2.
```
pluginlib_export_plugin_description_file(rviz_common rviz2_plugin.xml)
```
The custom panel can be added to your view by clicking `Panel` -> `Add new panel` and it should appear under a folder called `sauron`


`rviz2_plugin.xml`
: This links it all together. The CmakeLists.txt runs that special command which uses this file to install the panel to RViz.
This file links the front and back end (I think)


`push_button.ui`
: the front end design. Think of it sort of like HTML. STRONGLY reccommend
editing with the QTDesigner, its just so much easier. Its a drag-and-drop, gui
based, gui editor.


`rviz2_pannel.[cpp/hpp]`
: These files are the back end of the code. This is where the node and
publishers are made.

