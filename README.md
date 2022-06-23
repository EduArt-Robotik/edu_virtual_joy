# edu_virtual_joy
This package comprises a ROS joy interface for a keyboard steering concept. Install pygame on your host machine in order to launch the virtual joy node:

```console
sudo apt install python3-pygame
```

Ensure to have configured the ROS communication well:
```console
export ROS_MASTER_URI=http://<IP_OF_IOTBOT>:11311
export ROS_IP=<HOST_IP>
```
Launch the virtual joy node as follows:
```console
python3 src/edu_virtual_joy/scripts/edu_virtual_joy_node.py
```
This will provide the skid steering concept. Translation to the left and right side will be disabled, since the drives are seriously strained, if no mecanum wheels are mounted.

<img src="/images/gui_skid.png" alt="GUI skid steering" width="320"/>

To enable this feature for mecanum-driven robots, one can add the according parameter as follows:
```console
python3 src/edu_virtual_joy/scripts/edu_virtual_joy_node.py _mecanum:=1
```
<img src="/images/gui_mecanum.png" alt="GUI mecanum steering" width="320"/>

The key mapping can be found in the following table:

| Key    | Function             |
| ------ |:--------------------:|
| e      | Enable robot         |
| 0      | Disable robot        |
| w      | Move forward         |
| a      | Turn left            |
| s      | Move backward        |
| d      | Turn right           |
| y      | Move left (mecanum)  |
| c      | Move right (mecanum) |
| +      | Increase throttle    |
| -      | Decrease throttle    |
| 1      | Beam light           |
| 2      | Warning light        |
| 3      | Flash left           |
| 4      | Flash right          |
| 5      | Rotational light     |
| 6      | Running light        |

**Important:** Three conditions must be met before the robot moves. First, the stop button must not be pressed. Second, the charger must not be connected. And third, the enable bit must be set.

## Display of sensor information
This GUI visualizes some of the sensor information the robot provides. One can see the four distance measurements, which are integrated in the lighting concept. Additionally, the rotational speed of all wheels are given.
