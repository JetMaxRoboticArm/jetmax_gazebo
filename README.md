# jetmax_gazebo

## Forward/Inverse kinematics

```
roslaunch jetmax_control jetmax_control.launch
```

1. Homing

   ```
   rosrun jetmax_control go_home.py
   ```

   

2. Forward kinematics

      ```
      rosservice call /jetmax_control/forward_kinematics "x: 0.0
      
      y: -100.0
      
      z: 100.0"
      ```

      

3. Inverse kinematics

      ```
      rosservice call /jetmax_control/inverse_kinematics "angle_rotate: 90.0
      
      angle_left: 90.0
      
      angle_right: 0.0
      ```
      
You can use tab key to quickly complete commands