# HDZ MoveIt Assistant

This package is intended to use MoveIt Setup Assistant for collision matrix generation.

Collision matrix needs to be re-generated if the mechanical structure of the robot has been changed.

## How to generate collision matrix

1. Build and source workspace

2. Launch MoveIt Setup Assistant:

   ```shell
   ros2 launch hdz_moveit_assistant setup_assistant.launch.py 
   ```

3. Click on `Load Files`

4. Pull the slider of `Sampling Density` to the maximum and click on `Generate Collision Matrix`

5. Carefully exam the generated list whose `Reason to Disable` is `Collision by Default`. Uncheck those unneeded.

6. Click on `Configutation Files` on the left and check only the `hdz_full.srdf` file
7. Click on `Generate Package`
8. Copy all `<disable_collisions ... />` lines in `hdz_moveit_assistant/config/hdz_full.srdf` to `hdz_moveit_config/srdf/hdz_macro.srdf.xacro`

