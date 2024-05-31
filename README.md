

This repository contains the implementation of a visual servoing algorithm for a 2-DOF robot.


### Step 1: Spawning the object on the ground and get the coordinates of the four features.
Initial Placement of the robot and detecting the center of each feature

1. **Spawn Object:** 
   - Spawn the object created on the ground within the robot's workspace.

2. **Move Robot:**
   - Move the robot via the position controller to ensure the entire object is visible in the image.

3. **Image Capture:**
   - Capture an image and obtain the coordinates of the 4 circle centers.

4. **Output:**

   - Present the captured image from the virtual camera.
   - Display the detected circle centers.
   
<p align="center"><img width="540" alt="Screenshot 2024-05-31 at 8 33 13 AM" src="https://github.com/AnoushkaBaidya/Visual-Servoing-of-a-2-DOF-Robotic-Manipulator/assets/115124698/a8631f15-42e0-4ba3-a8fd-8d7d9cefb160"></p>


### Step 2: Robot Relocation and Image Capture

1. **Relocate Robot:**
   - Move the robot to a different location using the position controller.

2. **Ensure Visibility:**
   - Ensure the entire object is still visible by the virtual camera.

3. **New Image Capture:**
   - Capture a new image and obtain the coordinates of the 4 circle centers.

4. **Output:**
   - Present the new captured image from the virtual camera.
   - Display the detected circle centers in the new location.


<p align="center"><img width="621" alt="Screenshot 2024-05-31 at 8 37 11 AM" src="https://github.com/AnoushkaBaidya/Visual-Servoing-of-a-2-DOF-Robotic-Manipulator/assets/115124698/3e06fa95-b0cf-4c5c-afb2-b34cefde3eca"></p>


### Step 3: Visual Servoing Algorithm Implementation 

1. **Algorithm Implementation:**
   - Implement a visual servoing algorithm using the four point features (centers of the circles).

2. **Controller Switch:**
   - Switch the controller to velocity mode to execute the velocity command.

3. **Image Jacobian:**
   - In the image Jacobian, use f and Z, both set to 1.

4. **Note:**
   - All parameters are multiplied with a gain (lambda) in this 2D example and do not have a direct effect.

5. **Output:**
   - Record the locations (x, y coordinates) of all features over time during visual servoing.
   - Plot the trajectories of the features in the XY plane.
  
  
  https://github.com/AnoushkaBaidya/Visual-Servoing-of-a-2-DOF-Robotic-Manipulator/assets/115124698/268ab1d1-29d3-4137-9643-4028651a002a
  
  https://github.com/AnoushkaBaidya/Visual-Servoing-of-a-2-DOF-Robotic-Manipulator/assets/115124698/bc9aacd7-e302-4abf-812d-1af5c3b1ab7e
