This package project provides a way to supplement map-based navigation on ROS using basic visual cues. These visual supplementations
are basic shapes with solid colors that can be detected by an on-board webcam. Once detected these signs can then be reacted to
by the TurtleBot.

Setup requirements:

1. Download sticky_wickets package folder into your catkin_ws on your workstation and the turtlebot. 
2. Ensure the network between the turtlebot and your workstation is setup properly. 
  

Launch files (in launch sequence):

1. core.launch [run on turtlebot] 
2. viewing.launch [run on workstation] 
3. sticky_wickets.launch [run on turtlebot] 
  

Quickstart:

1. Ensure you have the sticky_wickets package on your workstation AND turtlebot. 
2. You may put your maps in the sticky_wickets/map/ directory.
3. Set up signs around the turtlebot. Current detected colors are green, orange, and blue. Detected shapes are circles, triangles, and squares. We have made some signs already using colored stickers. 
4. On the TURTLEBOT, run the core minimal and navigation requirements using the launch file core.launch with the yaml file for your desired map. Do not forget to edit your yaml to ensure it points to the PGM in the correct directory: 
    1. roslaunch sticky_wickets core.launch map:=/your/map.yaml 

5. On the WORKSTATION, run the viewing.launch file, which runs rviz and image_views subscribing to the topics we will publish from our nodes: 
    1. roslaunch sticky_wickets viewing.launch 

6. You should see rviz load with the map and the turtlebot localized. The image_views will be blank because they are subscribing to topics that have not yet been published. These topics will be published when the next launch file is launched. They should show (1) a blank white image map, where red dots will appear when landmarks are discovered, and (2) a masked image visualizing detected signs. 
7. Now you can use rviz to provide a 2D Pose Estimate for the turtlebot’s location. 
8. Once localized, on the TURTLEBOT, launch sticky_wickets.launch to launch the moving, sign handling, and object detection nodes: 
    1. roslaunch sticky_wickets sticky_wickets.launch 

9. The turtlebot should begin wandering and the system should be working. 
  
  
  
  
  

Known Issues:

1. If the turtlebot is having trouble moving at all, or intermittently stops moving, this is likely due to the astra’s erroneous costmap, which will prevent the turtlebot from moving or rotating in place. This is unfixable. The only known solution is to restart the package. 
2. If signs are being detected incorrectly or not detected at all, try moving them to better lit areas. Note that the Kinect’s camera gave poorer object detection results due to high image granularity. It is not recommended for use. 
3. The Astra sometimes does not reliably get depth data for the signs, and does not work within 0.5m of the signs. No known fix at the moment, keeping the signs between 0.5 and 1.0m seems to work best.
