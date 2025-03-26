
> 🚧 **Work in Progress**: This tutorial series is under active development. Currently, only the Camera Bridge tutorial is available. More tutorials will be added soon!

# Bridging Camera Feeds from Gazebo to ROS 2 in PX4 Simulations

This tutorial explains how to properly bridge camera feeds from a PX4 simulation in Gazebo to ROS 2. This is essential for developing computer vision applications, including visual navigation systems that use camera inputs.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Garden or Gazebo Harmonic
- PX4-Autopilot (latest version)
- ROS-GZ bridge packages

### Install Required Packages

```bash
# Install ROS-GZ bridge packages
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-image

# Install image visualization tools
sudo apt install ros-humble-rqt-image-view ros-humble-image-view ros-humble-image-transport-plugins
```

## Step 1: Launch PX4 Simulation with a Camera-Equipped Model

Several PX4 models come with cameras. The `x500_depth` is recommended as it includes both RGB and depth cameras:

```bash
# Navigate to PX4 directory
cd ~/PX4-Autopilot

# Launch simulation with camera-equipped drone
make px4_sitl gz_x500_depth
```

Other models with cameras include:
- `gz_iris_vision`
- `gz_typhoon_h480`

## Step 2: Identify Camera Topics in Gazebo

Once the simulation is running, check for available camera topics:

```bash
# List all topics and filter for camera-related ones
gz topic -l | grep camera
```

You should see topics like:
- `/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image` (RGB camera)
- `/depth_camera` (Depth camera)

Verify that data is being published on these topics:

```bash
# Check RGB camera data
gz topic -e -t /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```

You should see binary data being output to the terminal, which confirms that the camera is publishing data.

## Step 3: Visualize Camera Feed in Gazebo (Optional but Recommended)

Before bridging to ROS 2, it's a good idea to verify that the camera is working properly in Gazebo:

1. In the Gazebo GUI, click on the "+" button in the top-right corner
2. Select "Image Display" from the plugins list
3. In the dropdown menu that appears in the new window, select your camera topic
4. You should now see the camera feed directly in Gazebo

![Gazebo Camera Visualization Example](images/gazebo_camera_view.png)
<!-- Add a screenshot of your Gazebo window showing the camera feed here -->

## Step 4: Bridge the Camera Feed to ROS 2

Now that we've confirmed the camera is working in Gazebo, let's bridge the feed to ROS 2:

```bash
# Bridge RGB camera (use the exact topic path from your gz topic -l output)
ros2 run ros_gz_bridge parameter_bridge '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image'
```

The format of this command is:
`ros2 run ros_gz_bridge parameter_bridge '<gazebo_topic>@<ros_msg_type>@<gz_msg_type>'`

You should see output similar to:
```
[INFO] [123456.789012345] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image (gz.msgs.Image) -> /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image (sensor_msgs/msg/Image)] (Lazy 0)
[INFO] [123456.789012345] [ros_gz_bridge]: Creating ROS->GZ Bridge: [/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image (sensor_msgs/msg/Image) -> /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image (gz.msgs.Image)] (Lazy 0)
```

## Step 5: Verify the Bridge is Working

Check if the camera topic is now available in ROS 2:

```bash
# List ROS topics to verify camera topic is available
ros2 topic list | grep image
```

Check the data flow rate:

```bash
# This should show a steady rate (usually 30Hz)
ros2 topic hz /world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image
```

If you see a consistent Hz rate, the bridge is working correctly.

## Step 6: View the Camera Feed in ROS 2

Now let's visualize the camera feed in ROS 2:

```bash
# Launch the ROS 2 image viewer
ros2 run rqt_image_view rqt_image_view
```

In the dropdown menu at the top of the rqt_image_view window, select your camera topic.

![RQT Image View Example](images/rqt_image_view.png)
<!-- Add a screenshot of rqt_image_view showing your camera feed here -->

If you see the camera feed, congratulations! You have successfully bridged your camera from Gazebo to ROS 2.

## Step 7: Using a YAML Configuration (Optional but Recommended)

For a cleaner setup, especially with multiple topics or to simplify long topic names, you can use a YAML configuration:

```bash
# Create a config directory if it doesn't exist
mkdir -p ~/bridge_config

# Create a YAML file for camera bridge configuration
cat > ~/bridge_config/camera_bridge.yaml << EOF
- gz_topic_name: "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image"
  ros_topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
  lazy: false

# Add depth camera bridge if needed
- gz_topic_name: "/depth_camera"
  ros_topic_name: "/camera/depth"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
  lazy: false
EOF

# Run the bridge with the YAML config
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$HOME/bridge_config/camera_bridge.yaml
```

Now you can access your camera via the simpler topic names:
- `/camera/image` for RGB
- `/camera/depth` for depth (if available)

```bash
# View with the simplified topic name
ros2 run rqt_image_view rqt_image_view /camera/image
```

## Common Issues and Troubleshooting

### 1. No Camera Topics in Gazebo

**Problem**: Running `gz topic -l | grep camera` doesn't show any camera topics.

**Solution**: 
- Ensure you're using a model that includes a camera (like `x500_depth` or `iris_vision`)
- Check if your simulation is properly loaded by looking at the Gazebo GUI
- Try restarting the simulation

### 2. Bridge Shows No Errors but No Data Flows

**Problem**: The bridge starts without errors, but `ros2 topic hz` shows no messages.

**Solution**:
- Make sure the simulation isn't paused (press the play button in Gazebo)
- Double-check the exact topic name spelling
- Verify that data is being published in Gazebo with `gz topic -e -t <topic_name>`
- Try restarting both Gazebo and the bridge

### 3. Can See Camera in Gazebo but Not in ROS 2

**Problem**: The camera feed is visible in Gazebo's Image Display but not in ROS 2 tools.

**Solution**:
- Try an alternative viewer: `ros2 run image_view image_view image:=/camera/image`
- Check if necessary packages are installed: `sudo apt install ros-humble-image-transport-plugins`
- Try republishing the image: 
  ```bash
  ros2 run image_transport republish raw in:=/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image raw out:=/simple_camera
  ros2 run rqt_image_view rqt_image_view /simple_camera
  ```

### 4. Graphics or Display Issues

**Problem**: You get errors related to graphics or display.

**Solution**:
- Try running the simulation with explicit display options: `HEADLESS=0 make px4_sitl gz_x500_depth`
- Check your system's graphics drivers
- Try saving the image instead of viewing it:
  ```bash
  ros2 run image_view extract_images --ros-args -r image:=/camera/image
  ```

### 5. NumPy Version Compatibility Issues

**Problem**: You get errors like the following when running the camera subscriber:
```
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.3 as it may crash.
...
AttributeError: _ARRAY_API not found
...
ImportError: numpy.core.multiarray failed to import
```

**Solution**:
This is a compatibility issue between cv_bridge (compiled with NumPy 1.x) and newer NumPy 2.x versions. Here are three reliable solutions:

#### Option 1: Use a Direct Subscriber Without cv_bridge

Create a new Python script that doesn't rely on cv_bridge:

```bash
# Create a directory for your script
mkdir -p ~/camera_scripts

# Create the direct subscriber script
nano ~/camera_scripts/direct_camera_subscriber.py
```

Add the following code:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class DirectCameraSubscriber(Node):
    def __init__(self):
        super().__init__('direct_camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',  # Use your ROS topic name here
            self.camera_callback,
            10)
        self.get_logger().info('Direct camera subscriber started')
        
    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image directly (no cv_bridge)
            dtype = np.uint8
            img = np.array(msg.data, dtype=dtype).reshape((msg.height, msg.width, 3))
            
            # Note: ROS Image msg might be in RGB order, OpenCV uses BGR
            # If colors look wrong, you might need: img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
            # Display the image
            cv2.imshow("Camera Feed", img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DirectCameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Make it executable and run:

```bash
chmod +x ~/camera_scripts/direct_camera_subscriber.py
python3 ~/camera_scripts/direct_camera_subscriber.py
```

#### Option 2: Use the Compressed Image Topic

If your bridge publishes compressed images (or you can add compression):

```bash
# First, add compression to your image topic
ros2 run image_transport republish raw in:=/camera/image compressed out:=/camera/image
```

Then create a subscriber for compressed images:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',  # The compressed topic
            self.camera_callback,
            10)
        self.get_logger().info('Compressed image subscriber started')
        
    def camera_callback(self, msg):
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Display the image
            cv2.imshow("Camera Feed", img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Option 3: System-wide Downgrade of NumPy (Use with Caution)

If you absolutely need to use cv_bridge and other ROS packages, you can downgrade NumPy system-wide:

```bash
# Check current NumPy version
python3 -c "import numpy; print(numpy.__version__)"

# Downgrade NumPy system-wide
sudo pip3 install numpy==1.24.3 --force-reinstall

# Make sure the correct version is installed
python3 -c "import numpy; print(numpy.__version__)"

# Rebuild your ROS package
cd ~/camera_ws
colcon build --packages-select px4_vision_demo
source install/setup.bash

# Try running again
ros2 run px4_vision_demo camera_subscriber
```

**Warning**: System-wide pip changes might affect other Python applications. Only use this if you understand the potential impacts.

## Using Camera Feeds in Your ROS 2 Application

Now that you have the camera feed available in ROS 2, let's create a complete ROS 2 package to process this feed. This example will display the camera feed in an OpenCV window and provide a foundation for adding your own vision processing algorithms.

### Creating a ROS 2 Package for Camera Processing

#### 1. Set up a ROS 2 Workspace (if you don't already have one)

```bash
# Create a ROS 2 workspace
mkdir -p ~/camera_ws/src
cd ~/camera_ws/src
```

#### 2. Create a new ROS 2 package

```bash
# Create a Python package for camera processing
ros2 pkg create --build-type ament_python --dependencies rclpy sensor_msgs cv_bridge --node-name camera_subscriber px4_vision_demo
```

#### 3. Navigate to the package and add the camera subscriber code

```bash
cd px4_vision_demo
```

Edit the file `px4_vision_demo/camera_subscriber.py` with your favorite editor:

```bash
nano px4_vision_demo/camera_subscriber.py
```

Replace the contents with:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        # Create subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
            self.camera_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Camera subscriber node started')
        
        # Create a resizable window
        cv2.namedWindow('Camera Feed Processing', cv2.WINDOW_NORMAL)
        
    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get original dimensions
            height, width, _ = cv_image.shape
            
            # Resize to a more manageable size if too large
            max_width = 1280
            if width > max_width:
                scale_factor = max_width / width
                new_width = int(width * scale_factor)
                new_height = int(height * scale_factor)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
                height, width, _ = cv_image.shape
            
            # Add text overlay with resolution
            text = f"Resolution: {width}x{height}"
            cv2.putText(cv_image, text, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Example processing: Edge detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            
            # Stack the original and processed images side by side
            processed_display = np.hstack((cv_image, edges_colored))
            
            # Display the result in a resizable window
            cv2.imshow("Camera Feed Processing", processed_display)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    
    # Clean up on shutdown
    cv2.destroyAllWindows()
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 4. Update package files

Edit the `setup.py` file to ensure the entry point is correctly defined:

```bash
nano setup.py
```

Make sure the `entry_points` section includes:

```python
entry_points={
    'console_scripts': [
        'camera_subscriber = px4_vision_demo.camera_subscriber:main',
    ],
},
```

#### 5. Install dependencies

Make sure you have all required dependencies:

```bash
# Install OpenCV and cv_bridge
sudo apt install python3-opencv ros-humble-cv-bridge
```

#### 6. Build the package

```bash
# Go to workspace root
cd ~/camera_ws

# Build the package
colcon build --packages-select px4_vision_demo

# Source the workspace
source install/setup.bash
```

#### 7. Run the node

With the bridge still running in another terminal, run your node:

```bash
ros2 run px4_vision_demo camera_subscriber
```

### What This Code Does

When you run this node while the camera bridge is active:

1. It subscribes to the camera feed topic `/camera/image` (make sure this matches your bridged topic)
2. For each image received:
   - Converts the ROS image message to an OpenCV image
   - Adds text overlay showing the image resolution
   - Performs edge detection using Canny algorithm
   - Displays the original image and edge detection results side by side in a window

![OpenCV Camera Processing](images/opencv_processing.png)
<!-- Add a screenshot of your OpenCV window showing the processed image here -->

### Extending the Code

This provides a foundation for implementing more complex vision algorithms:

- Change the `/camera/image` topic name if you used a different name in your bridge
- Add your own computer vision algorithms in the `camera_callback` function
- Publish processed results back to ROS 2 topics for other nodes to use
- Save processed images or detection results to disk

For example, to add feature detection with ORB:

```python
# Add to the camera_callback function
orb = cv2.ORB_create()
keypoints, descriptors = orb.detectAndCompute(gray, None)
orb_image = cv2.drawKeypoints(cv_image, keypoints, None, color=(0,255,0), flags=0)
cv2.imshow("ORB Features", orb_image)
```

### Using the Camera Feed for Visual Navigation

For a PX4 visual navigation system, you might want to:

1. Detect features in the camera image
2. Match these features to a pre-loaded map or satellite imagery
3. Estimate the drone's position based on these matches
4. Send position updates to PX4 through MAVROS

This foundation code gives you the starting point to implement these algorithms.

## Conclusion

You've now successfully bridged a camera feed from a PX4 simulation in Gazebo to ROS 2. This allows you to develop and test vision-based algorithms using simulated camera data before deploying to real hardware.

## References and Further Reading

- [ROS-GZ Bridge Documentation](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge)
- [PX4 Simulation Documentation](https://docs.px4.io/main/en/simulation/)
- [ROS 2 Image Transport](https://index.ros.org/p/image_transport/)
- [OpenCV with ROS 2](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

