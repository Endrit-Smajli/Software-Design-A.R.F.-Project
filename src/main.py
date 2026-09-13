from oakd_device import OakDDevice
from processors import YoloDrawer, SimpleSortTracker
from outputs import FlaskServer, start_ros_publisher
import threading    # Used to run ROS2 in a background thread while the Flask server runs in the main thread.

if __name__ == "__main__":
    device = OakDDevice()           # Initialize the OAK-D device.
    yolo_drawer = YoloDrawer()      # Initialize processors.
    tracker = SimpleSortTracker()   # Initialize a simple tracking system.

    # Prepare ROS2 publisher.
    ros_holder = {}
    
    # Create a background thread to run ROS2 publisher so that it doesn't block the Flask server.
    ros_thread = threading.Thread(target=start_ros_publisher,
                                  args=(ros_holder,),
                                  daemon=True)
    
    # Start the ROS2 publisher thread.
    ros_thread.start()

    # Start the Flask server to serve the video stream and tracking data.
    server = FlaskServer(device, yolo_drawer, tracker, ros_holder)
    server.start()
