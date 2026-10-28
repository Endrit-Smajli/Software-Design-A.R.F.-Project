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

    # Signals the publisher thread to stop.
    ros_stop_event = ros_holder.setdefault(
        "stop_event",
        threading.Event()
    )
    
    # Create a background thread to run ROS2 publisher so that it doesn't block the Flask server.
    ros_thread = threading.Thread(
        target = start_ros_publisher,
        args=(ros_holder, ),
        daemon = False)

    server = None

    try:
        ros_thread.start()

        server = FlaskServer(
            device,
            yolo_drawer,
            tracker,
            ros_holder
        )
        
        # Run Flask in the main thread.
        server.start()

    except KeyboardInterrupt:
        print("\nShutdown requested. Stopping A.R.F. dashboard...\n")

    finally:
        
        # 1. Stop the Flask processing loop and wake stream clients.
        if server is not None:
            server.stop()

        # 2. Stop the ROS 2 publisher and wait for its cleanup.
        ros_stop_event.set()

        # Wait for the publisher thread to clean up.
        if ros_thread.is_alive():
            ros_thread.join(timeout = 3.0)

        if ros_thread.is_alive():
            print("Warning: ROS 2 publisher thread has not exited yet.")

        # 3. Stop the OAK-D pipeline after frame processing has stopped.
        try:
            device.pipeline.stop()
            print("OAK-D pipeline stopped.")
            
        except Exception as exc:
            print(f"Warning: Could not stop OAK-D pipeline cleanly: {exc}")

        print("Dashboard shutdown sequence finished.")
