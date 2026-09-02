from oakd_device import OakDDevice
from processors import YoloDrawer, SimpleSortTracker
from outputs import FlaskServer, start_ros_publisher
import threading

if __name__ == "__main__":
    device = OakDDevice()
    yolo_drawer = YoloDrawer()
    tracker = SimpleSortTracker()

    ros_holder = {}
    ros_thread = threading.Thread(target=start_ros_publisher,
                                  args=(ros_holder,),
                                  daemon=True)
    ros_thread.start()

    server = FlaskServer(device, yolo_drawer, tracker, ros_holder)
    server.start()