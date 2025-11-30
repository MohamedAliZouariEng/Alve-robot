#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from custom_msgs.msg import InferenceResult, Yolov12Inference
import os
from pathlib import Path

bridge = CvBridge()

class YoloObjectDetection(Node):
    def __init__(self) -> None:
        super().__init__('object_detection')

        # Load a pre-trained YOLOv12 object detection model
        #self.model = YOLO('/home/ubuntu24/ros2_ws/src/advanced_perception/data/yolo12n.pt') 
        self.model = YOLO(Path.home() / 'ros2_ws' / 'src' / 'advanced_perception' / 'data' / 'yolo12n.pt')
        self.yolov12_inference = Yolov12Inference()
        
        # Set of target classes to detect
        self.target_classes = {"orange", "donut", "pizza"}
        
        # Flag to track if we should continue running
        self.should_run = True
        self.detected_object = None

        self.subscription = self.create_subscription(
            Image,
            '/camera_two/image',
            self.camera_callback,
            10)

        self.yolov12_pub = self.create_publisher(Yolov12Inference, "/Yolov12_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        
        # New publisher for detected object names
        self.object_name_pub = self.create_publisher(String, "/object_detected_name", 1)
        
        # Subscriber to listen for object detection confirmation
        self.object_name_sub = self.create_subscription(
            String,
            '/object_detected_name',
            self.object_detected_callback,
            10
        )

    def object_detected_callback(self, msg: String) -> None:
        """Callback when object is detected - saves data and triggers shutdown"""
        self.detected_object = msg.data
        self.get_logger().info(f"Received object detection: {self.detected_object}")
        
        # Save to text file
        self.save_to_file()
        
        # Set flag to stop running
        self.should_run = False
        
        # Schedule node shutdown
        self.get_logger().info("Shutting down node...")
        self.create_timer(0.1, self.initiate_shutdown)  # Small delay to ensure file is saved

    def save_to_file(self) -> None:
        """Save the detected object to data.txt file"""
        #file_path = '/home/ubuntu24/ros2_ws/src/advanced_perception/data/data.txt'  # Your desired path
        file_path = Path.home() / 'ros2_ws' / 'src' / 'advanced_perception' / 'data' / 'data.txt'
        try:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            
            with open(file_path, 'w') as f:
                f.write(self.detected_object)
            self.get_logger().info(f"Saved '{self.detected_object}' to {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save to file: {e}")

    def initiate_shutdown(self) -> None:
        """Initiate the node shutdown process"""
        self.destroy_node()
        rclpy.shutdown()

    def camera_callback(self, msg: Image) -> None:
        """Performs object detection using the loaded YOLO model
           and processes the detection results. The image with 
           annotated detections is also published for visualization"""
        
        # Check if we should continue processing
        if not self.should_run:
            return

        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img) 

        self.yolov12_inference.header.frame_id = "inference"
        self.yolov12_inference.header.stamp = self.get_clock().now().to_msg()
        
        detected_objects = set()  # To avoid duplicate publications

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inf_result = InferenceResult()
                # get box coordinates in (top, left, bottom, right) format
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  
                c = box.cls
                class_name = self.model.names[int(c)]
                self.inf_result.class_name = class_name
                self.inf_result.left = int(b[0])
                self.inf_result.top = int(b[1])
                self.inf_result.right = int(b[2])
                self.inf_result.bottom = int(b[3])
                self.inf_result.box_width = (self.inf_result.right - self.inf_result.left) 
                self.inf_result.box_height = (self.inf_result.bottom - self.inf_result.top)
                self.inf_result.x = self.inf_result.left + (self.inf_result.box_width/2.0)
                self.inf_result.y = self.inf_result.top + (self.inf_result.box_height/2.0)
                self.yolov12_inference.yolov12_inference.append(self.inf_result)
                
                # Check if this is one of our target classes
                if class_name in self.target_classes and class_name not in detected_objects:
                    detected_objects.add(class_name)
                    # Publish the detected object name
                    object_msg = String()
                    object_msg.data = class_name
                    self.object_name_pub.publish(object_msg)
                    self.get_logger().info(f"Detected target object: {class_name}")

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)

        self.yolov12_pub.publish(self.yolov12_inference)
        self.yolov12_inference.yolov12_inference.clear()

def main(args=None) -> None:
    rclpy.init(args=args)
    object_detection = YoloObjectDetection()
    
    try:
        # Spin until the node is told to stop
        while rclpy.ok() and object_detection.should_run:
            rclpy.spin_once(object_detection, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        object_detection.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()