import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
from arm_zed_camera_py_pkg.orange_detector import detect_orange_objects
from arm_zed_camera_py_pkg.ArucoDetector import aruco_detection
from rclpy.logging import get_logger



class ZEDCameraNode(Node):
    def __init__(self):
        super().__init__('zed_camera_node')

        self.model = YOLO("yolov8n.pt")

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Updated per ZED warning for better depth
        init_params.coordinate_units = sl.UNIT.METER



        init_params.sdk_verbose = 0  


        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Failed to open ZED")
            exit(1)

        self.runtime_parameters = sl.RuntimeParameters()
        self.image_left = sl.Mat()
        self.point_cloud = sl.Mat()
        self.depth_map = sl.Mat()

        self.bbox_pub = self.create_publisher(String, 'person_bboxes', 10)
        self.distance_pub = self.create_publisher(Point, 'person_distance', 10)

        self.bridge = CvBridge()

        # Target ~60 FPS (16 ms)
        self.timer = self.create_timer(0.016, self.timer_callback)

    def timer_callback(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)

            left_img = self.image_left.get_data()
            left_rgb = cv2.cvtColor(left_img, cv2.COLOR_RGBA2RGB)

            # Resize image smaller for faster YOLO inference
            small_img = cv2.resize(left_rgb, (640, 384))
            results = self.model(small_img)[0]  # TODO: add the vervose=False if you don't want to see prints in terminal     

            pc_data = self.point_cloud.get_data()


            for box in results.boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                if label != "person":
                    continue

                # Scale box coords back to original size
                x1, y1, x2, y2 = box.xyxy[0]
                scale_x = left_rgb.shape[1] / 640
                scale_y = left_rgb.shape[0] / 384

                x1 = int(x1 * scale_x)
                y1 = int(y1 * scale_y)
                x2 = int(x2 * scale_x)
                y2 = int(y2 * scale_y)

                conf = float(box.conf[0])
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                point3d = pc_data[cy, cx]
                x, y, z = point3d[:3]
                distance = float(z) if z > 0 else -1.0

                bbox_msg = String()
                bbox_msg.data = f"Person ({conf:.2f}) - BBox: [{x1},{y1},{x2},{y2}]"
                self.bbox_pub.publish(bbox_msg)

                dist_msg = Point()
                dist_msg.x = distance
                dist_msg.y = float(cx)
                dist_msg.z = float(cy)
                self.distance_pub.publish(dist_msg)

                cv2.rectangle(left_rgb, (x1, y1), (x2, y2), (0, 200, 255), 2)
                cv2.putText(left_rgb, f"Person {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                dist_text = f"Distance: {distance:.2f} m" if distance > 0 else "Distance: N/A"
                cv2.putText(left_rgb, dist_text, (x1, y2 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Depth map visualization: normalize per-frame depth for better contrast
            depth_data = self.depth_map.get_data()




            # Mask invalid depths (<=0)
            valid_mask = depth_data > 0
            if np.any(valid_mask):
                min_depth = np.min(depth_data[valid_mask])
                max_depth = np.max(depth_data[valid_mask])
                norm_depth = np.zeros_like(depth_data, dtype=np.float32)


                if max_depth > min_depth:
                    norm_depth[valid_mask] = (depth_data[valid_mask] - min_depth) / (max_depth - min_depth +1 )
                else:
                    # All depths are equal, set normalized depth to zeros or ones (your choice)
                    norm_depth[valid_mask] = 0.0

                depth_8u = (norm_depth * 255).astype(np.uint8)
            else:
                depth_8u = np.zeros_like(depth_data, dtype=np.uint8)


            

            depth_color = cv2.applyColorMap(depth_8u, cv2.COLORMAP_TURBO)

            # Detect orange objects
            # left_rgb, orange_boxes = detect_orange_objects(left_rgb)
            image, boxes, mask = detect_orange_objects(left_rgb)
            cv2.imshow("Orange Mask", mask)
            cv2.waitKey(1)

            self.get_logger().info("[orange_detector] Detected orange object")

            left_rgb = aruco_detection(left_rgb)

            cv2.imshow("ZED Left + YOLO + Distance", left_rgb)
            cv2.imshow("ZED Depth", depth_color)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.zed.close()
                cv2.destroyAllWindows()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ZEDCameraNode()
    rclpy.spin(node)
    node.zed.close()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
