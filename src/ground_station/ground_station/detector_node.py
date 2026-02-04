#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time

class DetectorNode(Node):
    def __init__(self):
        super().__init__('ground_detector')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            'raw_image',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        
        # YOLO model yükle
        self.get_logger().info('🔄 YOLO model yükleniyor...')
        self.model = YOLO('yolov8n.pt')  # nano model (hızlı)
        self.get_logger().info('✅ Ground Station hazır - YOLO aktif')
        
        # Metrikler
        self.frame_count = 0
        self.start_time = time.time()
        
    def image_callback(self, msg):
        # ROS Image -> OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # YOLO inference
        results = self.model(cv_image, verbose=False)
        
        # Sonuçları çiz
        annotated_frame = results[0].plot()
        
        # Detection bilgileri
        detections = results[0].boxes
        if len(detections) > 0:
            for box in detections:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.model.names[cls]
                self.get_logger().info(f'🎯 Tespit: {label} ({conf:.2f})')
        
        # Görüntüyü göster
        cv2.imshow('EagleEye - Ground Station', annotated_frame)
        cv2.waitKey(1)
        
        # FPS hesapla
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            elapsed = time.time() - self.start_time
            fps = 30 / elapsed
            self.get_logger().info(f'📊 Detection FPS: {fps:.1f}')
            self.start_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
