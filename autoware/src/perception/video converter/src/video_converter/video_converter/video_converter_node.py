#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv_bridge
import cv2
import numpy as np

class ImageToVideoConverter(Node):
    def __init__(self):
        super().__init__('image_to_video_converter')
        self.bridge = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/compressed',  
            self.image_callback,
            10)
        self.video_writer = None

    def init_video_writer(self, frame):
        height, width = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = 30  
        self.video_writer = cv2.VideoWriter('output_video.mp4', fourcc, fps, (width, height))
        self.get_logger().info(f'VideoWriter inizializzato: output_video.mp4 {width}x{height} @ {fps}fps')

    def image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                raise RuntimeError('decode fallita')
            
            if self.video_writer is None:
                self.init_video_writer(frame)

            self.video_writer.write(frame)
        
        except Exception as e:
            self.get_logger().error(f'Errore durante elaborazione immagine: {e}')

    def destroy_node(self):
        if self.video_writer:
            self.video_writer.release()
            self.get_logger().info('VideoWriter rilasciato.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageToVideoConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
