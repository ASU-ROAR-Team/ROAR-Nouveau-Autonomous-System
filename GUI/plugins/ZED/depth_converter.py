#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage # Import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# --- Define your fixed depth range here (in meters) ---
# Adjust these values based on the typical depth range of your ZED camera and scene.
DEPTH_MIN = 0.5  # Minimum depth to consider (e.5 meters)
DEPTH_MAX = 10.0 # Maximum depth to consider (e.g., 10.0 meters)
# -----------------------------------------------------------

class DepthImageConverter:
    def __init__(self):
        rospy.init_node('depth_image_converter', anonymous=True)

        self.bridge = CvBridge()
        
        # Subscribe to the raw 32FC1 depth image topic
        self.image_sub = rospy.Subscriber(
            "/zed2i/zed_node/depth/depth_registered",
            Image,
            self.callback,
            queue_size=1
        )
        
        # --- NEW: Publisher for the COMPRESSED color-mapped image ---
        # We'll publish to a new topic name to clearly distinguish it
        self.compressed_image_pub = rospy.Publisher(
            "/zed2i/zed_node/depth/depth_registered/color_mapped_image/compressed_for_web",
            CompressedImage, # Publish CompressedImage type
            queue_size=1
        )

        rospy.loginfo("Depth Image Converter Node Started.")
        rospy.loginfo(f"Subscribing to /zed2i/zed_node/depth/depth_registered (32FC1)")
        rospy.loginfo(f"Publishing to /zed2i/zed_node/depth/depth_registered/color_mapped_image/compressed_for_web (JPEG)")


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        if cv_image is None:
            rospy.logwarn("Received empty or invalid image.")
            return

        # Handle NaN/Inf values and clip to fixed depth range
        processed_image = np.copy(cv_image)
        processed_image[np.isnan(processed_image)] = 0.0
        processed_image[np.isinf(processed_image)] = 0.0
        processed_image = np.clip(processed_image, DEPTH_MIN, DEPTH_MAX)

        # Normalize to 0-255 using the FIXED global min/max
        if DEPTH_MAX == DEPTH_MIN:
            normalized_image = np.zeros_like(processed_image, dtype=np.uint8)
        else:
            normalized_image = ((processed_image - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN) * 255).astype(np.uint8)
        
        # Apply a colormap (e.g., COLORMAP_JET)
        color_mapped_image = cv2.applyColorMap(normalized_image, cv2.COLORMAP_JET)

        # --- NEW: Compress the image to JPEG ---
        # cv2.imencode returns (success, buffer)
        # We want to encode it as JPEG, with a quality of 80 (0-100)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        try:
            result, encoded_image = cv2.imencode('.jpg', color_mapped_image, encode_param)
            if not result:
                rospy.logerr("Failed to encode image to JPEG.")
                return
            
            # Create a CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = data.header # Use original header for timestamp/frame_id
            compressed_msg.format = "jpeg" # Specify format
            compressed_msg.data = np.array(encoded_image).tobytes() # Convert numpy array to bytes

            # Publish the compressed image
            self.compressed_image_pub.publish(compressed_msg)
        except Exception as e:
            rospy.logerr(f"Image compression/publishing Error: {e}")


if __name__ == '__main__':
    try:
        converter = DepthImageConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass