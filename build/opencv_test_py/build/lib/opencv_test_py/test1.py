# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# This code is modified by Berk Calli from the following author.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

    self.subscription = self.create_subscription(
      Image, 
      '/camera1/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'output_image', 10)

      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)


    # PLACE YOUR CODE HERE. PROCESS THE CURRENT FRAME AND PUBLISH IT. IF YOU ARE HAVING DIFFICULTY PUBLISHING IT YOU CAN USE THE FOLLOWING LINES TO DISPLAY IT VIA OPENCV FUNCTIONS
    #cv2.imshow("output_image", current_frame)
    #cv2.waitKey(1)
    #Image thresholding Code
    
    hsv=cv2.cvtColor(current_frame,cv2.COLOR_BGR2HSV)
    
    #BLUE 
    lower_color_bounds = np.array([100, 0, 0])
    upper_color_bounds = np.array([225,10,10])
   
    mask = cv2.inRange(current_frame,lower_color_bounds,upper_color_bounds )
    mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    frame = current_frame & mask_rgb
   
    cv2.imshow("output",frame)
    self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))
    #center of blue
    M=cv2.moments(mask)
    center_x=int(M['m10']/M['m00'])
    center_y=int(M['m01']/M['m00'])
    cv2.circle(current_frame,(center_x,center_y),3,(0,0,255),-1)
    print("center",(center_x,center_y))
    cv2.imshow("p",current_frame)
    cv2.waitKey(1)
    
    
    #green
    lower_color_bounds = np.array([0, 100, 0])
    upper_color_bounds = np.array([40,225,40])
   
    mask = cv2.inRange(current_frame,lower_color_bounds,upper_color_bounds )
    mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    current_frame= current_frame & mask_rgb
   
    cv2.imshow("output",frame)
    self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))

    #center of green 
    M=cv2.moments(mask)
    center_x=int(M['m10']/M['m00'])
    center_y=int(M['m01']/M['m00'])
    cv2.circle(current_frame,(center_x,center_y),3,(0,0,255),-1)
    print("center",(center_x,center_y))
    cv2.imshow("p",current_frame)
    cv2.waitKey(1)
    
    
    
    #red
    lower_color_bounds = np.array([0, 0, 100])
    upper_color_bounds = np.array([10,10,255])
   
    mask = cv2.inRange(current_frame,lower_color_bounds,upper_color_bounds )
    mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    current_frame = current_frame & mask_rgb
   
    cv2.imshow("output",current_frame)
    self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))

    #center of red
    M=cv2.moments(mask)
    center_x=int(M['m10']/M['m00'])
    center_y=int(M['m01']/M['m00'])
    cv2.circle(current_frame,(center_x,center_y),3,(0,0,255),-1)
    print("center",(center_x,center_y))
    cv2.imshow("p",current_frame)
    cv2.waitKey(1)
    
    
    #purple 
    lower_color_bounds = np.array([120, 0, 120])
    upper_color_bounds = np.array([245,18,245])
   
    mask = cv2.inRange(current_frame,lower_color_bounds,upper_color_bounds )
    mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    current_frame = current_frame & mask_rgb
   
    cv2.imshow("output",current_frame)
    self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))

    #center of purple
    M=cv2.moments(mask)
    center_x=int(M['m10']/M['m00'])
    center_y=int(M['m01']/M['m00'])
    cv2.circle(current_frame,(center_x,center_y),3,(0,0,255),-1)
    print("center",(center_x,center_y))
    cv2.imshow("p",current_frame)
    cv2.waitKey(1)
    
    '''
    '''
    #Setting parameter values
    
    '''
    t_lower = 50  # Lower Threshold
    t_upper = 150  # Upper threshold
  
    #Applying the Canny Edge filter
    edge = cv2.Canny(current_frame, t_lower, t_upper)
    cv2.imshow("output_image",edge)
    cv2.waitKey(1)
    '''

    
    '''
    
    operatedImage=cv2.cvtColor(current_frame,cv2.COLOR_BGR2GRAY)
    operatedImage = np.float32(operatedImage)
    dest = cv2.cornerHarris(operatedImage, 2, 5, 0.07)
    dest = cv2.dilate(dest, None)
    current_frame[dest > 0.01 * dest.max()]=[0, 0, 255]
   
    #cv2.imshow("border",current_frame)
    self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))


'''

'''
    
    img = cv2.cvtColor(current_frame,cv2.COLOR_BGR2GRAY)
    img = cv2.medianBlur(img,5)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=0,maxRadius=0)
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
      cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
      # draw the center of the circle
      cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
    cv2.imshow("image",cimg)
    cv2.waitKey(1)
'''
    # Publish the image.
    # The 'cv2_to_imgmsg' method converts an OpenCV
    # image to a ROS 2 image message
  #self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))

   
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
