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
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import math
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Float64
# Enables the use of rclpy's Node class
from rclpy.node import Node 
 
# Base class to handle exceptions
from tf2_ros import TransformException 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray 
# Math library
import math
from tf_transformations import quaternion_matrix
 

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    self.red_x=[]
    self.red_y=[]
    self.green_x=[]
    self.green_y=[]
    self.blue_x=[]
    self.blue_y=[]
    self.current=[]

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
      1)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'output_image', 1)
    self.velocity_publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 1)

    #joint states values
    self.joint_state_sub=self.create_subscription(
      JointState, 
      '/joint_states', 
      self.joint_para, 
      1)
      
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
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
    mask_b = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    frame1= current_frame & mask_b
   
    #cv2.imshow("output",frame)
    #self.publisher_.publish(self.br.cv2_to_imgmsg(frame1, encoding="bgr8"))
    #center of blue
    Mb=cv2.moments(mask)
    center_bx=int(Mb['m10']/Mb['m00'])
    center_by=int(Mb['m01']/Mb['m00'])
    cv2.circle(frame1,(center_bx,center_by),3,(0,0,255),-1)
    #print("center",(center_x,center_y))
    self.blue_x.append(center_bx)
    self.blue_y.append(center_by)
    #array.append(center_y)
    #cv2.imshow("p",frame1)
    #cv2.waitKey(1)
    
    
    #green
    lower_color_bounds = np.array([0, 100, 0])
    upper_color_bounds = np.array([40,225,40])
   
    mask = cv2.inRange(current_frame,lower_color_bounds,upper_color_bounds )
    mask_g = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    frame2= current_frame & mask_g
   
    #cv2.imshow("output",frame)
    #self.publisher_.publish(self.br.cv2_to_imgmsg(frame2, encoding="bgr8"))

    #center of green 
    Mg=cv2.moments(mask)
    center_gx=int(Mg['m10']/Mg['m00'])
    center_gy=int(Mg['m01']/Mg['m00'])
    cv2.circle(frame2,(center_gx,center_gy),3,(0,0,255),-1)
    #print("center",(center_x,center_y))
    self.green_x.append(center_gx)
    self.green_y.append(center_gy)
    #cv2.imshow("p",frame2)
    #cv2.waitKey(1)
    
    
    #red
    lower_color_bounds = np.array([0, 0, 100])
    upper_color_bounds = np.array([10,10,255])
   
    mask = cv2.inRange(current_frame,lower_color_bounds,upper_color_bounds )
    mask_r = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    frame3 = current_frame & mask_r
   
    #cv2.imshow("output",current_frame)
    #self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))

    #center of red
    Mr=cv2.moments(mask)
    center_rx=int(Mr['m10']/Mr['m00'])
    center_ry=int(Mr['m01']/Mr['m00'])
    cv2.circle(frame3,(center_rx,center_ry),3,(0,0,255),-1)
    #print("center",(center_x,center_y))
    self.red_x.append(center_rx)
    self.red_y.append(center_ry)
    #cv2.imshow("p",frame3)
    #cv2.waitKey(1)
    
    #this is the s_current feature positions  
    #print(array)
    s_current = ([center_bx,center_by,center_gx,center_gy,center_rx,center_ry])
    self.current.append(s_current)
    print("s_current values")
    print(s_current)
    #print("all valuess")
    #print(self.current)

    #s_current=([53, 403, 165, 366, 127, 441])
    #DEFINING THE S* REFERNCE IMAGE CO-ORDINATES
    s_star=np.array([288, 261, 401, 294, 328, 334])
    error = s_star -s_current
    print("EROR", error)
    #implementing servo control loop
    
    #initialising lamda value
    la=0.003
    lam=np.array([[-la,0],[0,-la]])
    focus=1
    depth=1
    g=(-focus/depth)

    #initialise image jacobian
    le=np.array([[g,0], [0,g], [g,0], [0,g], [g,0], [0,g]])
    le_inv=np.linalg.pinv(le)
    a=np.matmul(lam,le_inv)
    v_c=np.matmul(a,error)
    print("ERRRRRRROOOOOORRRRR", error)
    
    #have to change this from camera frame to robot base frame using transformation rotation matrix

    aa=((-0.5*math.sin(self.theta1))-(0.5*math.sin(self.theta1+self.theta2)))
    bb=((0.5*math.cos(self.theta1))+(0.5*math.cos(self.theta1+self.theta2)))
    cc=((-0.5*math.sin(self.theta1+self.theta2)))
    dd=((0.5*math.cos(self.theta1+self.theta2)))

    jaco=[[aa,cc], [bb,dd]]
    q_dot=np.matmul(jaco,v_c)
    #print("jacobian values",jaco)

    #publish onto the screen 
    q1=-q_dot[0]
    q2=-q_dot[1]
    print("printinnnnng q dot valueeee")
    
    #print(q_dot)
    val=Float64MultiArray()

    val.data=[q1,q2]
    #Switch controller code 
    #self.publisher_ = self.create_publisher(Image, 'output_image', 1)
    #self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))
    #self.velocity_publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 1)
    self.velocity_publisher.publish(val)
    #print("blueeeeeee_xxxxx")
    #print(self.blue_x)
    #print("blueeeeeee_yyyy")
    #print(self.blue_y)
    #print("greeeeen_xxxxx")
    #print(self.green_x)
    #print("green_yyyyy")
    #print(self.green_y)
    #print("red_xxxxx")
    #print(self.red_x)
    #print("redddddd_yy")
    #print(self.red_y)
    

  

  def joint_para(self,data):
        self.theta1=data.position[0]

        #print("printing joint values")
        #print(self.theta1)
        self.theta2=data.position[1]
        #print(self.theta2)

    # Publish the image.
    # The 'cv2_to_imgmsg' method converts an OpenCV
    # image to a ROS 2 image message
    #self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding="bgr8"))

 
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  print(image_subscriber.blue_x)
  
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
