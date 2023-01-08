import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Float32
import math
import numpy as np
l1=1
l2=1
l3=1

def param(theta,d,a,alpha):
        c=math.cos(theta)
        s=math.sin(theta)
        t=math.cos(alpha)
        y=math.sin(alpha)
        an=[[c,-s*t,s*y,a*c], [s,c*t,-c*y,a*s],[0,y,t,d],[0, 0, 0,1]]
        return an

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'topic1',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    

    def listener_callback(self,joints):
        q1=joints.data[0]
        q2=joints.data[1]
        q3=joints.data[2]
        a1=param(q1,l1,0,-90)
        a2=param(q2,0,l2,0)
        a3=param(q3,0,l3,0)
        #Calculating trans=formation matrix
        fk=np.dot(a1,a2)
        ans=np.dot(fk,a3)
        print("Pose of End Effector")
        print(ans)

        '''
        cy=math.cos(euler.data[0] * 0.5)
        sy=math.sin(euler.data[0]* 0.5)
        cp=math.cos(euler.data[1] * 0.5)
        sp=math.sin(euler.data[1] * 0.5)
        cr=math.cos(euler.data[2]* 0.5)
        sr=math.sin(euler.data[2]*0.5)
        quater=Float32MultiArray()
        quater.data= [cr*cp*cy + sr*sp*sy, sr * cp * cy - cr * sp * sy,cr * sp * cy + sr * cp * sy,cr * cp * sy - sr * sp * cy]
        self.get_logger().info('I heard: "%f %f %f %f"' % (quater.data[0],quater.data[1],quater.data[2],quater.data[3] ))
        '''
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
