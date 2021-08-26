import sys, os, time, rospy
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROS_IMAGE
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np
from PIL import Image
from PyV4L2Camera.camera import Camera
import UltrasoundProcess
import transformations as tf
from scipy.spatial.transform import Rotation as R

if len(sys.argv) == 1:
    mode = "servo"
elif sys.argv[1]=='1':
    mode = "display" # used to identify the height and width of the ultrasound image
    def setMouseCallback(event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            print(x,y)
    cv2.namedWindow('ultrasound')
    cv2.setMouseCallback('ultrasound',setMouseCallback) 
elif sys.argv[1]=='2':
    mode = "reconstruction"
print("mode: " + mode)

############### ROS ###############
command = "0" # 0 for waiting; 1 for sampling; 2 for exit
def sample_command_callback(msg):
    # print(msg)
    global command
    command = msg.data

ee_link_tf = []
def ee_link_tf_callback(msg):
    global ee_link_tf
    ee_link_tf = list(msg.data)
    # print(ee_link_tf)

rospy.init_node('sample_ultrasound', anonymous=True)
rospy.Subscriber("/sample_command", String, sample_command_callback)
rospy.Subscriber("/ee_link_tf", Float64MultiArray, ee_link_tf_callback)
us_img_publisher = rospy.Publisher("/us_image", ROS_IMAGE, queue_size=10)
bridge = CvBridge()

os.system('rosparam load $(rospack find ultrasound_robot)/config/registration.yaml svr')
print("config file loaded: rosparam load $(rospack find ultrasound_robot)/config/registration.yaml svr")
############### ROS ###############

def crop_image(image_arr, starting_point, size):
    x = starting_point
    y = x[0]+size[0], x[1]+size[1]
    return image_arr[x[0]:y[0],x[1]:y[1]]

np_img_list = []
transformation_list = []
starting_point = [rospy.get_param("/svr/y"), rospy.get_param("/svr/x")] #(height, weight)
servo_starting_point = [rospy.get_param("/svr/servo_y"), rospy.get_param("/svr/servo_x")] #(height, weight)
servo_size = [rospy.get_param("/svr/servo_h"), rospy.get_param("/svr/servo_w")] # 1484-560q
reduction = rospy.get_param("/svr/volume_resol_reduction")
size = [rospy.get_param("/svr/reconstruction_h"), rospy.get_param("/svr/reconstruction_w")] # 1484-560q

# cv2.setMouseCallback("MyImage", onmouse)   #回调绑定窗口
# (camera.width, camera.height): [1920,1080]

camera = Camera('/dev/UltrasoundImage')
if(camera.width!=1920): 
    print("camera.width is not 1280")
    exit(0)
print("camera.width:"+str(camera.width)+"   camera.height:"+str(camera.height))
cropped = 0
count = 0
while rospy.is_shutdown() is False:
# while(1):
    try:
        frame = camera.get_frame()
        image=Image.frombytes('RGB', (camera.width, camera.height), frame, 'raw', 'RGB') 
        # image = Image.open("geeks.jpg")
    except:
        print("camera.get_frame() failed")
    # print("FPS: ", 1.0 / (time.time() - start_time))

    gray = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2GRAY)
    

    if mode == 'servo':
        enlarge_size = [servo_size[1]*reduction, servo_size[0]*reduction]
        cropped = crop_image(gray, servo_starting_point, enlarge_size)
        resized = cv2.resize(cropped, tuple(servo_size), interpolation = cv2.INTER_AREA)
        try:
            ros_img = bridge.cv2_to_imgmsg(resized, encoding="passthrough")
            us_img_publisher.publish(ros_img)
        except CvBridgeError as e:
            print(e)
        
        cv2.rectangle(gray, (servo_starting_point[1], servo_starting_point[0]), (servo_starting_point[1]+servo_size[1]*reduction, servo_starting_point[0]+servo_size[0]*reduction), 255, 2)
        cropped2 = crop_image(gray, starting_point, size)
        # cv2.imshow('servo', np.asarray(cropped2))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

    elif mode == 'display':
        cv2.imshow('ultrasound', np.asarray(image))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

    elif mode == 'reconstruction':
        cropped = crop_image(gray, starting_point, size)
        if command == '2':
            # volume = sitk.ReadImage("./dataset/thyroid.mhd")
            # trash, transformation_list = UltrasoundProcess.GenerateTestData(volume, count)
            if len(np_img_list) == len(transformation_list):
                UltrasoundProcess.SaveImage(np_img_list, transformation_list, "thyroid_sample_raw.mha")
            else:
                print("Error: len(np_img_list) != len(transformation_list)!")
            break
        elif command == '1':
            command = '0'
            count = count + 1
            print("sample: "+str(count))
            # start_time = time.time()
            
            if ee_link_tf:
                np_img_list.append(cropped)
                
                r = R.from_quat(ee_link_tf[3:])
                matrix = tf.transformations.compose_matrix(translate=ee_link_tf[0:3], angles=list(r.as_euler('xyz')))
                transformation_list.append(list(matrix.flatten()))
                # print(ee_link_tf)
                # print(list(r.as_euler('xyz')))
                # print(matrix)
                ee_link_tf = []
            else:
                print("Error: ee_link_tf is empty!")
        cv2.imshow('gray', cropped)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

cv2.destroyAllWindows()
exit()
rospy.spin()
