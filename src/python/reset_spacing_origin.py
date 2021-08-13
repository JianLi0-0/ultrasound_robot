import sys, os, time, rospy
import SimpleITK as sitk


rospy.init_node('xxx', anonymous=True)
os.system('rosparam load $(rospack find ultrasound_robot)/config/registration.yaml svr')
print("config file loaded: rosparam load $(rospack find ultrasound_robot)/config/registration.yaml svr")

file_name = "biopsy_single.mha"

spacing = rospy.get_param("/svr/spacing")
volume = sitk.ReadImage(file_name)

volume.SetOrigin([0, 0, 0])
volume.SetSpacing([spacing, spacing, spacing])

# print(volume)

writer = sitk.ImageFileWriter()
writer.SetFileName(file_name)
writer.Execute(volume)
print("Saved file name: " + file_name)
