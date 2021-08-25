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
flipped_volume = sitk.Flip(volume, [False, False, True])
flipped_volume.SetDirection(volume.GetDirection())
flipped_volume.SetOrigin([0, 0, 0])
# print(flipped_volume)

writer = sitk.ImageFileWriter()
writer.SetFileName("new_"+file_name)
writer.Execute(flipped_volume)
print("Saved file name: " + "new_"+file_name)
