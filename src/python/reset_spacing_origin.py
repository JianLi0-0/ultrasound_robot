import sys, os, time, rospy
import SimpleITK as sitk

def resampleVolume(outspacing, vol):
    
    """

    将体数据重采样的指定的spacing大小\n

    paras：

    outpacing：指定的spacing，例如[1,1,1]

    vol：sitk读取的image信息，这里是体数据\n

    return：重采样后的数据

    """

    outsize = [0, 0, 0]

    # 读取文件的size和spacing信息

    inputsize = vol.GetSize()

    inputspacing = vol.GetSpacing()

    transform = sitk.Transform()

    transform.SetIdentity()

    # 计算改变spacing后的size，用物理尺寸/体素的大小

    outsize[0] = round(inputsize[0] * inputspacing[0] / outspacing[0])

    outsize[1] = round(inputsize[1] * inputspacing[1] / outspacing[1])

    outsize[2] = round(inputsize[2] * inputspacing[2] / outspacing[2])

    # 设定重采样的一些参数

    resampler = sitk.ResampleImageFilter()

    resampler.SetTransform(transform)

    resampler.SetInterpolator(sitk.sitkLinear)

    resampler.SetOutputOrigin(vol.GetOrigin())

    resampler.SetOutputSpacing(outspacing)

    resampler.SetOutputDirection(vol.GetDirection())

    resampler.SetSize(outsize)

    newvol = resampler.Execute(vol)

    return newvol


rospy.init_node('xxx', anonymous=True)
os.system('rosparam load $(rospack find ultrasound_robot)/config/registration.yaml svr')
print("config file loaded: rosparam load $(rospack find ultrasound_robot)/config/registration.yaml svr")

file_name = "biopsy_single.mha"

spacing = rospy.get_param("/svr/spacing")
volume = sitk.ReadImage(file_name)

volume.SetOrigin([0, 0, 0])
volume.SetSpacing([spacing, spacing, spacing])

# flipped_volume = sitk.Flip(volume, [False, False, False])
# print(volume.GetDirection())
# flipped_volume.SetDirection((1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0))
# flipped_volume.SetOrigin([0, 70, 0])

flipped_volume = sitk.Flip(volume, [False, False, True])
flipped_volume.SetDirection(volume.GetDirection())
flipped_volume.SetOrigin([0, 0, 0])


flipped_volume.SetSpacing([spacing*1.2, spacing, spacing])
print(flipped_volume)
flipped_volume = resampleVolume([spacing, spacing, spacing], flipped_volume)
print(flipped_volume)

writer = sitk.ImageFileWriter()
writer.SetFileName("new_"+file_name)
writer.Execute(flipped_volume)
print("Saved file name: " + "new_"+file_name)
