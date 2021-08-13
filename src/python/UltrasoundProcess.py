import numpy as np
import SimpleITK as sitk
import time
# import ExtractSliceFromVolume
import transformations as tf
import math
import random

def SaveImage(np_img_list, transformation_list, file_name):
    img_array = np.stack(np_img_list)
    print("Image shape: " + str(img_array.shape))
    sitk_img = sitk.GetImageFromArray(img_array)
    # sitk_img.SetSpacing([0.18815431515639, 0.18815431515639, 0.18815431515639])
    sitk_img.SetSpacing([1, 1, 1])
    sitk_img.SetMetaData("Kinds", "domain domain list")
    sitk_img.SetMetaData("UltrasoundImageOrientation", "MFA")

    for i in range(len(transformation_list)):
        seq_num = "0"*(4-len(str(i))) + str(i)
        sitk_img.SetMetaData("Seq_Frame"+seq_num+"_ProbeToTrackerTransform", str(transformation_list[i])[1:-1].replace(",",""))
        sitk_img.SetMetaData("Seq_Frame"+seq_num+"_ProbeToTrackerTransformStatus", str("OK"))
        sitk_img.SetMetaData("Seq_Frame"+seq_num+"_Timestamp", str(i))
        sitk_img.SetMetaData("Seq_Frame"+seq_num+"_ImageStatus", str("OK"))

    writer = sitk.ImageFileWriter()
    writer.SetFileName(file_name)
    writer.Execute(sitk_img)
    print("Saved file name: " + file_name)
    return sitk_img

def MhaFileMeter2Pixel(volume, spacing ,file_name):
    # volume = sitk.ReadImage("./dataset/thyroid.mhd")
    dim = volume.GetSize()[2]
    # for i in range(dim):
    #     seq_num = "0"*(4-len(str(i))) + str(i)
    #     tf = volume.GetMetaData("Seq_Frame"+seq_num+"_ProbeToTrackerTransform")
    #     tf = tf.split( )
    #     for i in range(len(tf)):
    #         tf[i] = float(tf[i])
    #     tf[3] = tf[3]*1000/spacing
    #     tf[7] = tf[7]*1000/spacing
    #     tf[11] = tf[11]*1000/spacing
    #     volume.SetMetaData("Seq_Frame"+seq_num+"_ProbeToTrackerTransform", str(tf)[1:-1].replace(",",""))

    img_array = sitk.GetArrayFromImage(volume)
    print(img_array.shape)
    img_array = img_array.swapaxes(1,2)
    img_array = np.flip(img_array, 1)
    print(img_array.shape)
    sitk_img = sitk.GetImageFromArray(img_array)
    sitk_img.SetSpacing([1, 1, 1])
    sitk_img.SetMetaData("Kinds", "domain domain list")
    sitk_img.SetMetaData("UltrasoundImageOrientation", "MFA")

    for i in range(dim):
        seq_num = "0"*(4-len(str(i))) + str(i)
        tf = volume.GetMetaData("Seq_Frame"+seq_num+"_ProbeToTrackerTransform")
        tf = tf.split( )
        for i in range(len(tf)):
            tf[i] = float(tf[i])
        tf[3] = tf[3]*1000/spacing
        tf[7] = tf[7]*1000/spacing
        tf[11] = tf[11]*1000/spacing
        sitk_img.SetMetaData("Seq_Frame"+seq_num+"_ProbeToTrackerTransform", str(tf)[1:-1].replace(",",""))
        # sitk_img.SetMetaData("Seq_Frame"+seq_num+"_ProbeToTrackerTransform", str(transformation_list[i])[1:-1].replace(",",""))
        sitk_img.SetMetaData("Seq_Frame"+seq_num+"_ProbeToTrackerTransformStatus", str("OK"))
        sitk_img.SetMetaData("Seq_Frame"+seq_num+"_Timestamp", str(i))
        sitk_img.SetMetaData("Seq_Frame"+seq_num+"_ImageStatus", str("OK"))


    writer = sitk.ImageFileWriter()
    writer.SetFileName(file_name)
    writer.Execute(sitk_img)
    print("Saved file name: " + file_name)


def GenerateTestData(volume, slices_num):
    np_img_list = []
    transformation_list = []
    # # volume = sitk.ReadImage("./test.nrrd")
    # slice_width = 256
    # slcie_height = 256
    # spacing = volume.GetSpacing()[0]
    # out_spaceing = list((spacing, spacing)) + [1]
    # origin = (0,0,0)
    # groundtruth = sitk.Euler3DTransform()
    # gt = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # pixel_gt = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # for i in range(slices_num):
    #     # gt[0] = gt[0] + 0.1
    #     # gt[1] = gt[1] + 0.2
    #     # gt[2] = gt[2] + 3.14/6
    #     gt[0] = random.uniform(0, 3.14/180*2)
    #     gt[1] = random.uniform(0, 3.14/180*2)
    #     gt[2] = random.uniform(0, 3.14/180*2)
    #     # gt[4] = gt[4] + 10
    #     z_interval = 0.18815431515639
    #     gt[5] = gt[5] + z_interval
    #     pixel_gt[5] = pixel_gt[5] + z_interval/spacing

    #     matrix = tf.transformations.compose_matrix(angles=gt[0:3], translate=pixel_gt[3:])
    #     # matrix = tf.transformations.compose_matrix(angles=gt[0:3], translate=[-gt[3], -gt[4], -gt[5]])
    #     transformation_list.append(list(matrix.flatten()))
    #     # print(list(matrix.flatten()))

    #     groundtruth.SetParameters(gt)
    #     slice = ExtractSliceFromVolume.Execute(volume, groundtruth, slice_width, slcie_height, out_spaceing, origin)
    #     slice_array = sitk.GetArrayFromImage(slice)
    #     np_img_list.append(slice_array)
        
    return np_img_list, transformation_list

if __name__ == '__main__':
    # volume = sitk.ReadImage("./dataset/thyroid.mhd")
    # np_img_list, transformation_list = GenerateTestData(volume, 500)
    # SaveImage(np_img_list, transformation_list, "thyroid_sample.mha")
    volume = sitk.ReadImage("thyroid_sample_raw.mha")
    MhaFileMeter2Pixel(volume, 46.0/527.0, "thyroid_sample.mha")