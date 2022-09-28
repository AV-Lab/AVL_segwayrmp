import math

import matplotlib.pyplot as plt
import pyzed.sl as sl
import numpy as np

def     setupParameters():

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.sdk_verbose = True

    return init_params

def setupObjectDetection():
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking = True
    obj_param.image_sync = True
    obj_param.enable_mask_output = True

    return obj_param

def setupPositionalTracking(obj_params):
    if obj_param.enable_tracking:
        positional_tracking_param = sl.PositionalTrackingParameters()
        positional_tracking_param.set_floor_as_origin = True

    return positional_tracking_param

def setupRecordingVideo(file_name):
    rec_param = sl.RecordingParameters()
    rec_param.compression_mode = sl.SVO_COMPRESSION_MODE.H264
    rec_param.video_filename = file_name

    return rec_param

def verification(enabling_sys, params, camera):
    error = enabling_sys(params)
    if error != sl.ERROR_CODE.SUCCESS:
        print("Error info: ", error)
        camera.close()
        exit(1)

# Camera Initialization
zed = sl.Camera()

# Setup Parameters
init_params = setupParameters()

# Open Camera
error = zed.open(init_params)
if error != sl.ERROR_CODE.SUCCESS:
    exit(1)

# Define Object Detection Parameters
obj_param = setupObjectDetection()

# Setup object positional tracking
positional_tracking_param = setupPositionalTracking(obj_param)


# Enable Obj, Tracking sys
verification(zed.enable_positional_tracking, positional_tracking_param, zed)
verification(zed.enable_object_detection, obj_param, zed)

# Variables
objects   = sl.Objects()
image_zed = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()
#depth_img_data = sl.Mat()
#point_cloud = sl.Mat()
runtime_parameters = sl.RuntimeParameters()
runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL

obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
obj_runtime_param.detection_confidence_threshold = 40


#segmented_video = do_Video('ideoobj22.avi',  height=1242, width=2208)
# 7092 end
# 7092 end
# 7700 startpoints
map2d = np.zeros((250,250,3))
zed_pose = sl.Pose()
#begin 220
#end 3900
py_transform = sl.Transform()
tracking_parameters = sl.PositionalTrackingParameters(py_transform)
err = zed.enable_positional_tracking(tracking_parameters)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)
fused_point_cloud = sl.FusedPointCloud()

# Enable spatial mapping
mapping_parameters = sl.SpatialMappingParameters(map_type=sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD)
err = zed.enable_spatial_mapping(mapping_parameters)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)

zed.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 8)
coordinate = []
#params = setupRecordingVideo('track 6.svo')
#zed.enable_recording(params)
while True:
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)

        # Segway Pos
        py_translation = sl.Translation()
        tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
        ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
        tz = round(zed_pose.get_translation(py_translation).get()[2], 3)

        print('camera', tx, ty, rz)
        
#[[16.311, 31.205], [13.174, 33.273], [-5.179, 20.398], [-3.69, 12.103]]
zed.disable_recording()
zed.close()















