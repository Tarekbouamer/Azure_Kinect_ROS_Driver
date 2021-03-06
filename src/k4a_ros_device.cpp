// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"


// System headers
//
#include <thread>

// Library headers
//
#include <angles/angles.h>
#include <cv_bridge/cv_bridge.h>
#include <k4a/k4a.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <k4a/k4a.hpp>

#include <iostream>



// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_types.h"

using namespace ros;
using namespace sensor_msgs;
using namespace image_transport;
using namespace std;

#if defined(K4A_BODY_TRACKING)
using namespace visualization_msgs;
#endif



std::string syncStatus;






/*
* ####################################################################################################################################################
*/ 
#define VERIFY(result)                                                                                                 \
    if (result != K4A_RESULT_SUCCEEDED)                                                                                \
    {                                                                                                                  \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", #result " failed", __FILE__, __FUNCTION__, __LINE__);   \
        exit(1);                                                                                                       \
    }

#define FOURCC(cc) ((cc)[0] | (cc)[1] << 8 | (cc)[2] << 16 | (cc)[3] << 24)


typedef struct
{
    uint32_t biSize;
    uint32_t biWidth;
    uint32_t biHeight;
    uint16_t biPlanes;
    uint16_t biBitCount;
    uint32_t biCompression;
    uint32_t biSizeImage;
    uint32_t biXPelsPerMeter;
    uint32_t biYPelsPerMeter;
    uint32_t biClrUsed;
    uint32_t biClrImportant;
} BITMAPINFOHEADER;


void fill_bitmap_header(uint32_t width, uint32_t height, BITMAPINFOHEADER *out)
{
    out->biSize = sizeof(BITMAPINFOHEADER);
    out->biWidth = width;
    out->biHeight = height;
    out->biPlanes = 1;
    out->biBitCount = 16;
    out->biCompression = FOURCC("YUY2");
    out->biSizeImage = sizeof(uint16_t) * width * height;
    out->biXPelsPerMeter = 0;
    out->biYPelsPerMeter = 0;
    out->biClrUsed = 0;
    out->biClrImportant = 0;
}
/*
* ####################################################################################################################################################
*/

K4AROSDevice::K4AROSDevice(const NodeHandle& n, const NodeHandle& p)
  : k4a_device_(nullptr),
    k4a_playback_handle_(nullptr),
    k4a_recording_handle_(nullptr),
// clang-format off
#if defined(K4A_BODY_TRACKING)
    k4abt_tracker_(nullptr),
#endif
    // clang-format on
    node_(n),
    private_node_(p),
    image_transport_(n),
    last_capture_time_usec_(0),
    save_capture_time(0),
    last_imu_time_usec_(0),
    imu_stream_end_of_file_(false)
{
  // Collect ROS parameters from the param server or from the command line
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
  private_node_.param(#param_variable, params_.param_variable, param_default_val);
  ROS_PARAM_LIST
#undef LIST_ENTRY

  if (params_.recording_file != "")
  {
    ROS_INFO("Node is started in playback mode");
    ROS_INFO_STREAM("Try to open recording file " << params_.recording_file);

    // Open recording file and print its length
    k4a_playback_handle_ = k4a::playback::open(params_.recording_file.c_str());
    auto recording_length = k4a_playback_handle_.get_recording_length();
    ROS_INFO_STREAM("Successfully openend recording file. Recording is " << recording_length.count() / (double)1000000
                                                                         << " seconds long");

    // Get the recordings configuration to overwrite node parameters

    k4a_record_configuration_t record_config = k4a_playback_handle_.get_record_configuration();

    // Overwrite fps param with recording configuration for a correct loop rate in the frame publisher thread
    switch (record_config.camera_fps)
    {
      case K4A_FRAMES_PER_SECOND_5:
        params_.fps = 5;
        break;
      case K4A_FRAMES_PER_SECOND_15:
        params_.fps = 15;
        break;
      case K4A_FRAMES_PER_SECOND_30:
        params_.fps = 30;
        break;
      default:
        break;
        
    };

    // Disable color if the recording has no color track
    if (params_.color_enabled && !record_config.color_track_enabled)
    {
      ROS_WARN("Disabling color and rgb_point_cloud because recording has no color track");
      params_.color_enabled = false;
      params_.rgb_point_cloud = false;
    }
    // This is necessary because at the moment there are only checks in place which use BgraPixel size
    else if (params_.color_enabled && record_config.color_track_enabled)
    {
      if (params_.color_format == "jpeg" && record_config.color_format != K4A_IMAGE_FORMAT_COLOR_MJPG)
      {
        ROS_FATAL("Converting color images to K4A_IMAGE_FORMAT_COLOR_MJPG is not supported.");
        ros::requestShutdown();
        return;
      }
      if (params_.color_format == "bgra" && record_config.color_format != K4A_IMAGE_FORMAT_COLOR_BGRA32)
      {
        k4a_playback_handle_.set_color_conversion(K4A_IMAGE_FORMAT_COLOR_BGRA32);
      }
    }

    // Disable depth if the recording has neither ir track nor depth track
    if (!record_config.ir_track_enabled && !record_config.depth_track_enabled)
    {
      if (params_.depth_enabled)
      {
        ROS_WARN("Disabling depth because recording has neither ir track nor depth track");
        params_.depth_enabled = false;
      }
    }

    // Disable depth if the recording has no depth track
    if (!record_config.depth_track_enabled)
    {
      if (params_.point_cloud)
      {
        ROS_WARN("Disabling point cloud because recording has no depth track");
        params_.point_cloud = false;
      }
      if (params_.rgb_point_cloud)
      {
        ROS_WARN("Disabling rgb point cloud because recording has no depth track");
        params_.rgb_point_cloud = false;
      }
    }
  }
  else
  {
    // Print all parameters
    ROS_INFO("K4A Parameters:");
    params_.Print();

    // Setup the K4A device
    uint32_t k4a_device_count = k4a::device::get_installed_count();

    ROS_INFO_STREAM("Found " << k4a_device_count << " sensors");

    if (params_.sensor_sn != "")
    {
      ROS_INFO_STREAM("Searching for sensor with serial number: " << params_.sensor_sn);
    }
    else
    {
      ROS_INFO("No serial number provided: picking first sensor");
      ROS_WARN_COND(k4a_device_count > 1, "Multiple sensors connected! Picking first sensor.");
    }

    for (uint32_t i = 0; i < k4a_device_count; i++)
    {
      k4a::device device;
      try
      {
        device = k4a::device::open(i);
      }
      catch (exception)
      {
        ROS_ERROR_STREAM("Failed to open K4A device at index " << i);
        continue;
      }

      ROS_INFO_STREAM("K4A[" << i << "] : " << device.get_serialnum());

      // Try to match serial number
      if (params_.sensor_sn != "")
      {
        if (device.get_serialnum() == params_.sensor_sn)
        {
          k4a_device_ = std::move(device);
          break;
        }
      }
      // Pick the first device
      else if (i == 0)
      {
        k4a_device_ = std::move(device);
        break;
      }
    }

    if (!k4a_device_)
    {
      ROS_ERROR("Failed to open a K4A device. Cannot continue.");
      return;
    }

    k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_result_t result = params_.GetDeviceConfig(&device_config);   

    if (params_.recording_folder !="")
    {
      ROS_INFO_STREAM("Recording will be saved in  : " << params_.recording_folder);
      k4a_recording_handle_ = k4a::record::create(params_.recording_folder.c_str(), k4a_device_, device_config);
      
      // Add a custom recording tag labeling the camera streaming 
      k4a_recording_handle_.add_tag(  "CUSTOM_TAG", params_.file_name.c_str() );

      //if custom capturing is needed in the future follow the following depth template from MS 
      // examples SDK  
      /*
      k4a_calibration_t sensor_calibration;
      sensor_calibration = k4a_device_.get_calibration( device_config.depth_mode, 
                                                      K4A_COLOR_RESOLUTION_OFF );

      uint32_t depth_width  = (uint32_t)sensor_calibration.depth_camera_calibration.resolution_width;
      uint32_t depth_height = (uint32_t)sensor_calibration.depth_camera_calibration.resolution_height;
    
      k4a_record_video_settings_t video_settings;
      video_settings.width      = depth_width ;
      video_settings.height     = depth_height;
      video_settings.frame_rate = params_.fps; // Should be the same rate as params_.camera_fps
  
      BITMAPINFOHEADER codec_header;
      fill_bitmap_header( video_settings.width, 
                        video_settings.height, 
                        &codec_header
                        );

      k4a_recording_handle_.add_custom_video_track( "PROCESSED_DEPTH",
                                                  "V_MS/VFW/FOURCC",
                                                 (uint8_t *)(&codec_header),
                                                  sizeof(codec_header),
                                                  &video_settings);

      */
      k4a_recording_handle_.write_header();

    }  
 
    ROS_INFO_STREAM("K4A Serial Number: " << k4a_device_.get_serialnum());

    k4a_hardware_version_t version_info = k4a_device_.get_version();

    ROS_INFO("RGB Version: %d.%d.%d",           version_info.rgb.major  ,         version_info.rgb.minor  ,         version_info.rgb.iteration);
    ROS_INFO("Depth Version: %d.%d.%d",         version_info.depth.major,         version_info.depth.minor,         version_info.depth.iteration);
    ROS_INFO("Audio Version: %d.%d.%d",         version_info.audio.major,         version_info.audio.minor,         version_info.audio.iteration);
    ROS_INFO("Depth Sensor Version: %d.%d.%d",  version_info.depth_sensor.major,  version_info.depth_sensor.minor,  version_info.depth_sensor.iteration);
  }

  //>> TO DO : Extend it to other video formats if possible with ROS.

  if (params_.color_format == "jpeg")
  {
    // JPEG images are directly published on 'rgb/image_raw/compressed' so that
    // others can subscribe to 'rgb/image_raw' with compressed_image_transport.
    // This technique is described in:
    // http://wiki.ros.org/compressed_image_transport#Publishing_compressed_images_directly
    rgb_jpeg_publisher_ = node_.advertise<CompressedImage>(node_.resolveName("rgb/image_raw") + "/compressed", 1);
  }
  else if (params_.color_format == "bgra")
  {
    rgb_raw_publisher_ = image_transport_.advertise("rgb/image_raw", 1);
  }

  rgb_raw_camerainfo_publisher_     = node_.advertise<CameraInfo>("rgb/camera_info", 1);
  
  depth_raw_publisher_              = image_transport_.advertise("depth/image_raw", 1);
  depth_raw_camerainfo_publisher_   = node_.advertise<CameraInfo>("depth/camera_info", 1);
  
  depth_rect_publisher_             = image_transport_.advertise("depth_to_rgb/image_raw", 1);
  depth_rect_camerainfo_publisher_  = node_.advertise<CameraInfo>("depth_to_rgb/camera_info", 1);
  
  rgb_rect_publisher_               = image_transport_.advertise("rgb_to_depth/image_raw", 1);
  rgb_rect_camerainfo_publisher_    = node_.advertise<CameraInfo>("rgb_to_depth/camera_info", 1);
  
  ir_raw_publisher_                 = image_transport_.advertise("ir/image_raw", 1);
  ir_raw_camerainfo_publisher_      = node_.advertise<CameraInfo>("ir/camera_info", 1);
  
  imu_orientation_publisher_        = node_.advertise<Imu>("imu", 200);
  pointcloud_publisher_             = node_.advertise<PointCloud2>("points2", 1);

#if defined(K4A_BODY_TRACKING)
  body_marker_publisher_ = node_.advertise<MarkerArray>("body_tracking_data", 1);

  body_index_map_publisher_ = image_transport_.advertise("body_index_map/image_raw", 1);
#endif
}

K4AROSDevice::~K4AROSDevice()
{
  // Start tearing down the publisher threads
  running_ = false;

  // Join the publisher thread
  ROS_INFO("Joining camera publisher thread");
  frame_publisher_thread_.join();
  ROS_INFO("Camera publisher thread joined");

  // Join the publisher thread
  ROS_INFO("Joining IMU publisher thread");
  imu_publisher_thread_.join();
  ROS_INFO("IMU publisher thread joined");
  
  //stop recording  
  if(k4a_recording_handle_)
  {
    k4a_recording_handle_.flush();
    ROS_INFO("PAUSE RECODING ON DISK DRIVE ");
    k4a_recording_handle_.close();
    ROS_INFO("STOP RECODING ........");

  }
  //stop playingback  

  if (k4a_playback_handle_)
  {
    ROS_INFO("STOP PLAYINGBACK ........ " );
    k4a_playback_handle_.close();
  }
  //stop device Camera && IMU
  stopCameras();
  stopImu();

#if defined(K4A_BODY_TRACKING)
  if (k4abt_tracker_)
  {
    k4abt_tracker_.shutdown();
  }
#endif
}

k4a_result_t K4AROSDevice::startCameras()
{
  k4a_device_configuration_t k4a_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  k4a_result_t result = params_.GetDeviceConfig(&k4a_configuration);

  if (k4a_device_)
  {
    if (result != K4A_RESULT_SUCCEEDED)
    {
      ROS_ERROR("Failed to generate a device configuration. Not starting camera!");
      return result;
    }
  

    // Now that we have a proposed camera configuration, we can
    // initialize the class which will take care of device calibration information
    calibration_data_.initialize(k4a_device_, k4a_configuration.depth_mode, k4a_configuration.color_resolution,
                                 params_);
  }
  else if (k4a_playback_handle_)
  {
    // initialize the class which will take care of device calibration information from the playback_handle
    calibration_data_.initialize(k4a_playback_handle_, params_);
  }

#if defined(K4A_BODY_TRACKING)
  // When calibration is initialized the body tracker can be created with the device calibration
  if (params_.body_tracking_enabled)
  {
    k4abt_tracker_ = k4abt::tracker::create(calibration_data_.k4a_calibration_);
    k4abt_tracker_.set_temporal_smoothing(params_.body_tracking_smoothing_factor);
  }
#endif

  if (k4a_device_)
  {
    ROS_INFO_STREAM("STARTING CAMERAS");
    k4a_device_.start_cameras(&k4a_configuration);
  }

  // Cannot assume the device timestamp begins increasing upon starting the cameras.
  // If we set the time base here, depending on the machine performance, the new timestamp
  // would lag the value of ros::Time::now() by at least 0.5 secs which is much larger than
  // the real transmission delay as can be observed using the rqt_plot tool.
  // start_time_ = ros::Time::now();

  // Prevent the worker thread from exiting immediately
  running_ = true;

  // Start the thread that will poll the cameras and publish frames
  frame_publisher_thread_ = thread(&K4AROSDevice::framePublisherThread, this);

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::startImu()
{
  if (k4a_device_)
  {
    ROS_INFO_STREAM("STARTING IMU");
    k4a_device_.start_imu();
  }

  // Start the IMU publisher thread
  imu_publisher_thread_ = thread(&K4AROSDevice::imuPublisherThread, this);

  return K4A_RESULT_SUCCEEDED;
}

void K4AROSDevice::stopCameras()
{
  if (k4a_device_)
  {
    // Stop the K4A SDK
    ROS_INFO("Stopping K4A device");
    k4a_device_.stop_cameras();
    ROS_INFO("K4A device stopped");
  }
}

void K4AROSDevice::stopImu()
{
  if (k4a_device_)
  {
    k4a_device_.stop_imu();
  }
}

bool saveDepthFrame(k4a::image& k4a_frame, const std::string path)
{
  std::string visPath = path;     visPath.append("_Depth_Vis.png");
  std::string rawPath = path;     rawPath.append("_Depth.png");
  std::string pfmPath = path;     pfmPath.append("_Depth.pfm");

  cv::Mat depth_frame_buffer_mat(k4a_frame.get_height_pixels(), k4a_frame.get_width_pixels(), CV_16UC1,
                                 k4a_frame.get_buffer());

  cv::Mat vis_depth(k4a_frame.get_height_pixels(), k4a_frame.get_width_pixels(), CV_32FC1);        // visualization
  cv::Mat raw_depth(k4a_frame.get_height_pixels(), k4a_frame.get_width_pixels(), CV_32FC1);       // Raw

  // Fill in the depth image data, converting mm to m
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  for(size_t i = 0; i < depth_frame_buffer_mat.rows; i++)
  {
    for(size_t j = 0; j < depth_frame_buffer_mat.cols; j++)
    {
      uint16_t pDepth = depth_frame_buffer_mat.at<uint16_t>(i,j);
      raw_depth.at<float>(i,j) = (pDepth == 0) ? bad_point : (float) pDepth * 0.001f;
    }
  }   

  WriteFilePFM(raw_depth, pfmPath, 1/255.0);
  
  for(size_t i = 0; i < depth_frame_buffer_mat.rows; i++)
  {
    for(size_t j = 0; j < depth_frame_buffer_mat.cols; j++)
    {
      uint16_t pDepth = depth_frame_buffer_mat.at<uint16_t>(i,j);
      vis_depth.at<float>(i,j) = (pDepth == 0) ? bad_point : (float) pDepth * 0.1f;  // ;) just Gray Scale Visulaization   
    }
  }
  
  //cv::imwrite( visPath, vis_depth );  
  return cv::imwrite( rawPath, raw_depth );
};

k4a_result_t K4AROSDevice::getDepthFrame(const k4a::capture& capture, sensor_msgs::ImagePtr& depth_image,
                                        std::string path,
                                        bool rectified = false)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();

  if (!k4a_depth_frame)
  {
    ROS_ERROR("Cannot render depth frame: no frame");
    return K4A_RESULT_FAILED;
  }

  if(!rectified && k4a_playback_handle_)
  {
    // save Depth frame *.jpg
    if (!saveDepthFrame(k4a_depth_frame,path))
    {
      ROS_ERROR("Cannot save Depth frame to folder");
      return K4A_RESULT_FAILED;
    }
  }
  
  if (rectified)
  {
    calibration_data_.k4a_transformation_.depth_image_to_color_camera(k4a_depth_frame,
                                                                      &calibration_data_.transformed_depth_image_);
    return renderDepthToROS(depth_image, calibration_data_.transformed_depth_image_);
  }

  return renderDepthToROS(depth_image, k4a_depth_frame);
}

k4a_result_t K4AROSDevice::renderDepthToROS(sensor_msgs::ImagePtr& depth_image, k4a::image& k4a_depth_frame)
{
  cv::Mat depth_frame_buffer_mat(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_16UC1,
                                 k4a_depth_frame.get_buffer());
  cv::Mat new_image(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_32FC1);

  depth_frame_buffer_mat.convertTo(new_image, CV_32FC1, 1.0 / 1000.0f);

  depth_image =cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, new_image).toImageMsg();

  return K4A_RESULT_SUCCEEDED;
}


bool saveIrFrame(k4a::image& k4a_frame, const std::string path)
{
  cv::Mat Ir_buffer_mat(k4a_frame.get_height_pixels(), k4a_frame.get_width_pixels(), CV_16UC1,
                                 k4a_frame.get_buffer());

  return cv::imwrite( path, Ir_buffer_mat );
};

k4a_result_t K4AROSDevice::getIrFrame(const k4a::capture& capture, sensor_msgs::ImagePtr& ir_image,
                                      std::string path)
{
  k4a::image k4a_ir_frame = capture.get_ir_image();

  if (!k4a_ir_frame)
  {
    ROS_ERROR("Cannot render IR frame: no frame");
    return K4A_RESULT_FAILED;
  }

  if(k4a_playback_handle_)
  {
    path.append("_IR.png");
    if (!saveIrFrame(k4a_ir_frame, path))
    {
      ROS_ERROR("Cannot save IR frame to folder");
      return K4A_RESULT_FAILED;
   }
  } 
  return renderIrToROS(ir_image, k4a_ir_frame);
}


k4a_result_t K4AROSDevice::renderIrToROS(sensor_msgs::ImagePtr& ir_image, k4a::image& k4a_ir_frame)
{
  cv::Mat ir_buffer_mat(k4a_ir_frame.get_height_pixels(), k4a_ir_frame.get_width_pixels(), CV_16UC1,
                        k4a_ir_frame.get_buffer());

  

  // Rescale the image to mono8 for visualization and usage for visual(-inertial) odometry.
  if (params_.rescale_ir_to_mono8)
  {
    cv::Mat new_image(k4a_ir_frame.get_height_pixels(), k4a_ir_frame.get_width_pixels(), CV_8UC1);
    // Use a scaling factor to re-scale the image. If using the illuminators, a value of 1 is appropriate.
    // If using PASSIVE_IR, then a value of 10 is more appropriate; k4aviewer does a similar conversion.
    ir_buffer_mat.convertTo(new_image, CV_8UC1, params_.ir_mono8_scaling_factor);
    ir_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, new_image).toImageMsg();
  }
  else
  {
    ir_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO16, ir_buffer_mat).toImageMsg();
  }

  return K4A_RESULT_SUCCEEDED;
}

bool saveJpegRgbFrame(k4a::image& k4a_frame, const std::string path)
{
  cv::Mat rgb_buffer_mat(k4a_frame.get_height_pixels(), k4a_frame.get_width_pixels(), CV_8UC4,
                         k4a_frame.get_buffer());                             

  return cv::imwrite( path, rgb_buffer_mat );
};

k4a_result_t K4AROSDevice::getJpegRgbFrame(const k4a::capture& capture, sensor_msgs::CompressedImagePtr& jpeg_image,
                                          std::string path)
{
  k4a::image k4a_jpeg_frame = capture.get_color_image();

  if (!k4a_jpeg_frame)
  {
    ROS_ERROR("Cannot render Jpeg frame: no frame");
    return K4A_RESULT_FAILED;
  }

  if (k4a_playback_handle_)
  {
    // save image frame *.jpg
    path.append("_JPEG.png");
    if(!saveJpegRgbFrame(k4a_jpeg_frame, path))
    {
      ROS_ERROR("Cannot save BGRA frame to folder");
      return K4A_RESULT_FAILED;
    }
  }
  const uint8_t* jpeg_frame_buffer = k4a_jpeg_frame.get_buffer();
  jpeg_image->format = "jpeg";
  jpeg_image->data.assign(jpeg_frame_buffer, jpeg_frame_buffer + k4a_jpeg_frame.get_size());
  return K4A_RESULT_SUCCEEDED;
}

bool saveRbgFrame(k4a::image& k4a_frame, const std::string path)
{
  cv::Mat rgb_buffer_mat(k4a_frame.get_height_pixels(), k4a_frame.get_width_pixels(), CV_8UC4,
                         k4a_frame.get_buffer());                             

  return cv::imwrite( path, rgb_buffer_mat );
};

k4a_result_t K4AROSDevice::getRbgFrame(const k4a::capture& capture, sensor_msgs::ImagePtr& rgb_image,
                                      std::string path,
                                       bool rectified = false)
{
  k4a::image k4a_bgra_frame = capture.get_color_image();

  if (!k4a_bgra_frame)
  {
    ROS_ERROR("Cannot render BGRA frame: no frame");
    return K4A_RESULT_FAILED;
  }
  
  if (!rectified && k4a_playback_handle_)
  {
    // save image frame *.jpg
    path.append("_RGB.png");
    
    if (!saveRbgFrame(k4a_bgra_frame,path))
    {
      ROS_ERROR("Cannot save BGRA frame to folder");
      return K4A_RESULT_FAILED;
    }
  }

  size_t color_image_size =
      static_cast<size_t>(k4a_bgra_frame.get_width_pixels() * k4a_bgra_frame.get_height_pixels()) * sizeof(BgraPixel);

  if (k4a_bgra_frame.get_size() != color_image_size)
  {
    ROS_WARN("Invalid k4a_bgra_frame returned from K4A");
    return K4A_RESULT_FAILED;
  }

  if (rectified)
  {
    k4a::image k4a_depth_frame = capture.get_depth_image();

    calibration_data_.k4a_transformation_.color_image_to_depth_camera(k4a_depth_frame, k4a_bgra_frame,
                                                                      &calibration_data_.transformed_rgb_image_);

    return renderBGRA32ToROS(rgb_image, calibration_data_.transformed_rgb_image_);
  }

  return renderBGRA32ToROS(rgb_image, k4a_bgra_frame);
}

// Helper function that renders any BGRA K4A frame to a ROS ImagePtr. Useful for rendering intermediary frames
// during debugging of image processing functions
k4a_result_t K4AROSDevice::renderBGRA32ToROS(sensor_msgs::ImagePtr& rgb_image, k4a::image& k4a_bgra_frame)
{
  cv::Mat rgb_buffer_mat(k4a_bgra_frame.get_height_pixels(), k4a_bgra_frame.get_width_pixels(), CV_8UC4,
                         k4a_bgra_frame.get_buffer());

  rgb_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGRA8, rgb_buffer_mat).toImageMsg();

  return K4A_RESULT_SUCCEEDED;
}

bool saveImageToDepth(k4a::image* k4a_frame, std::string path)
{
  cv::Mat rgb_buffer_mat(k4a_frame->get_height_pixels(), k4a_frame->get_width_pixels(), CV_8UC4,
                         k4a_frame->get_buffer());                             

  return cv::imwrite( path, rgb_buffer_mat );

}

bool saveDepthToImage(k4a::image* k4a_frame, std::string path)
{
  std::string visPath = path;     visPath.append("_Depth_Registed_Vis.png");
  std::string rawPath = path;     rawPath.append("_Depth_Registed.png");
  std::string pfmPath = path;     pfmPath.append("_Depth_Registed.pfm");

  cv::Mat depth_frame_buffer_mat(k4a_frame->get_height_pixels(), k4a_frame->get_width_pixels(), CV_16UC1,
                                 k4a_frame->get_buffer());

  cv::Mat vis_depth(k4a_frame->get_height_pixels(), k4a_frame->get_width_pixels(), CV_32FC1);        // visualization
  cv::Mat raw_depth(k4a_frame->get_height_pixels(), k4a_frame->get_width_pixels(), CV_32FC1);       // Raw

  // Fill in the depth image data, converting mm to m
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  for(size_t i = 0; i < depth_frame_buffer_mat.rows; i++)
  {
    for(size_t j = 0; j < depth_frame_buffer_mat.cols; j++)
    {
      uint16_t pDepth = depth_frame_buffer_mat.at<uint16_t>(i,j);
      raw_depth.at<float>(i,j) = (pDepth == 0) ? bad_point : (float) pDepth * 0.001f;
    }
  }

  WriteFilePFM(raw_depth, pfmPath, 1/255.0);     

  
  for(size_t i = 0; i < depth_frame_buffer_mat.rows; i++)
  {
    for(size_t j = 0; j < depth_frame_buffer_mat.cols; j++)
    {
      uint16_t pDepth = depth_frame_buffer_mat.at<uint16_t>(i,j);
      vis_depth.at<float>(i,j) = (pDepth == 0) ? bad_point : (float) pDepth * 0.1f;  // ;) just Gray Scale Visulaization   
    }
  }
  
  //cv::imwrite( visPath, vis_depth );  
  return cv::imwrite( rawPath, raw_depth );
}

k4a_result_t K4AROSDevice::getRgbPointCloudInDepthFrame(const k4a::capture& capture,
                                                        sensor_msgs::PointCloud2Ptr& point_cloud,
                                                        std::string path)
{
  const k4a::image k4a_depth_frame = capture.get_depth_image();
  if (!k4a_depth_frame)
  {
    ROS_ERROR("Cannot render RGB point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  const k4a::image k4a_bgra_frame = capture.get_color_image();
  if (!k4a_bgra_frame)
  {
    ROS_ERROR("Cannot render RGB point cloud: no BGRA frame");
    return K4A_RESULT_FAILED;
  }

  // Transform color image into the depth camera frame:
  calibration_data_.k4a_transformation_.color_image_to_depth_camera(k4a_depth_frame, k4a_bgra_frame,
                                                                    &calibration_data_.transformed_rgb_image_);
  


  //calibration_data_.k4a_transformation_.depth_image_to_color_camera(k4a_depth_frame,
   //                                                                   &calibration_data_.transformed_depth_image_);

  std::string pathD = path;
  //std::string pathI = path;

  if (k4a_playback_handle_)
  {
    //save Image to Depth Frame
    pathD.append("_Image_Registed.png");

    if (!saveImageToDepth(&calibration_data_.transformed_rgb_image_, pathD))
    {
      ROS_ERROR("Cannot save Image to Depth Frame to folder");
      return K4A_RESULT_FAILED;
    }

   // save Depth to Image Frame
   // if (!saveDepthToImage(&calibration_data_.transformed_depth_image_, pathI))
   // {
   //   ROS_ERROR("Cannot save Depth to Image Frame to folder");
   //   return K4A_RESULT_FAILED;
   // }
  }

  // Tranform depth image to point cloud
  calibration_data_.k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame, K4A_CALIBRATION_TYPE_DEPTH,
                                                                   &calibration_data_.point_cloud_image_);

  point_cloud->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());
  printTimestampDebugMessage("RGB point cloud", point_cloud->header.stamp);

  return fillColorPointCloud(calibration_data_.point_cloud_image_, calibration_data_.transformed_rgb_image_,
                             point_cloud);
}


k4a_result_t K4AROSDevice::getRgbPointCloudInRgbFrame(const k4a::capture& capture,
                                                      sensor_msgs::PointCloud2Ptr& point_cloud,
                                                      std::string path)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();
  if (!k4a_depth_frame)
  {
    ROS_ERROR("Cannot render RGB point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  k4a::image k4a_bgra_frame = capture.get_color_image();
  if (!k4a_bgra_frame)
  {
    ROS_ERROR("Cannot render RGB point cloud: no BGRA frame");
    return K4A_RESULT_FAILED;
  }

  // transform depth image into color camera geometry
  calibration_data_.k4a_transformation_.depth_image_to_color_camera(k4a_depth_frame,
                                                                    &calibration_data_.transformed_depth_image_);
  if (k4a_playback_handle_)
  {
    // save Depth to Image Frame

    if (!saveDepthToImage(&calibration_data_.transformed_depth_image_, path))
    {
      ROS_ERROR("Cannot save Depth to Image Frame to folder");
      return K4A_RESULT_FAILED;
    }
  }
  // Tranform depth image to point cloud (note that this is now from the perspective of the color camera)
  calibration_data_.k4a_transformation_.depth_image_to_point_cloud(
      calibration_data_.transformed_depth_image_, K4A_CALIBRATION_TYPE_COLOR, &calibration_data_.point_cloud_image_);

  point_cloud->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());
  printTimestampDebugMessage("RGB point cloud", point_cloud->header.stamp);

  return fillColorPointCloud(calibration_data_.point_cloud_image_, k4a_bgra_frame, point_cloud);
}

k4a_result_t K4AROSDevice::getPointCloud(const k4a::capture& capture, sensor_msgs::PointCloud2Ptr& point_cloud)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();

  if (!k4a_depth_frame)
  {
    ROS_ERROR("Cannot render point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  point_cloud->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());
  printTimestampDebugMessage("Point cloud", point_cloud->header.stamp);

  // Tranform depth image to point cloud
  calibration_data_.k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame, K4A_CALIBRATION_TYPE_DEPTH,
                                                                   &calibration_data_.point_cloud_image_);

  return fillPointCloud(calibration_data_.point_cloud_image_, point_cloud);
}

k4a_result_t K4AROSDevice::fillColorPointCloud(const k4a::image& pointcloud_image, const k4a::image& color_image,
                                               sensor_msgs::PointCloud2Ptr& point_cloud)
{
  point_cloud->height = pointcloud_image.get_height_pixels();
  point_cloud->width = pointcloud_image.get_width_pixels();
  point_cloud->is_dense = false;
  point_cloud->is_bigendian = false;

  const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();
  const size_t pixel_count = color_image.get_size() / sizeof(BgraPixel);
  if (point_count != pixel_count)
  {
    ROS_WARN("Color and depth image sizes do not match!");
    return K4A_RESULT_FAILED;
  }

  sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*point_cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*point_cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*point_cloud, "b");

  pcd_modifier.resize(point_count);

  const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());
  const uint8_t* color_buffer = color_image.get_buffer();

  for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    // Z in image frame:
    float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);
    // Alpha value:
    uint8_t a = color_buffer[4 * i + 3];
    if (z <= 0.0f || a == 0)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
      *iter_r = *iter_g = *iter_b = 0;
    }
    else
    {
      constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
      *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
      *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
      *iter_z = kMillimeterToMeter * z;

      *iter_r = color_buffer[4 * i + 2];
      *iter_g = color_buffer[4 * i + 1];
      *iter_b = color_buffer[4 * i + 0];
    }
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::PointCloud2Ptr& point_cloud)
{
  point_cloud->height = pointcloud_image.get_height_pixels();
  point_cloud->width = pointcloud_image.get_width_pixels();
  point_cloud->is_dense = false;
  point_cloud->is_bigendian = false;

  const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();

  sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

  pcd_modifier.resize(point_count);

  const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());

  for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
  {
    float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);

    if (z <= 0.0f)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
    }
    else
    {
      constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
      *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
      *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
      *iter_z = kMillimeterToMeter * z;
    }
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::getImuFrame(const k4a_imu_sample_t& sample, sensor_msgs::ImuPtr& imu_msg)
{
  imu_msg->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.imu_frame_;
  imu_msg->header.stamp = timestampToROS(sample.acc_timestamp_usec);
  printTimestampDebugMessage("IMU", imu_msg->header.stamp);

  // The correct convention in ROS is to publish the raw sensor data, in the
  // sensor coordinate frame. Do that here.
  imu_msg->angular_velocity.x = sample.gyro_sample.xyz.x;
  imu_msg->angular_velocity.y = sample.gyro_sample.xyz.y;
  imu_msg->angular_velocity.z = sample.gyro_sample.xyz.z;

  imu_msg->linear_acceleration.x = sample.acc_sample.xyz.x;
  imu_msg->linear_acceleration.y = sample.acc_sample.xyz.y;
  imu_msg->linear_acceleration.z = sample.acc_sample.xyz.z;

  // Disable the orientation component of the IMU message since it's invalid
  imu_msg->orientation_covariance[0] = -1.0;

  return K4A_RESULT_SUCCEEDED;
}

#if defined(K4A_BODY_TRACKING)
k4a_result_t K4AROSDevice::getBodyMarker(const k4abt_body_t& body, MarkerPtr marker_msg, int jointType,
                                         ros::Time capture_time)
{
  k4a_float3_t position = body.skeleton.joints[jointType].position;
  k4a_quaternion_t orientation = body.skeleton.joints[jointType].orientation;

  marker_msg->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
  marker_msg->header.stamp = capture_time;

  // Set the lifetime to 0.25 to prevent flickering for even 5fps configurations.
  // New markers with the same ID will replace old markers as soon as they arrive.
  marker_msg->lifetime = ros::Duration(0.25);
  marker_msg->id = body.id * 100 + jointType;
  marker_msg->type = Marker::SPHERE;

  Color color = BODY_COLOR_PALETTE[body.id % BODY_COLOR_PALETTE.size()];

  marker_msg->color.a = color.a;
  marker_msg->color.r = color.r;
  marker_msg->color.g = color.g;
  marker_msg->color.b = color.b;

  marker_msg->scale.x = 0.05;
  marker_msg->scale.y = 0.05;
  marker_msg->scale.z = 0.05;

  marker_msg->pose.position.x = position.v[0] / 1000.0f;
  marker_msg->pose.position.y = position.v[1] / 1000.0f;
  marker_msg->pose.position.z = position.v[2] / 1000.0f;
  marker_msg->pose.orientation.w = orientation.wxyz.w;
  marker_msg->pose.orientation.x = orientation.wxyz.x;
  marker_msg->pose.orientation.y = orientation.wxyz.y;
  marker_msg->pose.orientation.z = orientation.wxyz.z;

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::getBodyIndexMap(const k4abt::frame& body_frame, sensor_msgs::ImagePtr body_index_map_image)
{
  k4a::image k4a_body_index_map = body_frame.get_body_index_map();

  if (!k4a_body_index_map)
  {
    ROS_ERROR("Cannot render body index map: no body index map");
    return K4A_RESULT_FAILED;
  }

  return renderBodyIndexMapToROS(body_index_map_image, k4a_body_index_map, body_frame);
}

k4a_result_t K4AROSDevice::renderBodyIndexMapToROS(sensor_msgs::ImagePtr body_index_map_image,
                                                   k4a::image& k4a_body_index_map, const k4abt::frame& body_frame)
{
  // Access the body index map as an array of uint8 pixels
  BodyIndexMapPixel* body_index_map_frame_buffer = k4a_body_index_map.get_buffer();
  auto body_index_map_pixel_count = k4a_body_index_map.get_size() / sizeof(BodyIndexMapPixel);

  // Build the ROS message
  body_index_map_image->height = k4a_body_index_map.get_height_pixels();
  body_index_map_image->width = k4a_body_index_map.get_width_pixels();
  body_index_map_image->encoding = sensor_msgs::image_encodings::MONO8;
  body_index_map_image->is_bigendian = false;
  body_index_map_image->step = k4a_body_index_map.get_width_pixels() * sizeof(BodyIndexMapPixel);

  // Enlarge the data buffer in the ROS message to hold the frame
  body_index_map_image->data.resize(body_index_map_image->height * body_index_map_image->step);

  // If the pixel doesn't belong to a detected body the pixels value will be 255 (K4ABT_BODY_INDEX_MAP_BACKGROUND).
  // If the pixel belongs to a detected body the value is calculated by body id mod 255.
  // This means that up to body id 254 the value is equals the body id.
  // Afterwards it will lose the relation to the body id and is only a information for the segmentation of the image.
  for (size_t i = 0; i < body_index_map_pixel_count; ++i)
  {
    BodyIndexMapPixel val = body_index_map_frame_buffer[i];
    if (val == K4ABT_BODY_INDEX_MAP_BACKGROUND)
    {
      body_index_map_image->data[i] = K4ABT_BODY_INDEX_MAP_BACKGROUND;
    }
    else
    {
      auto body_id = k4abt_frame_get_body_id(body_frame.handle(), val);
      body_index_map_image->data[i] = body_id % K4ABT_BODY_INDEX_MAP_BACKGROUND;
    }
  }

  return K4A_RESULT_SUCCEEDED;
}
#endif

k4a_result_t K4AROSDevice::saveCameraCalibration(k4a_calibration_t CalibData,  std::string path )
{
  // camera calibration will saved in the following format in txt file in playback folder only.
  // H W fx fy cx cy k1 k2 k3 k4 k5 k6 p1 p2 codx cody r

  k4a_calibration_camera_t depthCalibration  = CalibData.depth_camera_calibration;
  k4a_calibration_camera_t colorCalibration  = CalibData.color_camera_calibration;

  k4a_calibration_intrinsics_t depthIntrinsics = depthCalibration.intrinsics;
  k4a_calibration_extrinsics_t depthExtrinsics = depthCalibration.extrinsics;

  k4a_calibration_intrinsics_t colorIntrinsics = colorCalibration.intrinsics;
  k4a_calibration_extrinsics_t colorExtrinsics = colorCalibration.extrinsics;

  k4a_calibration_intrinsic_parameters_t::_param dp =depthIntrinsics.parameters.param;
  k4a_calibration_intrinsic_parameters_t::_param cp =colorIntrinsics.parameters.param;

  ofstream calibFile;
  path.append("_calibrations.txt");
  
  calibFile.open(path);
  
  // depth: H W fx fy cx cy k1 k2 k3 k4 k5 k6 p1 p2 codx cody r
  calibFile << "depth_intrinsics"; calibFile<<"  ";

  calibFile << depthCalibration.resolution_height;calibFile<<"  "; calibFile << depthCalibration.resolution_width;calibFile<<"  ";
  calibFile << dp.fx;calibFile<<"  ";     calibFile << dp.fy;calibFile<<"  ";     calibFile << dp.cx;calibFile<<" ";      calibFile << dp.cy;calibFile<<"  "; 
  calibFile << dp.k1;calibFile<<"  ";     calibFile << dp.k2;calibFile<<"  ";     calibFile << dp.k3;calibFile<<"  "; 
  calibFile << dp.k4;calibFile<<"  ";     calibFile << dp.k5;calibFile<<"  ";     calibFile << dp.k6;calibFile<<"  ";
  calibFile << dp.p1;calibFile<<"  ";     calibFile << dp.p2;calibFile<<"  "; 
  calibFile << dp.codx;calibFile<<"  ";   calibFile << dp.cody;calibFile<<"  "; 
  calibFile << dp.metric_radius;calibFile<<" "; 
  
  calibFile<<"  "; calibFile << "\n";

  // depth  ext    [R T] 
  calibFile << "depth_extrinsics"; calibFile<<"  ";

  calibFile <<depthExtrinsics.rotation[0];calibFile<<"  ";calibFile <<depthExtrinsics.rotation[1];calibFile<<"  ";calibFile <<depthExtrinsics.rotation[2];calibFile<<"  ";
  calibFile <<depthExtrinsics.translation[0];calibFile<<"  ";

  calibFile <<depthExtrinsics.rotation[3];calibFile<<"  ";calibFile <<depthExtrinsics.rotation[4];calibFile<<"  ";calibFile <<depthExtrinsics.rotation[5];calibFile<<"  ";
  calibFile <<depthExtrinsics.translation[1];calibFile<<"  ";
  
  calibFile <<depthExtrinsics.rotation[6];calibFile<<"  ";calibFile <<depthExtrinsics.rotation[7];calibFile<<"  ";calibFile <<depthExtrinsics.rotation[8];calibFile<<"  ";
  calibFile <<depthExtrinsics.translation[2];calibFile<<"  ";

  calibFile<<"  "; calibFile << "\n";

  // color: H W fx fy cx cy k1 k2 k3 k4 k5 k6 p1 p2 codx cody r
  calibFile << "rgb_intrinsics"; calibFile<<"  ";

  calibFile << colorCalibration.resolution_height;calibFile<<"  "; calibFile << colorCalibration.resolution_width;calibFile<<"  ";
  calibFile << cp.fx;calibFile<<"  ";     calibFile << cp.fy;calibFile<<"  ";     calibFile << cp.cx;calibFile<<" ";      calibFile << cp.cy;calibFile<<"  "; 
  calibFile << cp.k1;calibFile<<"  ";     calibFile << cp.k2;calibFile<<"  ";     calibFile << cp.k3;calibFile<<"  "; 
  calibFile << cp.k4;calibFile<<"  ";     calibFile << cp.k5;calibFile<<"  ";     calibFile << cp.k6;calibFile<<"  ";
  calibFile << cp.p1;calibFile<<"  ";     calibFile << cp.p2;calibFile<<"  "; 
  calibFile << cp.codx;calibFile<<"  ";   calibFile << cp.cody;calibFile<<"  "; 
  calibFile << cp.metric_radius;calibFile<<" "; 
    
  calibFile<<"  "; calibFile << "\n";
 
  // rgb  ext    [R T] 
  calibFile << "rgb_extrinsics"; calibFile<<"  ";

  calibFile <<colorExtrinsics.rotation[0];calibFile<<"  ";calibFile <<colorExtrinsics.rotation[1];calibFile<<"  ";calibFile <<colorExtrinsics.rotation[2];calibFile<<"  ";
  calibFile <<colorExtrinsics.translation[0];calibFile<<"  ";
  
  calibFile <<colorExtrinsics.rotation[3];calibFile<<"  ";calibFile <<colorExtrinsics.rotation[4];calibFile<<"  ";calibFile <<colorExtrinsics.rotation[5];calibFile<<"  ";
  calibFile <<colorExtrinsics.translation[1];calibFile<<"  ";

  calibFile <<colorExtrinsics.rotation[6];calibFile<<"  ";calibFile <<colorExtrinsics.rotation[7];calibFile<<"  ";calibFile <<colorExtrinsics.rotation[8];calibFile<<"  ";
  calibFile <<colorExtrinsics.translation[2];calibFile<<"  ";

  calibFile.close();
  return K4A_RESULT_SUCCEEDED;
}
void K4AROSDevice::framePublisherThread()
{
  ros::Rate loop_rate(params_.fps);

  k4a_wait_result_t wait_result;
  k4a_result_t result;

  CameraInfo rgb_raw_camera_info;
  CameraInfo depth_raw_camera_info;
  CameraInfo rgb_rect_camera_info;
  CameraInfo depth_rect_camera_info;
  CameraInfo ir_raw_camera_info;

  Time capture_time;

  k4a::capture capture;

  // Create frame files names using iso time  name_*.*
  std::string path = params_.recording_folder.c_str();
  boost::posix_time::ptime isoTime = boost::posix_time::ptime(capture_time.toBoost());
  std::string isoTimeString = boost::posix_time::to_iso_extended_string(isoTime); 
  path.append("_"); path.append(isoTimeString);

  calibration_data_.getDepthCameraInfo(depth_raw_camera_info);
  calibration_data_.getRgbCameraInfo(rgb_raw_camera_info);
  calibration_data_.getDepthCameraInfo(rgb_rect_camera_info);
  calibration_data_.getRgbCameraInfo(depth_rect_camera_info);
  calibration_data_.getDepthCameraInfo(ir_raw_camera_info);

  // save camera calibration in txt file. in playback mode only
  if (k4a_playback_handle_)
  {
    k4a_calibration_t cameraCalibData = calibration_data_.k4a_calibration_;

    result = saveCameraCalibration( cameraCalibData, path);

    if (result != K4A_RESULT_SUCCEEDED)
    {
      ROS_ERROR_STREAM("Failed to Save Camera calibration .txt ");
      ros::shutdown();
      return;
    }
  }

  std::string swap = syncStatus;

  while (running_ && ros::ok() && !ros::isShuttingDown())
  {
      sub = node_.subscribe("/chatter", 100, callback);

      // Create frame files names using iso time  name_*.*
      path = params_.recording_folder.c_str();
      boost::posix_time::ptime isoTime = boost::posix_time::ptime(capture_time.toBoost());
      std::string isoTimeString = boost::posix_time::to_iso_extended_string(isoTime); 
      path.append("_"); path.append(isoTimeString);

      if (syncStatus =="record" )
      {
        params_.recording_enabled = true;
      
      }
      else
      {
        params_.recording_enabled = false;
      }

      if (syncStatus != swap)
      {
        std::cout << params_.sensor_sn<< " >>>> " << syncStatus << std::endl;
        std::cout << "                      " << std::endl;
        swap = syncStatus;
      }

      if (k4a_device_)
      {
        // TODO: consider appropriate capture timeout based on camera framerate
        if (!k4a_device_.get_capture(&capture, std::chrono::milliseconds(K4A_WAIT_INFINITE)))
        {
          ROS_FATAL("Failed to poll cameras: node cannot continue.");
          ros::requestShutdown();
          return;
        }
      }
      else if (k4a_playback_handle_)
      {
        std::lock_guard<std::mutex> guard(k4a_playback_handle_mutex_);
        if (!k4a_playback_handle_.get_next_capture(&capture))   
        {
          // rewind recording if looping is enabled
          if (params_.recording_loop_enabled)
          {
            k4a_playback_handle_.seek_timestamp(std::chrono::microseconds(0), K4A_PLAYBACK_SEEK_BEGIN);
            k4a_playback_handle_.get_next_capture(&capture);
            imu_stream_end_of_file_ = false;
            last_imu_time_usec_ = 0;
          }
          else
          {
            ROS_INFO("Recording reached end of file. node cannot continue.");
            ros::requestShutdown();
            return;
          }
        }


        save_capture_time = getCaptureTimestamp(capture);
        last_capture_time_usec_ = save_capture_time.count();
      }

      CompressedImagePtr rgb_jpeg_frame(new CompressedImage);
      ImagePtr rgb_raw_frame(new Image);
      ImagePtr rgb_rect_frame(new Image);
      ImagePtr depth_raw_frame(new Image);
      ImagePtr depth_rect_frame(new Image);
      ImagePtr ir_raw_frame(new Image);
      PointCloud2Ptr point_cloud(new PointCloud2);

      
      if (params_.recording_enabled)
      {
        // TODO: consider appropriate capture function calling;  best time to capture
        k4a_recording_handle_.write_capture(capture);
      }
      
      if (params_.depth_enabled)
      {
        // Only do compute if we have subscribers
        // Only create ir frame when we are using a device or we have an ir image.
        // Recordings may not have synchronized captures. For unsynchronized captures without ir image skip ir frame.
        if (
          //(ir_raw_publisher_.getNumSubscribers() > 0 || ir_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
            (k4a_device_ || capture.get_ir_image() != nullptr))
        {
          // IR images are available in all depth modes
          result = getIrFrame(capture, ir_raw_frame , path);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR_STREAM("Failed to get raw IR frame");
            ros::shutdown();
            return;
          }
          else if (result == K4A_RESULT_SUCCEEDED)
          {
            capture_time = timestampToROS(capture.get_ir_image().get_device_timestamp());
            printTimestampDebugMessage("IR image", capture_time);

            // Re-sychronize the timestamps with the capture timestamp
            ir_raw_camera_info.header.stamp = capture_time;
            ir_raw_frame->header.stamp = capture_time;
            ir_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;

            ir_raw_publisher_.publish(ir_raw_frame);
            ir_raw_camerainfo_publisher_.publish(ir_raw_camera_info);
          }
        }

        // Depth images are not available in PASSIVE_IR mode
        if (calibration_data_.k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR)
        {
          // Only create depth frame when we are using a device or we have an depth image.
          // Recordings may not have synchronized captures. For unsynchronized captures without depth image skip depth
          // frame.
          if (
            //(depth_raw_publisher_.getNumSubscribers() > 0 || depth_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
              (k4a_device_ || capture.get_depth_image() != nullptr))
          {
            result = getDepthFrame(capture, depth_raw_frame, path);

            if (result != K4A_RESULT_SUCCEEDED)
            {
              ROS_ERROR_STREAM("Failed to get raw depth frame");
              ros::shutdown();
              return;
            }
            else if (result == K4A_RESULT_SUCCEEDED)
            {
              capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());
              printTimestampDebugMessage("Depth image", capture_time);

              // Re-sychronize the timestamps with the capture timestamp
              depth_raw_camera_info.header.stamp = capture_time;
              depth_raw_frame->header.stamp = capture_time;
              depth_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;

              depth_raw_publisher_.publish(depth_raw_frame);
              depth_raw_camerainfo_publisher_.publish(depth_raw_camera_info);
            }
          }

          // We can only rectify the depth into the color co-ordinates if the color camera is enabled!
          // Only create rect depth frame when we are using a device or we have an depth image.
          // Recordings may not have synchronized captures. For unsynchronized captures without depth image skip rect
          // depth frame.
          if (params_.color_enabled &&
              //(depth_rect_publisher_.getNumSubscribers() > 0 ||
              // depth_rect_camerainfo_publisher_.getNumSubscribers() > 0) &&
              (k4a_device_ || capture.get_depth_image() != nullptr))
          {
            result = getDepthFrame(capture, depth_rect_frame, path,  true/* rectified */);

            if (result != K4A_RESULT_SUCCEEDED)
            {
              ROS_ERROR_STREAM("Failed to get rectifed depth frame");
              ros::shutdown();
              return;
            }
            else if (result == K4A_RESULT_SUCCEEDED)
            {
              capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());
              printTimestampDebugMessage("Depth image", capture_time);

              depth_rect_frame->header.stamp = capture_time;
              depth_rect_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
              depth_rect_publisher_.publish(depth_rect_frame);

              // Re-synchronize the header timestamps since we cache the camera calibration message
              depth_rect_camera_info.header.stamp = capture_time;
              depth_rect_camerainfo_publisher_.publish(depth_rect_camera_info);


            }
          }

  #if defined(K4A_BODY_TRACKING)
          // Publish body markers when body tracking is enabled and a depth image is available
          if (params_.body_tracking_enabled &&
              (body_marker_publisher_.getNumSubscribers() > 0 || body_index_map_publisher_.getNumSubscribers() > 0))
          {
            capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());

            if (!k4abt_tracker_.enqueue_capture(capture))
            {
              ROS_ERROR("Error! Add capture to tracker process queue failed!");
              ros::shutdown();
              return;
            }
            else
            {
              k4abt::frame body_frame = k4abt_tracker_.pop_result();
              if (body_frame == nullptr)
              {
                ROS_ERROR_STREAM("Pop body frame result failed!");
                ros::shutdown();
                return;
              }
              else
              {
                if (body_marker_publisher_.getNumSubscribers() > 0)
                {
                  // Joint marker array
                  MarkerArrayPtr markerArrayPtr(new MarkerArray);
                  auto num_bodies = body_frame.get_num_bodies();
                  for (size_t i = 0; i < num_bodies; ++i)
                  {
                    k4abt_body_t body = body_frame.get_body(i);
                    for (int j = 0; j < (int) K4ABT_JOINT_COUNT; ++j)
                    {
                      MarkerPtr markerPtr(new Marker);
                      getBodyMarker(body, markerPtr, j, capture_time);
                      markerArrayPtr->markers.push_back(*markerPtr);
                    }
                  }
                  body_marker_publisher_.publish(markerArrayPtr);
                }

                if (body_index_map_publisher_.getNumSubscribers() > 0)
                {
                  // Body index map
                  ImagePtr body_index_map_frame(new Image);
                  result = getBodyIndexMap(body_frame, body_index_map_frame);

                  if (result != K4A_RESULT_SUCCEEDED)
                  {
                    ROS_ERROR_STREAM("Failed to get body index map");
                    ros::shutdown();
                    return;
                  }
                  else if (result == K4A_RESULT_SUCCEEDED)
                  {
                    // Re-sychronize the timestamps with the capture timestamp
                    body_index_map_frame->header.stamp = capture_time;
                    body_index_map_frame->header.frame_id =
                        calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;

                    body_index_map_publisher_.publish(body_index_map_frame);
                  }
                }
              }
            }
          }
  #endif
        }
      }

      if (params_.color_enabled)
      {
        // Only create rgb frame when we are using a device or we have a color image.
        // Recordings may not have synchronized captures. For unsynchronized captures without color image skip rgb frame.
        if (params_.color_format == "jpeg")
        {
          if (
            //(rgb_jpeg_publisher_.getNumSubscribers() > 0 || rgb_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
              (k4a_device_ || capture.get_color_image() != nullptr))
          {
            result = getJpegRgbFrame(capture, rgb_jpeg_frame, path);

            if (result != K4A_RESULT_SUCCEEDED)
            {
              ROS_ERROR_STREAM("Failed to get Jpeg frame");
              ros::shutdown();
              return;
            }

            capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());
            printTimestampDebugMessage("Color image", capture_time);

            rgb_jpeg_frame->header.stamp = capture_time;
            rgb_jpeg_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
            rgb_jpeg_publisher_.publish(rgb_jpeg_frame);
            // Re-synchronize the header timestamps since we cache the camera calibration message
            rgb_raw_camera_info.header.stamp = capture_time;
            rgb_raw_camerainfo_publisher_.publish(rgb_raw_camera_info);
          }
        }
        else if (params_.color_format == "bgra")
        {
          if (
            //(rgb_raw_publisher_.getNumSubscribers() > 0 || rgb_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
              (k4a_device_ || capture.get_color_image() != nullptr))
          {
            result = getRbgFrame(capture, rgb_raw_frame, path);

            if (result != K4A_RESULT_SUCCEEDED)
            {
              ROS_ERROR_STREAM("Failed to get RGB frame");
              ros::shutdown();
              return;
            }

            capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());
            printTimestampDebugMessage("Color image", capture_time);

            rgb_raw_frame->header.stamp = capture_time;
            rgb_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
            rgb_raw_publisher_.publish(rgb_raw_frame);

            // Re-synchronize the header timestamps since we cache the camera calibration message
            rgb_raw_camera_info.header.stamp = capture_time;
            rgb_raw_camerainfo_publisher_.publish(rgb_raw_camera_info);
          }

          // We can only rectify the color into the depth co-ordinates if the depth camera is enabled and processing depth
          // data Only create rgb rect frame when we are using a device or we have a synchronized image. Recordings may
          // not have synchronized captures. For unsynchronized captures image skip rgb rect frame.
          if (params_.depth_enabled && (calibration_data_.k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR) &&
              //(rgb_rect_publisher_.getNumSubscribers() > 0 || rgb_rect_camerainfo_publisher_.getNumSubscribers() > 0) &&
              (k4a_device_ || (capture.get_color_image() != nullptr && capture.get_depth_image() != nullptr)))
          {
            result = getRbgFrame(capture, rgb_rect_frame, path, true /* rectified */);

            if (result != K4A_RESULT_SUCCEEDED)
            {
              ROS_ERROR_STREAM("Failed to get rectifed depth frame");
              ros::shutdown();
              return;
            }

            capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());
            printTimestampDebugMessage("Color image", capture_time);

            rgb_rect_frame->header.stamp = capture_time;
            rgb_rect_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
            rgb_rect_publisher_.publish(rgb_rect_frame);

            // Re-synchronize the header timestamps since we cache the camera calibration message
            rgb_rect_camera_info.header.stamp = capture_time;
            rgb_rect_camerainfo_publisher_.publish(rgb_rect_camera_info);
          }
        }

      }

      // Only create pointcloud when we are using a device or we have a synchronized image.
      // Recordings may not have synchronized captures. In unsynchronized captures skip point cloud.
      if (
        //pointcloud_publisher_.getNumSubscribers() > 0 &&
          (k4a_device_ || (capture.get_color_image() != nullptr && capture.get_depth_image() != nullptr)))
      {   

        if (params_.rgb_point_cloud)
        {
          if (params_.point_cloud_in_depth_frame)
          {
            result = getRgbPointCloudInDepthFrame(capture, point_cloud, path);
          }
          else
          {
            result = getRgbPointCloudInRgbFrame(capture, point_cloud, path);
          }

          if (result != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR_STREAM("Failed to get RGB Point Cloud");
            ros::shutdown();
            return;
          }
        }
        else if (params_.point_cloud)
        {
          result = getPointCloud(capture, point_cloud);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR_STREAM("Failed to get Point Cloud");
            ros::shutdown();
            return;
          }
        }

        if (params_.point_cloud || params_.rgb_point_cloud)
        {
          pointcloud_publisher_.publish(point_cloud);
        }
      }

      if (loop_rate.cycleTime() > loop_rate.expectedCycleTime())
      {
        ROS_WARN_STREAM_THROTTLE(10, "Image processing thread is running behind."
                                        << std::endl
                                        << "Expected max loop time: " << loop_rate.expectedCycleTime() << std::endl
                                        << "Actual loop time: " << loop_rate.cycleTime() << std::endl);
      }

      ros::spinOnce();
      loop_rate.sleep();
    }

  //}
}

k4a_imu_sample_t K4AROSDevice::computeMeanIMUSample(const std::vector<k4a_imu_sample_t>& samples)
{
  // Compute mean sample
  // Using double-precision version of imu sample struct to avoid overflow
  k4a_imu_accumulator_t mean;
  for (auto imu_sample : samples)
  {
    mean += imu_sample;
  }
  float num_samples = samples.size();
  mean /= num_samples;

  // Convert to floating point
  k4a_imu_sample_t mean_float;
  mean.to_float(mean_float);
  // Use most timestamp of most recent sample
  mean_float.acc_timestamp_usec = samples.back().acc_timestamp_usec;
  mean_float.gyro_timestamp_usec = samples.back().gyro_timestamp_usec;

  return mean_float;
}

void K4AROSDevice::imuPublisherThread()
{
  ros::Rate loop_rate(300);    // try other loop rate 300

  k4a_result_t result;
  k4a_imu_sample_t sample;

  // For IMU throttling
  unsigned int count = 0;
  unsigned int target_count = IMU_MAX_RATE / params_.imu_rate_target;
  std::vector<k4a_imu_sample_t> accumulated_samples;
  accumulated_samples.reserve(target_count);
  bool throttling = target_count > 1;

  while (running_ && ros::ok() && !ros::isShuttingDown())
  {

    if (k4a_device_)
    {
      // IMU messages are delivered in batches at 300 Hz. Drain the queue of IMU messages by
      // constantly reading until we get a timeout
      bool read = false;
      do
      {
        read = k4a_device_.get_imu_sample(&sample, std::chrono::milliseconds(0));

        if (read)
        {
          if (throttling)
          {
            accumulated_samples.push_back(sample);
            count++;
          }

          if (count % target_count == 0)
          {
            ImuPtr imu_msg(new Imu);

            if (throttling)
            {
              k4a_imu_sample_t mean_sample_float = computeMeanIMUSample(accumulated_samples);
              result = getImuFrame(mean_sample_float, imu_msg);
              accumulated_samples.clear();
              count = 0;
            }
            else
            {
              result = getImuFrame(sample, imu_msg);
            }

            ROS_ASSERT_MSG(result == K4A_RESULT_SUCCEEDED, "Failed to get IMU frame");

            imu_orientation_publisher_.publish(imu_msg);
          }
        }

      } while (read);
    }
    else if (k4a_playback_handle_)
    {
      // publish imu messages as long as the imu timestamp is less than the last capture timestamp to catch up to the
      // cameras compare signed with unsigned shouldn't cause a problem because timestamps should always be positive
      while (last_imu_time_usec_ <= last_capture_time_usec_ && !imu_stream_end_of_file_)
      {
        std::lock_guard<std::mutex> guard(k4a_playback_handle_mutex_);
        if (!k4a_playback_handle_.get_next_imu_sample(&sample))
        {
          imu_stream_end_of_file_ = true;
        }
        else
        {
          if (throttling)
          {
            accumulated_samples.push_back(sample);
            count++;
          }

          if (count % target_count == 0)
          {
            ImuPtr imu_msg(new Imu);

            if (throttling)
            {
              k4a_imu_sample_t mean_sample_float = computeMeanIMUSample(accumulated_samples);
              result = getImuFrame(mean_sample_float, imu_msg);
              accumulated_samples.clear();
              count = 0;
            }
            else
            {
              result = getImuFrame(sample, imu_msg);
            }

            ROS_ASSERT_MSG(result == K4A_RESULT_SUCCEEDED, "Failed to get IMU frame");

            imu_orientation_publisher_.publish(imu_msg);

            last_imu_time_usec_ = sample.acc_timestamp_usec;
          }
        }
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

std::chrono::microseconds K4AROSDevice::getCaptureTimestamp(const k4a::capture& capture)
{
  // Captures don't actually have timestamps, images do, so we have to look at all the images
  // associated with the capture.  We just return the first one we get back.
  //
  // We check the IR capture instead of the depth capture because if the depth camera is started
  // in passive IR mode, it only has an IR image (i.e. no depth image), but there is no mode
  // where a capture will have a depth image but not an IR image.
  //
  const auto irImage = capture.get_ir_image();
  if (irImage != nullptr)
  {
    return irImage.get_device_timestamp();
  }

  const auto colorImage = capture.get_color_image();
  if (colorImage != nullptr)
  {
    return colorImage.get_device_timestamp();
  }

  return std::chrono::microseconds::zero();
}

// Converts a k4a_image_t timestamp to a ros::Time object
ros::Time K4AROSDevice::timestampToROS(const std::chrono::microseconds& k4a_timestamp_us)
{
  ros::Duration duration_since_device_startup(std::chrono::duration<double>(k4a_timestamp_us).count());

  // Set the time base if it is not set yet. Possible race condition should cause no harm.
  if (start_time_.isZero())
  {
    const ros::Duration transmission_delay(0.11);
    ROS_WARN_STREAM(
        "Setting the time base using a k4a_image_t timestamp. This will result in a "
        "larger uncertainty than setting the time base using the timestamp of a k4a_imu_sample_t sample. "
        "Assuming the transmission delay to be "
        << transmission_delay.toSec() * 1000.0 << " ms.");
    start_time_ = ros::Time::now() - duration_since_device_startup - transmission_delay;
  }
  return start_time_ + duration_since_device_startup;
}

// Converts a k4a_imu_sample_t timestamp to a ros::Time object
ros::Time K4AROSDevice::timestampToROS(const uint64_t& k4a_timestamp_us)
{
  ros::Duration duration_since_device_startup(k4a_timestamp_us / 1e6);

  // Set the time base if it is not set yet.
  if (start_time_.isZero())
  {
    const ros::Duration transmission_delay(0.005);
    ROS_INFO_STREAM(
        "Setting the time base using a k4a_imu_sample_t sample. "
        "Assuming the transmission delay to be "
        << transmission_delay.toSec() * 1000.0 << " ms.");
    start_time_ = ros::Time::now() - duration_since_device_startup - transmission_delay;
  }
  return start_time_ + duration_since_device_startup;
}

void printTimestampDebugMessage(const std::string& name, const ros::Time& timestamp)
{
  ros::Duration lag = ros::Time::now() - timestamp;
  static std::map<const std::string, std::pair<ros::Duration, ros::Duration>> map_min_max;
  auto it = map_min_max.find(name);
  if (it == map_min_max.end())
  {
    map_min_max[name] = std::make_pair(lag, lag);
    it = map_min_max.find(name);
  }
  else
  {
    auto& min_lag = it->second.first;
    auto& max_lag = it->second.second;
    if (lag < min_lag)
    {
      min_lag = lag;
    }
    if (lag > max_lag)
    {
      max_lag = lag;
    }
  }

  ROS_DEBUG_STREAM(name << " timestamp lags ros::Time::now() by\n"
                        << std::setw(23) << lag.toSec() * 1000.0 << " ms. "
                        << "The lag ranges from " << it->second.first.toSec() * 1000.0 << "ms"
                        << " to " << it->second.second.toSec() * 1000.0 << "ms.");
}

void callback(std_msgs::String  msg)
{
  //ROS_INFO("I heard: [%s]", msg.data.c_str());
  syncStatus = msg.data.c_str();
}

void skip_space(fstream& fileStream)
{
    char c;
    do {
        c = fileStream.get();
    } while (c == '\n' || c == ' ' || c == '\t' || c == '\r');
    fileStream.unget();
}

int littleendian()
{
    int intval = 1;
    uchar *uval = (uchar *)&intval;
    return uval[0] == 1;
}

void swapBytes(float* fptr) { 
	uchar* ptr = (uchar *) fptr;
	uchar tmp = 0;
	tmp = ptr[0]; ptr[0] = ptr[3]; ptr[3] = tmp;
	tmp = ptr[1]; ptr[1] = ptr[2]; ptr[2] = tmp;
}

int ReadFilePFM(cv::Mat&im, std::string path){


    fstream file(path.c_str(), ios::in | ios::binary);
    
   
    std::string bands;           // what type is the image   "Pf" = grayscale    (1-band)
                          
    int width, height;      // width and height of the image
    float scalef, fvalue;   // scale factor and temp value to hold pixel value
    cv::Vec3f vfvalue;          // temp value to hold 3-band pixel value

    // extract header information, skips whitespace 
    file >> bands;
    file >> width;
    file >> height;
    file >> scalef;

    // determine endianness 
    int littleEndianFile = (scalef < 0);
    int littleEndianMachine = littleendian();
    int needSwap = (littleEndianFile != littleEndianMachine);

    cout << setfill('=') << setw(19) << "=" << endl;
    cout << "Reading image to pfm file: " << path << endl;
    cout << "Little Endian?: "  << ((needSwap) ? "false" : "true")   << endl;
    cout << "width: "           << width                             << endl;
    cout << "height: "          << height                            << endl;
    cout << "scale: "           << scalef                            << endl;

    // skip SINGLE newline character after reading third arg
    char c = file.get();
    if (c == '\r')      // <cr> in some files before newline
        c = file.get();
    if (c != '\n') {
        if (c == ' ' || c == '\t' || c == '\r'){
            cout << "newline expected";
            return -1;
        }
        else{
        	cout << "whitespace expected";
            return -1;
        }
    }
    
    if(bands == "Pf"){          // handle 1-band image 
        cout << "Reading grayscale image (1-band)" << endl; 
        cout << "Reading into CV_32FC1 image" << endl;
        im = cv::Mat::zeros(height, width, CV_32FC1);
        for (int i=height-1; i >= 0; --i) {
            for(int j=0; j < width; ++j){
                file.read((char*) &fvalue, sizeof(fvalue));
                if(needSwap){
                	swapBytes(&fvalue);
                }
                im.at<float>(i,j) = (float) fvalue;
            }
        }
    }else if(bands == "PF"){    // handle 3-band image
        cout << "Reading color image (3-band)" << endl;
        cout << "Reading into CV_32FC3 image" << endl; 
        im = cv::Mat::zeros(height, width, CV_32FC3);
        for (int i=height-1; i >= 0; --i) {
            for(int j=0; j < width; ++j){
                file.read((char*) &vfvalue, sizeof(vfvalue));
                if(needSwap){
                	swapBytes(&vfvalue[0]);
                	swapBytes(&vfvalue[1]);
                	swapBytes(&vfvalue[2]);
                }
                im.at<cv::Vec3f>(i,j) = vfvalue;
            }
        }
    }else{
        cout << "unknown bands description";
        return -1;
    }
    cout << setfill('=') << setw(19) << "=" << endl << endl;
    return 0;
}

int WriteFilePFM(const cv::Mat &im, string path, float scalef=1/255.0){


    fstream file(path.c_str(), ios::out | ios::binary);

    
    // init variables 
    int type = im.type();
    string bands;
    int width = im.size().width, height = im.size().height;     // width and height of the image 
    float fvalue;       // scale factor and temp value to hold pixel value
    cv::Vec3f vfvalue;      // temp value to hold 3-band pixel value


    switch(type){       // determine identifier string based on image type
        case CV_32FC1:
            bands = "Pf";   // grayscale
            break;
        case CV_32FC3:
            bands = "PF";   // color
            break;
        default:
            cout << "Unsupported image type, must be CV_32FC1 or CV_32FC3";
            return -1;
    }

    // sign of scalefact indicates endianness, see pfms specs
    if(littleendian())
        scalef = -scalef;

    // insert header information 
    file << bands   << "\n";
    file << width   << "\n";
    file << height  << "\n";
    file << scalef  << "\n";
    
    if(bands == "Pf"){          // handle 1-band image 
        for (int i=height-1; i >= 0; --i) {
            for(int j=0; j < width; ++j){
                fvalue = im.at<float>(i,j);
                file.write((char*) &fvalue, sizeof(fvalue));
                
            }
        }
    }else if(bands == "PF"){    // handle 3-band image
        for (int i=height-1; i >= 0; --i) {
            for(int j=0; j < width; ++j){
                vfvalue = im.at<cv::Vec3f>(i,j);
                file.write((char*) &vfvalue, sizeof(vfvalue));
            }
        }
    }else{
        return -1;
    }
    return 0;
}