/*
 * File - ros_sensor.cpp
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

/**
 * \cond INTERNAL
 */

//#include "config.h"

#include <limits>

//#include <nodelet/nodelet.h>
//#include <rclcpp::rate.h>
#include "rclcpp/rclcpp.hpp"

/**
 * \endcond
 */

//#include "ros_publisher.h"
#include "ros_sensor.h"
#include "ros_imu_publisher.h"
#include "ros_video_publisher.h"
#include "ros_webcam_publisher.h"
//#include "ros_disparity_publisher.h"
#include "ros_pointcloud_publisher.h"
#include "ros_depth_publisher.h"
#include "ros_fisheye_publisher.h"
#include "ros_slam_publisher.h"
#include "ros_objectdetection_publisher.h"
#include "ros_features_publisher.h"

/**
 * \file ros_sensor.cpp
 *
 * \brief CRosSensor
 */

using namespace std;
static const int connectionTimeout = 10000;
static bool sensorconnectionok = false;

#define myABS(X,Y) (X >= Y) ? (X -Y) : (Y - X)
namespace __INUROS__NAMESPACE__
{
    CRosSensor::CRosSensor(rclcpp::Node::SharedPtr _node, int fps, const std::string _str)
        : CRosPublisherBookkeeping(_str)
        , mLastKnownTemperature(std::numeric_limits<int>::max())
        , node(_node)
        , mTimestampWeb(0)
        , logger(_node->get_logger())
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
    }

    bool CRosSensor::connected(void)
    {
        return state == InuDev::CInuSensor::ESensorState::eStarted;
    }

    int CRosSensor::connect(void)
    {
        while (sensorconnectionok == false)
        {

          bool finnished=(StartStream() != InuDev::EErrorCode::eOK || !StartTemperaStream());
          StopTemperaStream();

          if (finnished)
          {

              sensorconnectionok = false;
              std::this_thread::sleep_for(std::chrono::seconds(2));
              if(sensor)
              {
                  StopStream();
              }
          }
          else
          {
              sensorconnectionok = true;
              state = sensor->GetState();
              RCLCPP_WARN_STREAM(logger, "*** ROSSensor Init succeed ***: ");
              connectionStateTimer = node->create_wall_timer(std::chrono::milliseconds(4000), std::bind(&CRosSensor::connectionCheckCallback, this));
          }
        }
      return 0;
    }

    int CRosSensor::start_publish(void)
    {
        if (callback_handle_) {
            RCLCPP_WARN_STREAM(logger, "CRosSensor is already started.");
            return -1;
        }

        if (publisherList.empty())
        {
            RCLCPP_WARN_STREAM(logger, "Using default topic names. If needed, use topic remapping to change topic name.");

            rclcpp::Node::SharedPtr webcamNode = node->create_sub_node("webcam");
            publisherList.push_back(std::make_shared<CRosWebcamPublisher>(webcamNode, this));
            rclcpp::Node::SharedPtr videoNode = node->create_sub_node("video");
            publisherList.push_back(std::make_shared<CRosVideoPublisher>(videoNode, this));
            rclcpp::Node::SharedPtr depthNode = node->create_sub_node("depth");
            publisherList.push_back(std::make_shared<CRosDepthPublisher>(depthNode, this));
            //rclcpp::Node::SharedPtr disparityNode = node->create_sub_node("disparity");
            //publisherList.push_back(std::make_shared<CRosDisparityPublisher>(disparityNode, this));
            rclcpp::Node::SharedPtr pointcloudNode = node->create_sub_node("pointcloud");
            publisherList.push_back(std::make_shared<CRosPointcloudPublisher>(pointcloudNode, this));
            rclcpp::Node::SharedPtr fisheyeNode = node->create_sub_node("fisheye");
            publisherList.push_back(std::make_shared<CRosFisheyePublisher>(fisheyeNode, this));
            // rclcpp::Node::SharedPtr slamNode = node->create_sub_node("slam");
            // publisherList.push_back(std::make_shared<CRosSlamPublisher>(slamNode, this));
            rclcpp::Node::SharedPtr imuNode = node->create_sub_node("imu");
            publisherList.push_back(std::make_shared<CRosImuPublisher>(imuNode, this));
            // rclcpp::Node::SharedPtr objectdetectionNode = node->create_sub_node("objectdetection");
            // publisherList.push_back(std::make_shared<CRosObjectDetectionPublisher>(objectdetectionNode, this));
            rclcpp::Node::SharedPtr featuresNode = node->create_sub_node("features");
            publisherList.push_back(std::make_shared<CRosFeaturesPublisher>(featuresNode, this));
        }

        callback_handle_ = node->add_on_set_parameters_callback(std::bind(&CRosSensor::parametersCallback, this, std::placeholders::_1));

#if defined(USE_LIFECYCLE_MANAGEMENT)
        spin_prom = std::promise<int>();
        spin_th = std::thread([this](void) {rclcpp::spin_until_future_complete(node, spin_prom.get_future()); });
#endif

        // timer = node->create_wall_timer(std::chrono::milliseconds(4000), std::bind(&CRosSensor::timerCallback, this));

        // rclcpp::spin(node);
        return 0;
    }

    void CRosSensor::disconnect(void)
    {
        if (connectionStateTimer) {
            connectionStateTimer.reset();
            StopStream();
        }
    }

    void CRosSensor::stop_publish(void)
    {
        if (!callback_handle_) {
            RCLCPP_WARN_STREAM(logger, "CRosSensor is not publishing.");
            return;
        }
        #if defined(USE_LIFECYCLE_MANAGEMENT)
        //stop spinning
        spin_prom.set_value(1);
        spin_th.join();
        callback_handle_.reset();
        //timer.reset();
        // we must call StopStream for each publisher explicitly before destruct them
        for (auto const& it : publisherList)
        {
            it->StopStream();
        }
        #endif
        publisherList.clear();
    }

    std::shared_ptr<InuDev::CInuSensorExt> CRosSensor::getSensor()
    {
        return sensor;
    }

    std::string CRosSensor::getSerialNumber()
    {
        return version[InuDev::CEntityVersion::EEntitiesID::eSerialNumber].VersionName;
    }

    void CRosSensor::InsertWebImage(const cv::Mat& img, uint64_t ts)
    {
        unique_lock<mutex> lock(mMutexWeb);
        mMatWeb = img.clone();
        mTimestampWeb = ts;
    }

    uint64_t CRosSensor::GetWebImage(cv::Mat& img, uint64_t ts)
    {
        unique_lock<mutex> lock(mMutexWeb);
        if (mMatWeb.empty())
        {
            return 0;
        }
        uint64_t dif = myABS(ts, mTimestampWeb);
        if (dif > 1e9)
        {
            return 0;
        }
        img = mMatWeb.clone();
        return mTimestampWeb;
    }

    InuDev::CInuError CRosSensor::StartStream()
    {
        std::unique_lock<std::mutex>  uniqueLock(mMutexOpt);

        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        if (sensor)
        {
            // Already created (in constructor)
            return InuDev::EErrorCode::eOK;
        }

        InitializeParams();

        RCLCPP_INFO_STREAM(logger, " Starting sensor with IP: " << mIpAddress <<"  | with ID: " << mServiceID);

        if(mIpAddress !="")
        {
            sensor = InuDev::CInuSensorExt::Create(mServiceID,mIpAddress);
        }
        else if(mServiceID != "")
        {
            sensor = InuDev::CInuSensorExt::Create(mServiceID);
        }
        else
        {
            sensor = InuDev::CInuSensorExt::Create();
        }

        if (!sensor)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": sensor is null!!");
            return InuDev::EErrorCode::eInitError;
        }

        RCLCPP_WARN_STREAM(logger, "Starting sensor with FPS: " << mDeviceParams.FPS << " Resolution: " << mDeviceParams.SensorRes);
        InuDev::CInuError err = InuDev::EErrorCode::eOK;

        if (mDeviceParams.SensorRes == 0 && mDeviceParams.FPS == -1)
        {
            err = sensor->Init(mHwInfo, mVecDpeParams);
        }
        else
        {
            err = sensor->Init(mHwInfo, mVecDpeParams, mDeviceParams);
        }
        if (mDeviceParams.FPS == -1)
        {
            mDeviceParams.FPS = std::numeric_limits<uint32_t>::max();
        }

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": InuSensor failed to Init: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            sensor->Terminate();
            // sensor.reset();
            return err;
        }

        mChannels = mHwInfo.GetChannels();

        // InuDev::CInuSensor::CallbackFunction callback = std::bind(&CRosSensor::SensorCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        // err = sensor->Register(callback);

        // if (err != InuDev::EErrorCode::eOK)
        // {
        //     RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": InuSensor failed register Callback: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
        //     return err;
        // }

        std::map<uint32_t, InuDev::CChannelControlParams> mapChannelParams;
        std::map<uint32_t, InuDev::CChannelSize> channelsSize;
        uint32_t videoChannel;
        bool foundVideoChannel = false;

        for (auto channel : mChannels)
        {
            if (channel.second.ChannelType == InuDev::eStereoChannel)
            {
                videoChannel = channel.first;
                foundVideoChannel = true;
            }

            if (channel.second.ChannelType == InuDev::eGeneralCameraChannel || channel.second.ChannelType == InuDev::eDepthChannel)
            {
                InuDev::CChannelControlParams channelParams;

                if (channel.second.ChannelType == InuDev::eGeneralCameraChannel)
                {
                    channelParams = InuDev::CChannelControlParams(InuDev::ESensorResolution(mRGBResolution), mDeviceParams.FPS);
                }
                else
                {
                    channelParams = InuDev::CChannelControlParams(InuDev::ESensorResolution(mDepthResolution), mDeviceParams.FPS);
                  #ifdef _GE428_
                    int _D2Cchannel = dynamicParams.find("DepthRegistarionChannel")->second.as_int();
                    if(_D2Cchannel != -1){
                        channelParams.ActivateRegisteredDepth = true;
                        channelParams.RegisteredDepthChannelID = _D2Cchannel;
                    }
                  #endif
                }
                mapChannelParams.insert(std::make_pair(channel.first, channelParams));
            }
            else if (channel.second.ChannelType == InuDev::eTrackingChannel)
            {
                //mapChannelParams.insert(std::make_pair(channel.first, InuDev::CChannelControlParams(InuDev::eFull, mDeviceParams.FPS)));
                InuDev::CChannelControlParams channelParams;
                channelParams.InterleaveMode = InuDev::eInterleave;
                channelParams.SensorRes = InuDev::eFull;
                channelParams.FPS = mDeviceParams.FPS;

                mapChannelParams.insert(std::make_pair(channel.first, channelParams));
            }
        }

        if (foundVideoChannel)
        {
            InuDev::CChannelControlParams channelParams;
            channelParams.InterleaveMode = InuDev::eInterleave;
            channelParams.SensorRes = mDeviceParams.SensorRes;
            channelParams.FPS = mDeviceParams.FPS;

            mapChannelParams.insert(std::make_pair(videoChannel, channelParams));

            err = sensor->Start(channelsSize, mapChannelParams);

        }
        else
        {
            err = sensor->Start();
        }

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": InuSensor failed to Start: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            sensor->Terminate();
            // sensor.reset();
            return err;
        }

        err = sensor->GetVersion(version);

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Failed to get Sensor version: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            sensor->Terminate();
            // sensor.reset();
            return err;
        }

        auto it = version.find(InuDev::CEntityVersion::EEntitiesID::eSerialNumber);

        if (it == version.end())
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Failed to get Sensor's serial number: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            sensor->Terminate();
            // sensor.reset();
            return InuDev::EErrorCode::eInternalError;
        }

        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Serial number: " << it->second.VersionName);

        calibrationVersion = version.find(InuDev::CEntityVersion::EEntitiesID::eCalibrationVersion)->second.VersionName;
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": calibrationVersion number: " << calibrationVersion);

        auto calibrationData = std::make_shared<InuDev::CCalibrationData>();

        err = sensor->GetCalibrationData(*calibrationData.get());

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Failed to retreive calibration data: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            sensor->Terminate();
            // sensor.reset();
            return err;
        }

        mCalibrationData = calibrationData; // shared_ptr assignemt

        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Started " << getName());

        return InuDev::EErrorCode::eOK;
    }

    InuDev::CInuError CRosSensor::StopStream()
    {
        std::unique_lock<std::mutex>  uniqueLock(mMutexOpt);
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!sensor)
        {
            return InuDev::EErrorCode::eOK;
        }


        InuDev::CInuError err = sensor->Register(nullptr);

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Unregister: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
        }

        err = sensor->Stop();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Stop: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
        }

        err = sensor->Terminate();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Terminate: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
        }

        sensor = nullptr;

        return err;
    }

    void CRosSensor::MiddleUpdate()
    {
        // start/stop individual streams

        for (auto const& it : publisherList)
        {
            it->UpdatePublisher();
        }
    }

    CRosSensor::~CRosSensor()
    {

        publisherList.clear();

        StopStream();
    }

    InuDev::CInuError CRosSensor::SensorCallback(std::shared_ptr<InuDev::CInuSensor> iStream, InuDev::EConnectionState iConnectionState, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        if (iError == InuDev::EErrorCode::eOK)
        {
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": connection state = " << iConnectionState);
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": connection state = " << iConnectionState << ", error = 0x" << std::hex << std::setw(8) << std::setfill('0') << int(iError));
        }

        switch (int(iError))
        {
        case InuDev::EErrorCodeExt::eUSBDisconnect:

            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Target disconnected");
            return InuDev::EErrorCode::eOK;
            break;

        case InuDev::EErrorCodeExt::eUSBConnect:

            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Target re-connected");
            return InuDev::EErrorCode::eOK;
            break;
        }

        return InuDev::EErrorCode::eOK;
    }

    int CRosSensor::GetNumSubscribers()
    {
        int n = 0;
        for (auto const& it : publisherList)
        {
            n += it->GetNumSubscribers();
        }

        publisherPrev = publisherCurr;
        publisherCurr = n;

        return publisherCurr;
    }

    void CRosSensor::timerCallback()
    {

        GetNumSubscribers();

        UpdatePublisher();

        if (sensor && !mLastKnownTemperature.compare_exchange_strong(mCurrentTemperature, mCurrentTemperature))
        {
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": new sensor temperature: " << mLastKnownTemperature);
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Getting Calibration Data for " << getName());

            auto calibrationData = std::make_shared<InuDev::CCalibrationData>();
            InuDev::EErrorCode err = sensor->GetCalibrationData(*calibrationData.get());

            if (err == InuDev::EErrorCode::eOK)
            {
                mCalibrationData = calibrationData; // shared_ptr assigment
            }
            else
            {
                RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Failed to retreive calibration data: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            }
        }
    }

    std::vector<int> CRosSensor::GetSensors(uint32_t channel)
    {
        std::vector<int> sensors;

        sensors.clear();

        if (mChannels.find(channel) != mChannels.end())
        {

            auto hwc = mChannels[channel].ChannelSensorsID;

            for (auto& elem : hwc)
            {
                sensors.push_back(int(elem));
            }
        }

        return sensors;
    }

    std::shared_ptr<InuDev::CCalibrationData> CRosSensor::GetCalibrationData(int currTemp, int iChannelID)
    {
        mCurrentTemperature = currTemp;

        auto calibrationData = std::make_shared<InuDev::CCalibrationData>();
        sensor->GetCalibrationData(*calibrationData.get(), iChannelID, mCurrentTemperature);

        mCalibrationData = calibrationData; // shared_ptr assignemt

        return mCalibrationData;
    }

    void CRosSensor::InitializeParams()
    {
       rclcpp::Parameter param;
       getParam("ServiceID", "", param, false, "Device ID support for multiple sensors. empty is default. 1 or 2 can be used for multiple sensors");
       mServiceID = param.as_string();
       getParam("IP_Address" , "", param, false);
       mIpAddress = param.as_string();
       getParam("FPS", 15, param, true, "General FPS, Default: -1 (can be overriden for specific channels)");
       mDeviceParams.FPS = param.as_int();
       getParam("Resolution", 1, param, true, "0 - Default (Full), 1 - Binning, 2 - VerticalBInning, 3 - Full");
       mDeviceParams.SensorRes = (InuDev::ESensorResolution)param.as_int();
       getParam("RGBResolution", 1, param, true, "0 - Default (Full), 1 - Binning, 2 - VerticalBInning, 3 - Full");
       mRGBResolution = (InuDev::ESensorResolution)param.as_int();
       getParam("DepthResolution", 1, param, true, "0 - Default (Full), 1 - Binning, 2 - VerticalBInning, 3 - Full");
       mDepthResolution = (InuDev::ESensorResolution)param.as_int();

       // dynamic params which can be used from other classes
        getParam("RGBChannel", -1, param, true, "-1 - Default, usually 4 or 7");
        getParam("RGBOutputFormat", 3, param, true, "0 - eDefault (eBGRA, or eRGBA on Android), 1 - eRaw , 2 - eBGRA, 3- eRGBA, 4 - eBGR");
        getParam("RGBPostProcessing", 0, param, true, "0 - None, 1 - eRegistered, 2 - eUndistorted, 4 - eGammaCorrect");
        getParam("RGBRegistrationChannel", -1, param, true, "Depth channel to register, Default: -1 (verify that output format is registered)");
        getParam("TrackingCameraInterleaveMode", -1, param, true, "Tracking camera interleave mode: -1 - interleaved, 0 - right, 1 - left");
        getParam("DepthChannel", -1, param, true, "-1 - Default");
        getParam("DepthOutputFormat", -1, param, true, "-1 - Default (Depth), 0 - depth, 1 - disparity , 2 - pointcloud, 3- RGB, 4 - registered, 5- Raw");
        getParam("DepthRegistarionChannel", -1, param, true, "RGB channel to register, Default: -1 (verify that output format is registered)");
        getParam("Show_Color", false, param, true, "Show RGB image in depth output");
        getParam("Confidence", -1, param, true, "Depth confidence");
        getParam("PointCloudFormat", 0, param, true, "0 - Default(XYZ) ; 1 - XYZRGBa (with registration)");
        getParam("PointCloudRegistrationChannel", 4, param, true, "RGB channel to register, 0 - IR, 4 - RGB");
        getParam("ConvertPointCloudShortToFloat", true, param, true);
        getParam("ObjectDetectionType", "SSD", param, true, "SSD or YOLOV3");
        getParam("AutoExposure", true, param, true, "Auto Exposure flag On or Off");
        getParam("SensorID", 0, param, true, "Auto Exposure sensor ID");
        getParam("ExposureControl", 3500, param, true, "Exposure value (if AutoExposure is off)");
        getParam("Gain", 25, param, true, "Gain value (if AutoExposure is off)");
        getParam("UseROI", false, param, true, "Use ROI in Auto Exposure");
        getParam("ROITopLeftX", 0, param, true);
        getParam("ROITopLeftY", 0, param, true);
        getParam("ROIBottomRightX", 0, param, true);
        getParam("ROIBottomRightY", 0, param, true);
        getParam("PCLRGBChannelID", -1, param, true);
        getParam("PCLOutputFormat", 0, param, true);
        getParam("SLAMPathSize", -1, param, true);

        getParam("Simplify_2", true, param, true, "Simplify 2");

        // Temporal_Filter
        getParam("Temporal_Filter_ROS", false, param, true);
        getParam("Temporal_Filter_SDK", false, param, true);

        // Temporal2_Filter
        getParam("Temporal2_Filter_ROS", false, param, true);
        getParam("Temporal2_Filter_alpha", 40, param, true);
        getParam("Temporal2_Filter_delta", 20, param, true);

        // Edge_preserve_Filter
        getParam("Edge_Preserve_ROS", false, param, true);
        getParam("Edge_Preserve_alpha", 40, param, true);
        getParam("Edge_Preserve_delta", 20, param, true);

        // Hole_Fill_Filter
        getParam("Hole_Fill_Filter_ROS", false, param, true);
        getParam("Hole_Fill_Filter_SDK", false, param, true);
        // Passthrough_Filter
        getParam("Passthrough_Filter_ROS", false, param, true);
        getParam("Passthrough_Filter_SDK", false, param, true);

        // Outlier_Remove_Fitler
        getParam("Outlier_Remove_Fitler_ROS", false, param, true);
        getParam("Outlier_Remove_Fitler_SDK", false, param, true);

        getParam("Temporal_BufferFrames", 6, param, true);
        getParam("Temporal_ProcessAllPixel", false, param, true);
        getParam("Temporal_Threads", 1, param, true);

        // PT_zmin
        getParam("PT_zmin", 40, param, true);
        getParam("PT_zmax", 1000, param, true);
        getParam("PT_xmin", -1, param, true);
        getParam("PT_xmax", -1, param, true);
        getParam("PT_ymin", -1, param, true);
        getParam("PT_ymax", -1, param, true);

        // OR_percent
        getParam("OR_percent", 10, param, true);
        getParam("OR_min_dist", 35, param, true);
        getParam("OR_max_remove", 50, param, true);

        // Hole fill
        getParam("HF_maxradius", 25, param, true);

        // PCOrder
        getParam("PC_ordered", false, param, true);
        getParam("PC_scale", 1, param, true);

        // ativate depth 2 point cloud
        getParam("Dept2PcSupportted", false, param, false);
    }

    template<typename T> void CRosSensor::getParam(std::string iParamName, T iDefaultValue, rclcpp::Parameter& oValue, bool iIsDynamic, std::string iDescription)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = !iIsDynamic;
        descriptor.description = iDescription;
        if (dynamicParams.find(iParamName) == dynamicParams.end())
        {
            node->declare_parameter(iParamName, rclcpp::ParameterValue(iDefaultValue), descriptor);
            if (node->get_parameter(iParamName, oValue))
            {
                RCLCPP_INFO_STREAM(logger, " ====== : " << iParamName << " = " << oValue);
                dynamicParams.insert(std::make_pair(iParamName, oValue));
            }
            else
            {
                RCLCPP_WARN_STREAM(logger, "Failed to load param: " << iParamName);
            }
        }
        else
        {
            oValue = dynamicParams.find(iParamName)->second;
        }

    }

    rcl_interfaces::msg::SetParametersResult CRosSensor::parametersCallback(const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto& param : parameters)
        {
            RCLCPP_INFO_STREAM(logger, "Trying to find param name: " << param.get_name().c_str());

            if (dynamicParams.find(param.get_name().c_str()) != dynamicParams.end())
            {
                RCLCPP_INFO_STREAM(logger, "Recieved parameter name: " << param.get_name().c_str());
                RCLCPP_INFO_STREAM(logger, "Recieved parameter type: " << param.get_type_name().c_str());
                RCLCPP_INFO_STREAM(logger, "Recieved parameter value: " << param.value_to_string().c_str());
                dynamicParams.insert_or_assign(param.get_name(), param);
            }
            else
            {
                RCLCPP_INFO_STREAM(logger, "Can't find param with name: " << param.get_name().c_str());
            }
        }
        return result;
    }

    void CRosSensor::ReStart(bool fromThread)   //restart sensor
    {
        std::unique_lock<std::mutex> my_unique(mMutexOpt, std::try_to_lock);

        if(!my_unique.owns_lock())
        {
            return;
        }

        if (!sensor)
        {
            return;
        }
        my_unique.unlock();


        // Log restart sensor
        InuDev::CInuError err = StopStream();
        if (err == InuDev::EErrorCode::eOK)
        {
            RCLCPP_INFO_STREAM(logger, "Sensor stopped");
        }
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Log Start sensor
        InuDev::CInuError error = StartStream();
        if (error == InuDev::EErrorCode::eOK)
        {
            RCLCPP_INFO_STREAM(logger, "Sensor started");
        }

        if(StartTemperaStream())
        {
            RCLCPP_INFO_STREAM(logger, "Succeed to check tempera");
        }

        StopTemperaStream();


        for (auto const& it : publisherList)
        {
            it->UpdatePublisherRecover();
        }
    }

    void CRosSensor::connectionCheckCallback()
    {
        if (sensor)
        {
            InuDev::CInuSensor::ESensorState newState = sensor->GetState();
            RCLCPP_INFO_STREAM(logger, "InuSensor  state  ****** new state =  " << int(newState));

            if ( !mCheckTemperaSucceeded || int(newState) == 0 || int(newState) == 4 || int(newState) == 3)
            {
                ReStart(true);
            }
            state = newState;
        }
    }

    void CRosSensor::TemperaCallback(std::shared_ptr<InuDev::CTemperaturesStream> iStream, std::shared_ptr<const InuDev::CTemperaturesFrame>  iFrame, InuDev::CInuError retCode)
    {
        if (retCode != InuDev::eOK || iFrame->Valid == false)
        {
            return;
        }

        float tempera[3];//sensor1,sensor2,pvt;

        iFrame->GetTemperature((InuDev::CTemperaturesFrame::ESensorTemperatureType)1,tempera[0]);
        iFrame->GetTemperature((InuDev::CTemperaturesFrame::ESensorTemperatureType)2,tempera[1]);
        iFrame->GetTemperature(InuDev::CTemperaturesFrame::ESensorTemperatureType::ePVT,tempera[2]);

        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": sensor1_tempera  = " <<tempera[0]<< " sensor2_tempera  = " <<tempera[1]<<" pvt_tempera  = " <<tempera[2]);

        std::unique_lock<std::mutex> lk(mMutexTempera);

        mCheckTemperaSucceeded=true;

        mConditionVariableTempera.notify_one();
    }


    bool CRosSensor::StartTemperaStream()
    {
        mCheckTemperaSucceeded=false;

        mTemperaStream = sensor->CreateTemperaturesStream(InuDev::CTemperaturesFrame::ESensorTemperatureType::eAll);
        if (mTemperaStream == nullptr)
        {
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Unexpected error, failed to get Temperature Stream ");
            return false;
        }

        InuDev::EErrorCode retCode = std::static_pointer_cast<InuDev::CTemperaturesStream>(mTemperaStream)->Init();
        if (retCode != InuDev::eOK)
        {
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Temperature failed to Init: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(retCode));
            return false;
        }

        retCode = std::static_pointer_cast<InuDev::CTemperaturesStream>(mTemperaStream)->Start();
        if (retCode != InuDev::eOK)
        {
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Temperature failed to Start: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(retCode));
            return false;
        }

        // retCode = rgbStream->Register([this](auto stream, auto frame, auto error) { FrameCallback(stream, frame, error); });
        retCode = std::static_pointer_cast<InuDev::CTemperaturesStream>(mTemperaStream)->Register(std::bind(&CRosSensor::TemperaCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        if (retCode != InuDev::eOK)
        {
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Temperature failed to Register: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(retCode));
            return false;
        }

        std::unique_lock<std::mutex> lk(mMutexTempera);

        mConditionVariableTempera.wait_for(lk, std::chrono::seconds(2), [&]() {return mCheckTemperaSucceeded; });

        if (!mCheckTemperaSucceeded)
        {
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Temperature failed to check because of timeout." );
            return false;
        }

        return true;
    }

    bool CRosSensor::StopTemperaStream()
    {
        if (!mTemperaStream) { return true; }

        InuDev::CInuError retCode = std::static_pointer_cast<InuDev::CTemperaturesStream>(mTemperaStream)->Register(nullptr);
        if (retCode != InuDev::eOK)
        {
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Temperature failed to Unregister: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(retCode));
            return false;
        }

        retCode = mTemperaStream->Stop();
        if (retCode != InuDev::eOK)
        {
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Temperature failed to Stop: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(retCode));
            return false;
        }

        retCode = mTemperaStream->Terminate();
        if (retCode != InuDev::eOK)
        {
            RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Temperature failed to Terminate: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(retCode));
            return false;
        }

        mTemperaStream = nullptr;

        return true;
    }

}
