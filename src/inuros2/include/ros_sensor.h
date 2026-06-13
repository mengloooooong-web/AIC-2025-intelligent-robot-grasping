#pragma once

/*
 * File - ros_sensor.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include <atomic>
#include <mutex>
#include <memory>
#include <map>
#include <condition_variable>
#include "ros_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "TemperaturesStream.h"
#include <opencv2/opencv.hpp>
/**
 * \cond INTERNAL
 */

#include "config.h"
#include <future>
#include <thread>

/**
 * \endcond
 */

/**
 * \file ros_sensor.h
 *
 * \brief ROS sensor
 */
//#include <image_transport/image_transport.h>

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS Sensor
     *
     * Provides access to InuDev::CInuSensorExt and tracks inudev_ros_nodelet ROS publishers
     */

    class CRosSensor : public CRosPublisherBookkeeping
    {
        InuDev::CInuSensor::ESensorState state;

        std::shared_ptr<InuDev::CInuSensorExt> sensor;

        std::vector<std::shared_ptr<CRosPublisher>> publisherList;

        //std::vector<std::shared_ptr<rclcpp::Publisher>> publisherListNew;

        //std::shared_ptr<image_transport::ImageTransport> image_transport;

        image_transport::CameraPublisher pub_;

        InuDev::CInuError SensorCallback(std::shared_ptr<InuDev::CInuSensor>, InuDev::EConnectionState, InuDev::CInuError);

        std::map<InuDev::CEntityVersion::EEntitiesID, InuDev::CEntityVersion> version;

        // rclcpp::TimerBase::SharedPtr nodetimer;
        rclcpp::TimerBase::SharedPtr connectionStateTimer;

        // void connectionCheckCallback();

#if defined(USE_LIFECYCLE_MANAGEMENT)
        std::promise<int> spin_prom;
        std::thread spin_th;
#endif
        //rclcpp::Node &node;
        rclcpp::Node::SharedPtr node;

        rclcpp::Logger logger;

        std::map<uint32_t, InuDev::CHwChannel> mChannels;

        std::shared_ptr<InuDev::CCalibrationData> mCalibrationData;

        /**
         * \brief Last Known Temperature Atomic object
         */
        std::atomic<int> mLastKnownTemperature;

        /**
         * \brief Last temp received from sensor in a frame
         */
        int mCurrentTemperature;

        /**
         * \brief Device parameters
         */
        InuDev::CDeviceParams mDeviceParams;

        /**
         * \brief Vector DPE params
         */
        std::vector<InuDev::CDpeParams> mVecDpeParams;

        /**
         * \brief HW Information
         */
        InuDev::CHwInformation mHwInfo;

        /**
         * \brief HW Service ID - Used for multiple sensors
         */
        std::string mServiceID;

        /**
         * \brief HW IP Address
         */
        std::string mIpAddress;

        std::mutex mMutexWeb;
        cv::Mat mMatWeb;
        uint64_t mTimestampWeb;
        std::mutex mMutexOpt;

        InuDev::ESensorResolution mRGBResolution;

        InuDev::ESensorResolution mDepthResolution;
        bool mActivateDepthRegistration = 0;
        int mDepthRegistrationChannel = -1;

        bool restarting;

public:

        /**
        * \brief check temperature stream, if we can get it, that means service work well, if no, the MCU on sensor will reset the NU40000.
        */

        std::shared_ptr<InuDev::CTemperaturesStream> mTemperaStream;

        std::mutex mMutexTempera;
        std::condition_variable mConditionVariableTempera;
        bool mCheckTemperaSucceeded;

        void TemperaCallback(std::shared_ptr<InuDev::CTemperaturesStream>, std::shared_ptr<const InuDev::CTemperaturesFrame> , InuDev::CInuError);

        bool StartTemperaStream();

        bool StopTemperaStream();


    public:

        std::string calibrationVersion;

        inline const std::string& GetSensorId() { return mServiceID; }
        void InsertWebImage(const cv::Mat &img, uint64_t ts);
        uint64_t GetWebImage(cv::Mat &img, uint64_t ts);
        virtual void ReStart(bool fromThread);


        /**
         * \brief CRosSensor
         *
         * CRosSensor constructor
         */
        CRosSensor(rclcpp::Node::SharedPtr _node, int fps, const std::string _str);
        //CRosSensor(int fps, const std::string _str);


        virtual ~CRosSensor();

        /**
         * \brief getSensor
         *
         * \return InuDev sensor associated with the nodelet
         */
        std::shared_ptr<InuDev::CInuSensorExt> getSensor();

        /**
         * \brief getSerialNumber
         *
         * \return The sensor's serial number as string. Used to store ROS's version of the sensor's OpticalData (CameraInfo).
         */
        std::string getSerialNumber();

        /**
         * \brief Get CalibrationData
         *
         * \param[in] currTemp
         * \param[in] iChannelID
         *
         * \return true for success, false for fail
         */
        std::shared_ptr<InuDev::CCalibrationData> GetCalibrationData(int currTemp , int iChannelID = 0);

        /**
         * \brief Get list of sensors assoiated with a channel
         *
         * \param[in] channel
         *
         * \return list of channels
         */
        std::vector<int> GetSensors(uint32_t channel);


        virtual void MiddleUpdate();

        virtual InuDev::CInuError StartStream();
        virtual InuDev::CInuError StopStream();

        virtual int GetNumSubscribers() override;

        rclcpp::TimerBase::SharedPtr timer;

        void timerCallback();
        void connectionCheckCallback();

        std::map<std::string, rclcpp::Parameter> dynamicParams;

        int connect(void);
        int start_publish(void);
        void stop_publish(void);
        void disconnect(void);
        bool connected(void);

    private:

        void InitializeParams();
        InuDev::CInuError InitialzeSensor();

        template<typename T> void getParam(std::string iParamName, T iDefaultValue, rclcpp::Parameter& oValue ,bool iIsDynamic, std::string iDescription = "");

        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

        rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr callback_handle_;


    };
}
