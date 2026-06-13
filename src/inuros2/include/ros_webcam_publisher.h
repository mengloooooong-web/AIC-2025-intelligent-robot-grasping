#pragma once

/*
 * File - ros_webcam_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "ImageStreamExt.h"
// #include "ImageRegisteredStream.h"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_webcam_publisher.h
 *
 * \brief ROS Webcam publisher
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS Webcam Publisher
     */
    class CRosWebcamPublisher : public CRosPublisher
    {
        image_transport::CameraPublisher Imagepublisher;

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr CIpublisher;

        void FrameCallback(std::shared_ptr<InuDev::CImageStream> iStream, std::shared_ptr<const InuDev::CImageFrame> iFrame, InuDev::CInuError iError);
        InuDev::CInuError RegisterCallback();
        InuDev::CInuError UnregisterCallback();

        /**
         * \brief fillCameraInfoFromWebcam
         *
         * Fill ROS CameraInfo for frames originating from WebCam sensor
         */
        void fillCameraInfoFromWebcam(unsigned int frameIndex, int width, int height);

    protected:
        virtual InuDev::CInuError InitStream() override;
        virtual InuDev::CInuError StartStream() override;

    public:

        virtual int GetNumSubscribers() override;

        /**
         * \brief CRosWebcamPublisher
         *
         * CRosWebcamPublisher constructor
         */
        CRosWebcamPublisher(rclcpp::Node::SharedPtr _node, CRosSensor* __rosSensor, std::string iTopicName = "camera/color/image_raw");

    private:

        /**
         * \brief ChannelID
         *
         * Channel ID as recieved from InuRosParams.xml : -1 is default
         */
        int mParamsChannelID;

        std::string mServiceId;

        /**
         * \brief OutputFormat
         *
         * Output format: 0 is default
         */
        int mOutputFormat;

        /**
        * \brief OutputFormat
        *
        * PostProcessing: 0 is default - none
        */
        int mPostProcessing;

        /**
         * \brief RegistrationChannel
         *
         * Registration channel: 3 is default
         */
        int mRegistrationChannel;

        /**
         * \brief Calibration Data
         */
        std::shared_ptr<InuDev::CCalibrationData> mCalibrationData;

    };
}
