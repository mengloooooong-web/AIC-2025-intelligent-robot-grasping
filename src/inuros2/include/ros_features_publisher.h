#pragma once

/*
 * File - ros_features_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_publisher.h"
#include "FeaturesTrackingStream.h"
//#include "inuros2_messages/msg/features.hpp"
//#include "msg/features.hpp"
#include "inuros2/msg/features.hpp"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_features_publisher.h
 *
 * \brief ROS features publisher
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS Features Publisher
     */
    class CRosFeaturesPublisher : public CRosPublisher
    {
        rclcpp::Publisher<inuros2::msg::Features>::SharedPtr publisher;

        void FrameCallback(std::shared_ptr<InuDev::CFeaturesTrackingStream> iStream, std::shared_ptr<const InuDev::CFeaturesTrackingFrame> iFrame, InuDev::CInuError iError);
        InuDev::CInuError RegisterCallback();
        InuDev::CInuError UnregisterCallback();

        /**
         * \brief fillCameraInfoFromWebcam
         *
         * Fill ROS CameraInfo for frames originating from WebCam sensor
         */
        void fillCameraInfoFromFisheye(unsigned int frameIndex, int width, int height);

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
        CRosFeaturesPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* __rosSensor, std::string iTopicName = "camera/fisheye/features");

    private:

        // Channel ID as recieved from InuRosParams.xml : -1 is default
        int mParamsChannelID;

        std::shared_ptr<InuDev::CCalibrationData> mCalibrationData;

    };
}
