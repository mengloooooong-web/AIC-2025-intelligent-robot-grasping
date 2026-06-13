#pragma once

/*
 * File - ros_fisheye_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_publisher.h"

#include "ImageStream.h"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_fisheye_publisher.h
 *
 * \brief ROS Fisheye publisher
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS FIsheye Publisher
     */
    class CRosFisheyePublisher : public CRosPublisher
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
        void fillCameraInfoFromFisheye(unsigned int frameIndex, int width, int height);

    protected:

        virtual InuDev::CInuError StartStream() override;

    public:

        virtual int GetNumSubscribers() override;

        /**
         * \brief CRosFisheyPublisher
         *
         * CRosFisheyePublisher constructor
         */
        CRosFisheyePublisher(rclcpp::Node::SharedPtr _node,  CRosSensor* __rosSensor, std::string iTopicName = "sensor_msgs/Image/Fisheye");

    private:

        std::shared_ptr<InuDev::CCalibrationData> mCalibrationData;

        int mCurrentSensorID;
        
        std::string mServiceId;

    };
}
