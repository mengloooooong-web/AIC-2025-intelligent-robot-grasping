#pragma once

/*
 * File - ros_video_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_video_publishers_base.h"

#include "StereoImageStream.h"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_video_publisher.h
 *
 * \brief ROS video publisher
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS Video Publisher
     */
    class CRosVideoPublisher: public CRosVideoPublishersBase
    {
        image_transport::CameraPublisher rightImagePublisher;
        image_transport::CameraPublisher leftImagePublisher;


        void FrameCallback(std::shared_ptr<InuDev::CStereoImageStream> iStream, std::shared_ptr<const InuDev::CStereoImageFrame> iFrame, InuDev::CInuError iError);
        InuDev::CInuError RegisterCallback();
        InuDev::CInuError UnregisterCallback();

    protected:

        virtual InuDev::CInuError InitStream() override;
        virtual InuDev::CInuError StartStream() override;

    public:

        virtual int GetNumSubscribers() override;

        /**
         * \brief CRosVideoPublisher
         *
         * CRosVideoPublisher constructor
         */
        CRosVideoPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicNameRight = "sensor_msgs/Image/Video/right/image", std::string iTopicNameLeft = "sensor_msgs/Image/Video/left/image");

    };

}
