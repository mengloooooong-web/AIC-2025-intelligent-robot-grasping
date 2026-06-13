#pragma once

/*
 * File - ros_disparity_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_video_publishers_base.h"

#include "DepthStream.h"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_disparity_publisher.h
 *
 * \brief Disparity publisher
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS disparity publisher
     *
     * This publisher will publish the following topics:
     *
     * - /sensor_msgs/Image/Video/disparity
     */
    class CRosDisparityPublisher : public CRosVideoPublishersBase
    {
        image_transport::Publisher publisher;

        void FrameCallback(std::shared_ptr<InuDev::CDepthStream> iStream, std::shared_ptr<const InuDev::CImageFrame> iFrame, InuDev::CInuError iError);
        InuDev::CInuError RegisterCallback();
        InuDev::CInuError UnregisterCallback();

    protected:

        virtual InuDev::CInuError InitStream() override;
        virtual InuDev::CInuError StartStream() override;

    public:

        virtual int GetNumSubscribers() override;

        /**
         * \brief CRosDisparityPublisher
         * 
         * CRosDisparityPublisher constructor
         */
        CRosDisparityPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicName = "sensor_msgs/Image/Video/disparity");
    };

}

