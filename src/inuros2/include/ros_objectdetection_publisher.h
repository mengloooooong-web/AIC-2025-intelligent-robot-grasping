#pragma once

/*
 * File - ros_objectdetection_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2021 Inuitive All Rights Reserved
 *
 */

#include "ros_publisher.h"
#include <vision_msgs/msg/detection2_d_array.hpp>


//#include "../../vision_msgs/"
#include "CnnAppStream.h"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_objectdetection_publisher.h
 *
 * \brief Object detection publisher
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS Object detection publisher
     */
    class CRosObjectDetectionPublisher : public CRosPublisher
    {
        rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr publisher;

        void FrameCallback(std::shared_ptr<InuDev::CCnnAppStream> iStream, std::shared_ptr<const InuDev::CCnnAppFrame> iFrame, InuDev::CInuError iError);
        InuDev::CInuError RegisterCallback();
        InuDev::CInuError UnregisterCallback();

        static std::vector<const char*> fieldNames;

    protected:

        virtual InuDev::CInuError InitStream() override;
        virtual InuDev::CInuError StartStream() override;

    public:

        virtual int GetNumSubscribers() override;

        /**
         * \brief CRosObjectDetectionPublisher
         *
         * CRosObjectDetectionPublisher constructor
         */
        CRosObjectDetectionPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicName = "vision_msgs/Detections");

        /**
         * \brief mObjects
         *
         * TODO
         */
        vision_msgs::msg::Detection2DArray mObjects;
    };
}
