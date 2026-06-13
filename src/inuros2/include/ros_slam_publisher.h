#pragma once

/*
 * File - ros_slam_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_publisher.h"

#include "SlamStream.h"

#include "config.h"
#include <nav_msgs/msg/path.hpp>

/**
 * \file ros_slam_publisher.h
 *
 * \brief Slam publisher
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS Slam publisher
     */
    class CRosSlamPublisher : public CRosPublisher
    {
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr SLAMpublisher;

        void FrameCallback(std::shared_ptr<InuDev::CSlamStream> iStream, std::shared_ptr<const InuDev::CSlamFrame> iFrame, InuDev::CInuError iError);
        InuDev::CInuError RegisterCallback();
        InuDev::CInuError UnregisterCallback();

        static std::vector<const char*> fieldNames;

    protected:

        virtual InuDev::CInuError InitStream() override;
        virtual InuDev::CInuError StartStream() override;

    public:

        virtual int GetNumSubscribers() override;

        /**
         * \brief CRosSlamPublisher
         *
         * CRosSlamPublisher constructor
         */
        CRosSlamPublisher(rclcpp::Node::SharedPtr _node, CRosSensor* _rosSensor, std::string iTopicName = "sensor_msgs/Path");

        /**
         * \brief path
         *
         * path
         */
        nav_msgs::msg::Path path;

    private:

        int mPathSize;

        //Position struct
        struct poseT
                {
            float mQuaternion[4];
            float mTranslation[3];
        };

        void Inverse_pos(poseT& iPose, poseT& oPose);

        void MultiplyQuaternionByVec(poseT& oPose, float v[3], float newTrans[3]);

    };
}
