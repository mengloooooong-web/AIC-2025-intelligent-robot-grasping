#pragma once

/*
 * File - ros_imu_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_publisher.h"

#include "ImuStream.h"

#include "sensor_msgs/msg/imu.hpp"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_imu_publisher.h
 *
 * \brief IMU publisher
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS IMU publisher
     */
    class CRosImuPublisher : public CRosPublisher
    {
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr IMUpublisher;

        rclcpp::Node::SharedPtr node;

        void FrameCallback(std::shared_ptr<InuDev::CImuStream> iStream, std::shared_ptr<const InuDev::CImuFrame> iFrame, InuDev::CInuError iError);
        InuDev::CInuError RegisterCallback();
        InuDev::CInuError UnregisterCallback();

        InuDev::CPoint3D gyro;
        InuDev::CPoint3D accel;

    protected:
        virtual InuDev::CInuError InitStream() override;

        virtual InuDev::CInuError StartStream() override;

    public:

        /**
         * \brief CRosImuPublisher
         *
         * CRosImuPublisher constructor
         */
        CRosImuPublisher(rclcpp::Node::SharedPtr _node, CRosSensor* _rosSensor, std::string iTopicName = "sensor_msgs/Imu");

        virtual int GetNumSubscribers() override;
    };
}
