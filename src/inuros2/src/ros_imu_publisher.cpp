/*
 * File - ros_imu_publisher.cpp
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

/**
 * \cond INTERNAL
 */

#include "config.h"
//#include <nodelet/nodelet.h>
//#include <ros/rate.h>
//#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>

/**
 * \endcond
 */

#include "ros_imu_publisher.h"
#include "inuros2_node_factory.h"

/**
 * \file src/ros_imu_publisher.cpp
 *
 * \brief CRosImuPublisher
 */

namespace __INUROS__NAMESPACE__
{
    CRosImuPublisher::CRosImuPublisher(rclcpp::Node::SharedPtr _node, CRosSensor* _rosSensor, std::string iTopicName)
        : CRosPublisher(_node, "imu", _rosSensor)
        , node(_node)
    {
        const std::string serviceId = _rosSensor->GetSensorId();
        if (!serviceId.empty())
        {
            iTopicName = "service" + serviceId + "/" + iTopicName;
        }
        IMUpublisher = _node->create_publisher<sensor_msgs::msg::Imu>(iTopicName, 50);
        RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());
    }

    void CRosImuPublisher::FrameCallback(std::shared_ptr<InuDev::CImuStream> iStream, std::shared_ptr<const InuDev::CImuFrame> iFrame, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!CheckFrame(iFrame, iError))
        {
            return;
        }

        auto imu_gyro_search = iFrame->SensorsData.find(InuDev::EImuType::eGyroscope);
        auto imu_accel_search = iFrame->SensorsData.find(InuDev::EImuType::eAccelerometer);

        // was should have both accel and gyro - ROS needs them both in the message.

        if (imu_gyro_search != iFrame->SensorsData.end())
        {
            gyro = imu_gyro_search->second;
        }

        if (imu_accel_search != iFrame->SensorsData.end())
        {
            accel = imu_accel_search->second;
        }

        // http://www.ros.org/reps/rep-0145.html
        // rqt_plot: add '/sensor_msgs/Imu/angular_velocity/{x,y,z}

        sensor_msgs::msg::Imu imu_msg;

        // imu_msg.header.stamp = node->now();
        imu_msg.header.stamp = rclcpp::Time(iFrame->Timestamp / 1000000000ll, iFrame->Timestamp % 1000000000ll);
        imu_msg.header.frame_id = "map";
        //imu_msg.header.seq = iFrame->FrameIndex;

        imu_msg.orientation_covariance[0] = -1;
        imu_msg.angular_velocity_covariance[0] = -1;
        imu_msg.linear_acceleration_covariance[0] = -1;


        imu_msg.angular_velocity.x = gyro.X();
        imu_msg.angular_velocity.y = gyro.Y();
        imu_msg.angular_velocity.z = gyro.Z();


        imu_msg.linear_acceleration.x = accel.X();
        imu_msg.linear_acceleration.y = accel.Y();
        imu_msg.linear_acceleration.z = accel.Z();

        IMUpublisher->publish(imu_msg);
    }

    InuDev::CInuError CRosImuPublisher::RegisterCallback()
    {
        RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CImuStream::CallbackFunction callback = std::bind(&CRosImuPublisher::FrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        return std::static_pointer_cast<InuDev::CImuStream>(stream)->Register(callback);
    }

    InuDev::CInuError CRosImuPublisher::UnregisterCallback()
    {
        return std::static_pointer_cast<InuDev::CImuStream>(stream)->Register(nullptr);
    }

    int CRosImuPublisher::GetNumSubscribers()
    {
        publisherPrev = publisherCurr;
        publisherCurr = IMUpublisher->get_subscription_count();

        return publisherCurr;
    }

    InuDev::CInuError CRosImuPublisher::InitStream()
    {
        InuDev::CInuError err = std::static_pointer_cast<InuDev::CImuStream>(stream)->Init();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        return InuDev::EErrorCode::eOK;
    }


    InuDev::CInuError CRosImuPublisher::StartStream()
    {
        if (!sensor || !sensor->connected())
            return InuDev::EErrorCode::eStateError;

        stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateImuStream());
        if(!stream)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": failed creating " << getName());
        }

        InuDev::EErrorCode err = CRosPublisher::StartStream();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        return InuDev::EErrorCode::eOK;
    }

}
