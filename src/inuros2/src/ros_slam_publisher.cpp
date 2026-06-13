/*
 * File - ros_slam_publisher.cpp
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "config.h"

#include <fstream>
#include <iostream>
 //#include <nodelet/nodelet.h>
#include "rclcpp/rclcpp.hpp"
//#include <ros/rate.h>
//#include <ros/ros.h>
//#include <sensor_msgs/distortion_models.h>
#include <nav_msgs/msg/path.hpp>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include "ros_slam_publisher.h"

#include "inuros2_node_factory.h"

/**
 * \file ros_slam_publisher.cpp
 *
 * \brief CRosSlamPublisher
 */

namespace __INUROS__NAMESPACE__
{
    std::vector<const char*> CRosSlamPublisher::fieldNames = { "x", "y", "z" };

    CRosSlamPublisher::CRosSlamPublisher(rclcpp::Node::SharedPtr _node, CRosSensor* _rosSensor, std::string iTopicName)
        : CRosPublisher(_node, "nav_msgs", _rosSensor)
    {
        const std::string serviceId = _rosSensor->GetSensorId();
        if (!serviceId.empty())
        {
            iTopicName = "service" + serviceId + "/" + iTopicName;
        }
        SLAMpublisher = _node->create_publisher<nav_msgs::msg::Path>(iTopicName, 10);

        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
        path.header.stamp = _node->now();
        path.header.frame_id = "map";
        mPathSize = -1;
    }

    void CRosSlamPublisher::FrameCallback(std::shared_ptr<InuDev::CSlamStream> iStream, std::shared_ptr<const InuDev::CSlamFrame> iFrame, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
        if (!CheckFrame(iFrame, iError))
        {
            return;
        }

        std::array<float, 4> Q;
        std::array<float, 3> T;

        iFrame->ConvertPose4x4ToQuaternionTranslation(
            iFrame->mPose4x4BodyToWorld,
            Q,
            T
        );

        poseT iPose, oPose;
        iPose.mQuaternion[0] = Q[0];
        iPose.mQuaternion[1] = Q[1];
        iPose.mQuaternion[2] = Q[2];
        iPose.mQuaternion[3] = Q[3];
        iPose.mTranslation[0] = T[0];
        iPose.mTranslation[1] = T[1];
        iPose.mTranslation[2] = T[2];

        Inverse_pos(iPose, oPose);

        geometry_msgs::msg::PoseStamped poseStamped;

        poseStamped.pose.position.x = oPose.mTranslation[0];
        poseStamped.pose.position.y = oPose.mTranslation[1];
        poseStamped.pose.position.z = oPose.mTranslation[2];
        poseStamped.pose.orientation.x = oPose.mQuaternion[0];
        poseStamped.pose.orientation.y = oPose.mQuaternion[1];
        poseStamped.pose.orientation.z = oPose.mQuaternion[2];
        poseStamped.pose.orientation.w = oPose.mQuaternion[3];

        poseStamped.header.stamp = path.header.stamp;
        poseStamped.header.frame_id = "map";

        if (path.poses.size() == mPathSize)
        {
            path.poses.erase(path.poses.begin());
        }

        path.poses.push_back(poseStamped);
        SLAMpublisher->publish(path);

    }

    InuDev::CInuError CRosSlamPublisher::RegisterCallback()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CSlamStream::CallbackFunction callback = std::bind(&CRosSlamPublisher::FrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        return std::static_pointer_cast<InuDev::CSlamStream>(stream)->Register(callback);
    }

    InuDev::CInuError CRosSlamPublisher::UnregisterCallback()
    {
        return std::static_pointer_cast<InuDev::CSlamStream>(stream)->Register(nullptr);
    }

    int CRosSlamPublisher::GetNumSubscribers()
    {
        publisherPrev = publisherCurr;
        publisherCurr = SLAMpublisher->get_subscription_count();;

        return publisherCurr;
    }

    InuDev::CInuError CRosSlamPublisher::InitStream()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CInuError err = std::static_pointer_cast<InuDev::CSlamStream>(stream)->Init();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        return InuDev::EErrorCode::eOK;
    }

    InuDev::CInuError CRosSlamPublisher::StartStream()
    {
        if (!sensor || !sensor->connected())
            return InuDev::EErrorCode::eStateError;

        stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateSlamStream());
        RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": failed creating " << getName());

        InuDev::EErrorCode err = CRosPublisher::StartStream();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        if (sensor->dynamicParams.find("SLAMPathSize") != sensor->dynamicParams.end())
        {
            mPathSize = sensor->dynamicParams.find("SLAMPathSize")->second.as_int();
        }

        if (!path.poses.empty())
        {
            path.poses.clear();
        }

        return InuDev::EErrorCode::eOK;
    }

    void CRosSlamPublisher::Inverse_pos(poseT& iPose, poseT& oPose)
    {
        int k;
        for (k = 0; k < 3; k++)
        {
            oPose.mQuaternion[k] = -iPose.mQuaternion[k];
            oPose.mTranslation[k] = -iPose.mTranslation[k];
        }
        oPose.mQuaternion[3] = iPose.mQuaternion[3];

        float newTrans[3];
        MultiplyQuaternionByVec(oPose, oPose.mTranslation, newTrans);
        oPose.mTranslation[0] = newTrans[0];
        oPose.mTranslation[1] = newTrans[1];
        oPose.mTranslation[2] = newTrans[2];

    }
    void CRosSlamPublisher::MultiplyQuaternionByVec(poseT& oPose, float v[3], float newTrans[3])
    {
        float quaternion[4];
        quaternion[0] = oPose.mQuaternion[0];
        quaternion[1] = oPose.mQuaternion[1];
        quaternion[2] = oPose.mQuaternion[2];
        quaternion[3] = oPose.mQuaternion[3];

        float uv[3];
        uv[0] = 2 * (quaternion[1] * v[2] - quaternion[2] * v[1]);
        uv[1] = 2 * (quaternion[2] * v[0] - quaternion[0] * v[2]);
        uv[2] = 2 * (quaternion[0] * v[1] - quaternion[1] * v[0]);
        newTrans[0] = v[0] + quaternion[3] * uv[0] + (quaternion[1] * uv[2] - quaternion[2] * uv[1]);
        newTrans[1] = v[1] + quaternion[3] * uv[1] + (quaternion[2] * uv[0] - quaternion[0] * uv[2]);
        newTrans[2] = v[2] + quaternion[3] * uv[2] + (quaternion[0] * uv[1] - quaternion[1] * uv[0]);
    }
}
