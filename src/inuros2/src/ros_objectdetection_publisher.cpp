/*
 * File - ros_objectdetection_publisher.cpp
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2021 Inuitive All Rights Reserved
 *
 */

/**
 * \cond INTERNAL
 */

#include "config.h"

#include <iostream>
//#include <nodelet/nodelet.h>
//#include <ros/rate.h>
//#include <ros/ros.h>
//#include <sensor_msgs/distortion_models.h>
//#include <vision_msgs/Detection2DArray.h>
//#include <vision_msgs/detection2_d_array.h>

#include <opencv2/opencv.hpp>

/**
 * \endcond
 */

#include "ros_objectdetection_publisher.h"

#include "inuros2_node_factory.h"

/**
 * \file ros_objectdetection_publisher.cpp
 *
 * \brief CRosObjectDetectionPublisher
 */

namespace __INUROS__NAMESPACE__
{
    CRosObjectDetectionPublisher::CRosObjectDetectionPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicName)
            : CRosPublisher(_node, "vision_msgs", _rosSensor)
            //, publisher(node.advertise<vision_msgs::Detection2DArray>(iTopicName, 1, false))
    {
        const std::string serviceId = _rosSensor->GetSensorId();
        if (!serviceId.empty())
        {
            iTopicName = "service" + serviceId + "/" + iTopicName;
        }
        publisher = _node->create_publisher<vision_msgs::msg::Detection2DArray>(iTopicName, 1);

        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
        //mObjects.header.stamp = _node->now();
    }

    void CRosObjectDetectionPublisher::FrameCallback(std::shared_ptr<InuDev::CCnnAppStream> iStream, std::shared_ptr<const InuDev::CCnnAppFrame> iFrame, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        if (iError != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": frame " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(iError));
            return;
        }

        std::shared_ptr<std::vector<InuDev::CCnnAppFrame::CDetectedObject>> tmpObjects = iFrame->GetObjectData();

        if (!mObjects.detections.empty())
        {
            mObjects.detections.clear();
        }


        mObjects.header.stamp.sec = static_cast<int32_t>(iFrame->Timestamp / 1e9);
        mObjects.header.stamp.nanosec = iFrame->Timestamp % (uint64_t)1e9;
        mObjects.header.frame_id = std::to_string(iFrame->FrameIndex);
        //RCLCPP_ERROR_STREAM(logger ,__INUROS_FUNCTION_NAME__ << ": frame id: "<< std::to_string(iFrame->FrameIndex)<<" timestamp: " << std::to_string(iFrame->Timestamp) <<" sec: " <<std::to_string(mObjects.header.stamp.sec)<<" nano: "<<std::to_string(mObjects.header.stamp.nanosec));

        if (tmpObjects)
        {
            for (std::vector<InuDev::CCnnAppFrame::CDetectedObject>::iterator it = tmpObjects->begin(); it != tmpObjects->end(); ++it)
            {
                vision_msgs::msg::Detection2D newObject;
                std::string classId = it->ClassID;

                if (!classId.empty() && classId.back() == '\r') {
                    classId.erase(classId.size() - 1);
                }
                newObject.header.frame_id = classId;
#ifndef HIGH_ROS
                newObject.bbox.center.position.x = it->ClosedRectTopLeft.X() + (it->ClosedRectSize.X() / 2);
                newObject.bbox.center.position.y = it->ClosedRectTopLeft.Y() + (it->ClosedRectSize.Y() / 2);
#else
                newObject.bbox.center.x = it->ClosedRectTopLeft.X() + (it->ClosedRectSize.X() / 2);
                newObject.bbox.center.y = it->ClosedRectTopLeft.Y() + (it->ClosedRectSize.Y() / 2);
#endif
                newObject.bbox.size_x = it->ClosedRectSize.X();
                newObject.bbox.size_y = it->ClosedRectSize.Y();
                mObjects.detections.push_back(newObject);
            }
        }

        publisher->publish(mObjects);
    }

    InuDev::CInuError CRosObjectDetectionPublisher::RegisterCallback()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CCnnAppStream::CallbackFunction callback = std::bind(&CRosObjectDetectionPublisher::FrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        return std::static_pointer_cast<InuDev::CCnnAppStream>(stream)->Register(callback);
    }

    InuDev::CInuError CRosObjectDetectionPublisher::UnregisterCallback()
    {
        return std::static_pointer_cast<InuDev::CCnnAppStream>(stream)->Register(nullptr);
    }

    int CRosObjectDetectionPublisher::GetNumSubscribers()
    {
        publisherPrev = publisherCurr;
        publisherCurr = publisher->get_subscription_count();

        return publisherCurr;
    }

    InuDev::CInuError CRosObjectDetectionPublisher::InitStream()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CCnnAppFrame::EOutputType ObjectDetectionType = InuDev::CCnnAppFrame::eObjectDetection;
        std::string ObjectDetectionTypetmp;
        if (sensor->dynamicParams.find("ObjectDetectionType") != sensor->dynamicParams.end())
        {
            ObjectDetectionTypetmp = sensor->dynamicParams.find("ObjectDetectionType")->second.as_string();
        }
        if (ObjectDetectionTypetmp == "YOLOV3")
        {
            ObjectDetectionType = InuDev::CCnnAppFrame::eObjectDetectionYoloV3;
        }

        if (ObjectDetectionTypetmp == "YOLOV7")
        {
            ObjectDetectionType = InuDev::CCnnAppFrame::eObjectDetectionYoloV7;
        }

        InuDev::CInuError err = std::static_pointer_cast<InuDev::CCnnAppStream>(stream)->Init(ObjectDetectionType);

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        return InuDev::EErrorCode::eOK;
    }

    InuDev::CInuError CRosObjectDetectionPublisher::StartStream()
    {
        if (!sensor || !sensor->connected())
            return InuDev::EErrorCode::eStateError;

        stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateCnnAppStream());
        if (!stream)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": failed creating " << getName());
        }

        InuDev::EErrorCode err = InitStream();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        err = std::static_pointer_cast<InuDev::CCnnAppStream>(stream)->Start();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        if (!mObjects.detections.empty())
        {
            mObjects.detections.clear();
        }

        err = RegisterCallback(); // generic!

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Register callback: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        return InuDev::EErrorCode::eOK;
    }

}
