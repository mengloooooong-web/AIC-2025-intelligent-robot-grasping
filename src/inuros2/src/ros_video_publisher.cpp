/*
 * File - ros_video_publisher.cpp
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
//#include <ros/rate.h>
//#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/msg/camera_info.hpp>
//#include <sensor_msgs/distortion_models.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * \endcond
 */

#include "ros_video_publisher.h"

#include "inuros2_node_factory.h"

/**
 * \file ros_video_publisher.cpp
 *
 * \brief CRosVideoPublisher
 *
 * This publisher will publish the following topics:
 *
 * - /sensor_msgs/Image/Video/left/image
 * - /sensor_msgs/Image/Video/right/image
 *
 */

namespace __INUROS__NAMESPACE__
{
    CRosVideoPublisher::CRosVideoPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicNameRight, std::string iTopicNameLeft)
        : CRosVideoPublishersBase(_node, "video", _rosSensor)
    {
        const std::string serviceId = _rosSensor->GetSensorId();
        if (!serviceId.empty())
        {
            iTopicNameRight = "service" + serviceId + "/" + iTopicNameRight;
            iTopicNameLeft = "service" + serviceId + "/" + iTopicNameLeft;
        }
        rightImagePublisher = image_transport::create_camera_publisher(_node.get(), iTopicNameRight);
        leftImagePublisher = image_transport::create_camera_publisher(_node.get(), iTopicNameLeft);

        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
    }

    void CRosVideoPublisher::FrameCallback(std::shared_ptr<InuDev::CStereoImageStream> iStream, std::shared_ptr<const InuDev::CStereoImageFrame> iFrame, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!CheckFrame(iFrame, iError))
        {
            return;
        }

        std::vector<const InuDev::CImageFrame *> frameVector;
        frameVector.push_back(iFrame->GetLeftFrame());
        frameVector.push_back(iFrame->GetRightFrame());

        for (std::vector<const InuDev::CImageFrame *>::size_type i = 0; i<frameVector.size(); i++)
        {
            const InuDev::CImageFrame* f = frameVector.at(i);

            if (!f)
            {
                RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Received empty image");
                return;
            }

            InuDev::EImageFormat format = (InuDev::EImageFormat)f->Format();

            if (format != InuDev::EImageFormat::eBGRA)
            {
                RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Video Frame's format (" << format << ") is not eBGRA");
                return;
            }

            int width = f->Width();
            int height = f->Height();

            const InuDev::byte *data = f->GetData();

            cv::Mat matFrame(height, width, CV_8UC4, (char *)data);

            std_msgs::msg::Header frameHeader;
            frameHeader.stamp = mCi.header.stamp = rclcpp::Time(iFrame->Timestamp / 1000000000ll, iFrame->Timestamp % 1000000000ll);
            frameHeader.frame_id = "camera_link";

            //bgra8: CV_8UC4, BGR color image with an alpha channel
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(frameHeader, "rgba8", matFrame).toImageMsg();


            bool bIsRightImage = (i == 1);
            fillCameraInfoFromVideo(iFrame->FrameIndex, width, height, bIsRightImage);

            // Supply publisher both image and cameraInfo
            if (i==0)
            {
                leftImagePublisher.publish(*msg, mCi);
            }
            else
            {
                rightImagePublisher.publish(*msg, mCi);
            }
            /*
            if (CInuDevRosNodeFactory::config.video_verbose)
            {
                RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Video frame: width=" << width << ", height=" << height << ", format=" << format);

                cv::imshow(CV_WINDOW_NAME, matFrame);

                cv::waitKey(1);
            }
            */
        }
    }

    InuDev::CInuError CRosVideoPublisher::RegisterCallback()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CStereoImageStream::CallbackFunction callback = std::bind(&CRosVideoPublisher::FrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        return std::static_pointer_cast<InuDev::CStereoImageStream>(stream)->Register(callback);
    }

    InuDev::CInuError CRosVideoPublisher::UnregisterCallback()
    {
        return std::static_pointer_cast<InuDev::CStereoImageStream>(stream)->Register(nullptr);
    }

    int CRosVideoPublisher::GetNumSubscribers()
    {
        publisherPrev = publisherCurr;
        publisherCurr = leftImagePublisher.getNumSubscribers() + rightImagePublisher.getNumSubscribers();

        return publisherCurr;
    }

    InuDev::CInuError CRosVideoPublisher::InitStream()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CInuError err = std::static_pointer_cast<InuDev::CStereoImageStream>(stream)->Init();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        uint32_t channelID;
        err = stream->GetChannel(channelID);

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " " << std::string(err) << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        mChannelID = int(channelID);
        mSensors = sensor->GetSensors(mChannelID);


        return InuDev::EErrorCode::eOK;
    }

    InuDev::CInuError CRosVideoPublisher::StartStream()
    {
        if (!sensor || !sensor->connected())
            return InuDev::EErrorCode::eStateError;

        stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateStereoImageStream());

        RCLCPP_ERROR_STREAM(logger,  __INUROS_FUNCTION_NAME__ << ": failed creating " << getName());

        InuDev::EErrorCode err = CRosPublisher::StartStream();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        return InuDev::EErrorCode::eOK;
    }

}
