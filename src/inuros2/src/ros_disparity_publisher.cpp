/*
 * File - ros_disparity_publisher.cpp
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

//#include <sensor_msgs/image_encodings.h>
//#include <stereo_msgs/DisparityImage.h>
#include "stereo_msgs/msg/disparity_image.hpp"
//#include <sensor_msgs/distortion_models.h>

#include <opencv2/opencv.hpp>

/**
 * \endcond
 */

#include "ros_disparity_publisher.h"

#include "inuros2_node_factory.h"

/**
 * \file ros_disparity_publisher.cpp
 *
 * \brief CRosDisparityPublisher
 *
 * This publisher will publish the following topics:
 *
 * - /stereo_msgs/DisparityImage
 *
 */

namespace __INUROS__NAMESPACE__
{
    CRosDisparityPublisher::CRosDisparityPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicName)
        : CRosVideoPublishersBase(_node, "disparity", _rosSensor)
    {
        const std::string serviceId = _rosSensor->GetSensorId();
        if (!serviceId.empty())
        {
            iTopicName = "service" + serviceId + "/" + iTopicName;
        }
        publisher = image_transport::create_publisher(_node.get(), iTopicName);

        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
    }

    void CRosDisparityPublisher::FrameCallback(std::shared_ptr<InuDev::CDepthStream> iStream, std::shared_ptr<const InuDev::CImageFrame> iFrame, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!CheckFrame(iFrame, iError))
        {
            return;
        }

        InuDev::EImageFormat format = (InuDev::EImageFormat)iFrame->Format();

        if (format != InuDev::EImageFormat::eDisparity)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": Disparity Frame's format (" << format << ") is not eDisparity");
            return;
        }

        int width = iFrame->Width();
        int height = iFrame->Height();
        InuDev::byte *data = const_cast<InuDev::byte *>(iFrame->GetData());

        stereo_msgs::DisparityImagePtr disp_msg(new stereo_msgs::DisparityImage);

        disp_msg->min_disparity         = 0.0f; // iFrame->MinDisparity ???
        disp_msg->max_disparity         = 4096.0f; // iFrame->MaxDisparity ???

        disp_msg->valid_window.x_offset = 0;
        disp_msg->valid_window.y_offset = 0;
        disp_msg->valid_window.width    = 0;
        disp_msg->valid_window.height   = 0;

        std::shared_ptr<InuDev::CCalibrationData> cd(sensor->GetCalibrationData(mCurrentTemprature));

        disp_msg->T                     = cd->Baseline(mSensors.at(0), mSensors.at(1));
        disp_msg->f                     = cd->Sensors[mSensors.at(0)].VirtualCamera.Intrinsic.FocalLength[0];
        disp_msg->delta_d               = 0.0f;

        disp_msg->header.stamp.sec      = iFrame->Timestamp / 1000000000ll;
        disp_msg->header.stamp.nsec     = iFrame->Timestamp % 1000000000ll;
        disp_msg->header.frame_id       = ros2::this_node::getName();
        disp_msg->header.seq            = iFrame->FrameIndex;

        sensor_msgs::Image& dimage      = disp_msg->image;

        dimage.width                    = width;
        dimage.height                   = height;
        dimage.encoding                 = sensor_msgs::image_encodings::TYPE_32FC1;
        sensor_msgs::msg::
        dimage.step                     = dimage.width * sizeof(float);

        dimage.data.resize(dimage.step * dimage.height);


        // OpenCV cv::Mat::type()
        // +--------+----+----+----+----+------+------+------+------+
        // |        | C1 | C2 | C3 | C4 | C(5) | C(6) | C(7) | C(8) |
        // +--------+----+----+----+----+------+------+------+------+
        // | CV_8U  |  0 |  8 | 16 | 24 |   32 |   40 |   48 |   56 |
        // | CV_8S  |  1 |  9 | 17 | 25 |   33 |   41 |   49 |   57 |
        // | CV_16U |  2 | 10 | 18 | 26 |   34 |   42 |   50 |   58 |
        // | CV_16S |  3 | 11 | 19 | 27 |   35 |   43 |   51 |   59 |
        // | CV_32S |  4 | 12 | 20 | 28 |   36 |   44 |   52 |   60 |
        // | CV_32F |  5 | 13 | 21 | 29 |   37 |   45 |   53 |   61 |
        // | CV_64F |  6 | 14 | 22 | 30 |   38 |   46 |   54 |   62 |
        // +--------+----+----+----+----+------+------+------+------+

        cv::Mat_<float> dmat(height, width, reinterpret_cast<float*>(&dimage.data[0]), dimage.step);
        cv::Mat_<uint16_t> cv_image(height, width, reinterpret_cast<uint16_t*>(&data[0]), width*sizeof(uint16_t));

        // here type is 5, which is CV_32FC1
        cv_image.convertTo(dmat, CV_32F);

        fillCameraInfoFromVideo(iFrame->FrameIndex, width, height, true);

        publisher.publish(disp_msg);

    }

    InuDev::CInuError CRosDisparityPublisher::RegisterCallback()
    {
        RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CDepthStream::CallbackFunction callback = std::bind(&CRosDisparityPublisher::FrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        return std::static_pointer_cast<InuDev::CDepthStream>(stream)->Register(callback);
    }

    InuDev::CInuError CRosDisparityPublisher::UnregisterCallback()
    {
        return std::static_pointer_cast<InuDev::CDepthStream>(stream)->Register(nullptr);
    }

    int CRosDisparityPublisher::GetNumSubscribers()
    {
        publisherPrev = publisherCurr;
        publisherCurr = publisher.getNumSubscribers();

        return publisherCurr;
    }

    InuDev::CInuError CRosDisparityPublisher::InitStream()
    {
        RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());
        InuDev::CInuError err = std::static_pointer_cast<InuDev::CDepthStream>(stream)->Init(InuDev::CDepthStream::EOutputFormat::eRaw);

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        uint32_t channelID;
        err = stream->GetChannel(channelID);

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << " " << std::string(err) << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        mChannelID = int(channelID);

        mSensors = sensor->GetSensors(mChannelID);

        return InuDev::EErrorCode::eOK;
    }

    InuDev::CInuError CRosDisparityPublisher::StartStream()
    {
        if (!sensor || !sensor->connected())
            return InuDev::EErrorCode::eStateError;

        stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateDepthStream(5));

        if(!stream)
        {
        RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": failed creating " << getName());
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
