/*
 * File - ros_fisheye_publisher.cpp
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

//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/distortion_models.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

/**
 * \endcond
 */

#include "ros_fisheye_publisher.h"

#include "inuros2_node_factory.h"

/**
 * \file ros_fisheye_publisher.cpp
 *
 * \brief CRosFisheyePublisher
 */

namespace __INUROS__NAMESPACE__
{
    CRosFisheyePublisher::CRosFisheyePublisher(rclcpp::Node::SharedPtr _node, CRosSensor* _rosSensor, std::string iTopicName)
        : CRosPublisher(_node, "fisheye", _rosSensor)
    {
        const std::string serviceId = _rosSensor->GetSensorId();
        if (!serviceId.empty())
        {
            iTopicName = "service" + serviceId + "/" + iTopicName;
        }
        Imagepublisher = image_transport::create_camera_publisher(_node.get(), iTopicName);
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
        mCurrentSensorID = 0;
    }

    void CRosFisheyePublisher::fillCameraInfoFromFisheye(unsigned int frameIndex, int width, int height)
    {
        mCi.header.frame_id  = getName();
        mCi.header.frame_id  = "camera_link" + mServiceId;
        //mCi.header.seq       = frameIndex;
        mCi.width            = width;
        mCi.height           = height;
        mCi.distortion_model = "plumb_bob";

        if (!mSensors.empty())
        {
            auto c = mCalibrationData->Sensors[mSensors.at(mCurrentSensorID)].RealCamera.Intrinsic;

            if (mSensors.size() == 2)
            {
                if (mCurrentSensorID == 0)
                {
                    mCurrentSensorID = 1;
                }
                else
                {
                    mCurrentSensorID = 0;
                }
            }

            // Intrinsic camera matrix for the raw (distorted) images.
            mCi.k[0] = mCi.p[0] = c.FocalLength[0];
            mCi.k[1] = mCi.p[1] = 0.0f;
            mCi.k[2] = mCi.p[2] = c.OpticalCenter[0];
            mCi.k[3] = mCi.p[3] = 0.0f;
            mCi.k[4] = c.FocalLength[1];
            mCi.p[4] = 0;
            mCi.p[5] = c.FocalLength[1];
            mCi.k[5] = mCi.p[6] = c.OpticalCenter[1];
            mCi.k[6] = mCi.p[7] = 0.0f;
            mCi.k[7] = mCi.p[8] = 0.0f;
            mCi.k[8] = 1.0f;
            mCi.p[9] = 0.0f;
            mCi.p[10] = 1.0f;
            mCi.p[11] = 0.0f;
            mCi.r[0] = mCi.r[4] = mCi.r[8] = 1.0f;
            mCi.r[1] = mCi.r[2] = mCi.r[3] = mCi.r[5] = mCi.r[6] = mCi.r[7] = 0.0f;

            // distortion parameters
            mCi.d.resize(c.LensDistortion.size());

            for (std::vector<double>::size_type i=0; i<mCi.d.size(); i++)
            {
                mCi.d.at(i) = c.LensDistortion.at(i);
            }
        }
    }

    void CRosFisheyePublisher::FrameCallback(std::shared_ptr<InuDev::CImageStream> iStream, std::shared_ptr<const InuDev::CImageFrame> iFrame, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!CheckFrame(iFrame, iError))
        {
            return;
        }

        InuDev::EImageFormat format = (InuDev::EImageFormat)iFrame->Format();

        if (format != InuDev::EImageFormat::eBGRA)
        {
            //RCLCPP_ERROR_STREAM(__INUROS_FUNCTION_NAME__ << ": WebCam Frame's format (" << format << ") is not eBGRA");
            //return;
        }

        int width = iFrame->Width();
        int height = iFrame->Height();
        int bpp = iFrame->BytesPerPixel();
        const InuDev::byte* data = iFrame->GetData();

        cv::Mat frame(height, width, CV_8UC4, (char*)data);

        //bgra8: CV_8UC4, BGR color image with an alpha channel
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgba8", frame).toImageMsg();

        fillCameraInfoFromFisheye(iFrame->FrameIndex, width, height);

        Imagepublisher.publish(*msg, mCi);

    }

    InuDev::CInuError CRosFisheyePublisher::RegisterCallback()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CImageStream::CallbackFunction callback = std::bind(&CRosFisheyePublisher::FrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        return std::static_pointer_cast<InuDev::CImageStream>(stream)->Register(callback);
    }

    InuDev::CInuError CRosFisheyePublisher::UnregisterCallback()
    {
        return std::static_pointer_cast<InuDev::CImageStream>(stream)->Register(nullptr);
    }

    int CRosFisheyePublisher::GetNumSubscribers()
    {
        publisherPrev = publisherCurr;
        publisherCurr = Imagepublisher.getNumSubscribers();

        return publisherCurr;
    }

    InuDev::CInuError CRosFisheyePublisher::StartStream()
    {
        if (!sensor || !sensor->connected())
            return InuDev::EErrorCode::eStateError;

        stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateImageStream(2));

        RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": failed creating " << getName());

        InuDev::EErrorCode err = CRosPublisher::StartStream();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        uint32_t channelID;
        err = stream->GetChannel(channelID);

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        mChannelID = int(channelID);

        mSensors = sensor->GetSensors(mChannelID);
        mCalibrationData = sensor->GetCalibrationData(mCurrentTemprature, mChannelID);
        
        if(sensor->dynamicParams.find("ServiceID") != sensor->dynamicParams.end())
        {
           mServiceId = sensor->dynamicParams.find("ServiceID")->second.as_string();
        } 

        return InuDev::EErrorCode::eOK;
    }
}
