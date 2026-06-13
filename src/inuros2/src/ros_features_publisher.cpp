/*
 * File - ros_features_publisher.cpp
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
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
//#include "inudev_ros_nodelet/features_msg.h"

/**
 * \endcond
 */

#include "ros_features_publisher.h"
#include "inuros2_node_factory.h"

/**
 * \file ros_features_publisher.cpp
 *
 * \brief CRosFeaturesPublisher
 */
namespace __INUROS__NAMESPACE__
{
    CRosFeaturesPublisher::CRosFeaturesPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicName)
            : CRosPublisher(_node, "features", _rosSensor)
            //, publisher(node.advertise<features_msg>(iTopicName, 1, false))
    {
        const std::string serviceId = _rosSensor->GetSensorId();
        if (!serviceId.empty())
        {
            iTopicName = "service" + serviceId + "/" + iTopicName;
        }
        publisher = _node->create_publisher<inuros2::msg::Features>(iTopicName, 50);
        RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());
    }

    void CRosFeaturesPublisher::fillCameraInfoFromFisheye(unsigned int frameIndex, int width, int height)
    {
        mCi.header.frame_id  = getName();
        //mCi.header.seq       = frameIndex;
        mCi.width            = width;
        mCi.height           = height;

        mCi.distortion_model = "plumb_bob";

        if (!mSensors.empty())
        {
            auto c = mCalibrationData->Sensors[mSensors.at(0)].RealCamera.Intrinsic;

            // Intrinsic camera matrix for the raw (distorted) images.
            mCi.k[0] = mCi.p[0]  = c.FocalLength[0];
            mCi.k[1] = mCi.p[1]  = 0.0f;
            mCi.k[2] = mCi.p[2]  = c.OpticalCenter[0];
            mCi.k[3] = mCi.p[3]  = 0.0f;
            mCi.k[4]             = c.FocalLength[1];
                       mCi.p[4]  = 0;
                       mCi.p[5]  = c.FocalLength[1];
            mCi.k[5] = mCi.p[6]  = c.OpticalCenter[1];
            mCi.k[6] = mCi.p[7]  = 0.0f;
            mCi.k[7] = mCi.p[8]  = 0.0f;
            mCi.k[8]             = 1.0f;
                       mCi.p[9]  = 0.0f;
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

    void CRosFeaturesPublisher::FrameCallback(std::shared_ptr<InuDev::CFeaturesTrackingStream> iStream, std::shared_ptr<const InuDev::CFeaturesTrackingFrame> iFrame, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!CheckFrame(iFrame, iError))
        {
            return;
        }

        int width = iFrame->GetImageWidth();
        int height = iFrame->GetImageHeight();
        auto features = inuros2::msg::Features();

        for(int i = 0; i<iFrame->GetKeyPointNumber(); i++)
        {
            if (iFrame->GetProcessedData()[i].Descriptor)
            {
                for(auto elem : iFrame->GetProcessedData()->Descriptor)
                {
                    features.descriptor.push_back(elem);
                }
            }

            features.angle.push_back(iFrame->GetProcessedData()[i].Angle);
            features.confidence.push_back(iFrame->GetProcessedData()[i].Confidence);
            features.patternzise.push_back(iFrame->GetProcessedData()[i].PatternSize);
            features.uniqueid.push_back(iFrame->GetProcessedData()[i].UniqId);
            features.x.push_back(iFrame->GetProcessedData()[i].X);
            features.y.push_back(iFrame->GetProcessedData()[i].Y);
            features.isrightimage.push_back(iFrame->GetProcessedData()[i].IsRightImage);
            features.descriptorsize =  iFrame->GetDescriptorSize();
        }

        features.keypointnumber = iFrame->GetKeyPointNumber();
        fillCameraInfoFromFisheye(iFrame->FrameIndex, width, height);
        publisher->publish(features);
    }

    InuDev::CInuError CRosFeaturesPublisher::RegisterCallback()
    {
        RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CFeaturesTrackingStream::CallbackFunction callback = std::bind(&CRosFeaturesPublisher::FrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        return std::static_pointer_cast<InuDev::CFeaturesTrackingStream>(stream)->Register(callback);
    }

    InuDev::CInuError CRosFeaturesPublisher::UnregisterCallback()
    {
        return std::static_pointer_cast<InuDev::CFeaturesTrackingStream>(stream)->Register(nullptr);
    }

    int CRosFeaturesPublisher::GetNumSubscribers()
    {
        publisherPrev = publisherCurr;
        publisherCurr = publisher->get_subscription_count();

        return publisherCurr;
    }

    InuDev::CInuError CRosFeaturesPublisher::InitStream()
    {
        RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());
        InuDev::CInuError err = std::static_pointer_cast<InuDev::CFeaturesTrackingStream>(stream)->Init(InuDev::FeaturesTracking::EOutputType::eProcessed);

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        return InuDev::EErrorCode::eOK;
    }

    InuDev::CInuError CRosFeaturesPublisher::StartStream()
    {
        if (!sensor || !sensor->connected())
            return InuDev::EErrorCode::eStateError;

        stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateFeaturesTrackingStream());
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

        uint32_t channelID;
        err = stream->GetChannel(channelID);
        mChannelID = int(channelID);

        mSensors = sensor->GetSensors(mChannelID);
        mCalibrationData = sensor->GetCalibrationData(mCurrentTemprature, mChannelID);

        return InuDev::EErrorCode::eOK;
    }
}
