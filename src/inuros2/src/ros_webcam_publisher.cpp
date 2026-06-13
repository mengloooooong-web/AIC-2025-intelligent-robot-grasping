/*
 * File - ros_webcam_publisher.cpp
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

/**
 * \endcond
 */

#include "ros_webcam_publisher.h"
#include "inuros2_node_factory.h"

/**
 * \file ros_webcam_publisher.cpp
 *
 * \brief CRosWebcamPublisher
 */

namespace __INUROS__NAMESPACE__
{
    CRosWebcamPublisher::CRosWebcamPublisher(rclcpp::Node::SharedPtr _node, CRosSensor* _rosSensor, std::string iTopicName)
        : CRosPublisher(_node, "webcam", _rosSensor)
    {
        const std::string serviceId = _rosSensor->GetSensorId();
        if (!serviceId.empty())
        {
            iTopicName = "service" + serviceId + "/" + iTopicName;
        }
        Imagepublisher = image_transport::create_camera_publisher(_node.get(), iTopicName);
        //CIpublisher = _node->create_publisher<sensor_msgs::msg::CameraInfo>("webcamCI", 10);
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
    }

    void CRosWebcamPublisher::fillCameraInfoFromWebcam(unsigned int frameIndex, int width, int height)
    {
        mCi.header.frame_id = "camera_link" + mServiceId;
        //mCi.header.seq       = frameIndex;
        mCi.width           = width;
        mCi.height          = height;
        mCi.distortion_model = "plumb_bob";

        if (!mSensors.empty())
        {
            auto c = mCalibrationData->Sensors[mSensors.at(0)].RealCamera.Intrinsic;

            // Intrinsic camera matrix for the raw (distorted) images.
            mCi.k[0] = mCi.p[0] = c.FocalLength[0];
            mCi.k[1] = mCi.p[1] = 0.0f;
            mCi.k[2] = mCi.p[2] = c.OpticalCenter[0];
            mCi.k[3] = mCi.p[3] = 0.0f;
            mCi.k[4]            = c.FocalLength[1];
                       mCi.p[4] = 0;
                       mCi.p[5] = c.FocalLength[1];
            mCi.k[5] = mCi.p[6] = c.OpticalCenter[1];
            mCi.k[6] = mCi.p[7] = 0.0f;
            mCi.k[7] = mCi.p[8] = 0.0f;
            mCi.k[8]            = 1.0f;
                       mCi.p[9] = 0.0f;
                       mCi.p[10] = 1.0f;
                       mCi.p[11] = 0.0f;

            mCi.r[0] = mCi.r[4] = mCi.r[8] = 1.0f;
            mCi.r[1] = mCi.r[2] = mCi.r[3] = mCi.r[5] = mCi.r[6] = mCi.r[7] = 0.0f;

            mCi.d.resize(c.LensDistortion.size());

            for (std::vector<double>::size_type i = 0; i < mCi.d.size(); i++)
            {
                mCi.d.at(i) = c.LensDistortion.at(i);
            }
        }
    }

    void CRosWebcamPublisher::FrameCallback(std::shared_ptr<InuDev::CImageStream> iStream, std::shared_ptr<const InuDev::CImageFrame> iFrame, InuDev::CInuError iError)
    {
        //RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!CheckFrame(iFrame, iError))
        {
            return;
        }

        InuDev::EImageFormat format = (InuDev::EImageFormat)iFrame->Format();

        int width = iFrame->Width();
        int height = iFrame->Height();
        int bpp = iFrame->BytesPerPixel();
        const InuDev::byte* data = iFrame->GetData();

        // cv::Mat frame(height, width, CV_8UC3, (char *)data); // Hack due to openCV runtime error on cv::mat frame
        cv::Mat frame;



        // if (format == InuDev::EImageFormat::eBGR)
        // {
        //     cv::Mat registeredFrame(height, width, CV_8UC3, (char *) data);
        //     frame = registeredFrame;
        // }
        // else
        // {
        //      cv::Mat defaultFrame(height, width, CV_8UC4, (char *)data);
        //      frame = defaultFrame;
        // }
        switch (format)
        {
            case InuDev::EImageFormat::eBGR:
                frame = cv::Mat(height, width, CV_8UC3, (char *)data);
                cv::cvtColor(frame, frame, CV_BGR2RGB);
                break;
            case InuDev::EImageFormat::eBGRA:
                frame = cv::Mat(height, width, CV_8UC4, (char *)data);
                cv::cvtColor(frame, frame, CV_BGRA2RGB);
                break;
            case InuDev::EImageFormat::eRGBA:
                frame = cv::Mat(height, width, CV_8UC4, (char *)data);
                cv::cvtColor(frame, frame, CV_RGBA2RGB);
                break;
            default:
                RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": unrecognised WebCam frame foramt: " << format );
                return;
        }

        std_msgs::msg::Header frameHeader;
        //frameHeader.seq = iFrame->FrameIndex;
        frameHeader.stamp = mCi.header.stamp = rclcpp::Time(iFrame->Timestamp / 1000000000ll, iFrame->Timestamp % 1000000000ll);
	//frameHeader.stamp = mCi.header.stamp = rclcpp::Clock().now();
        frameHeader.frame_id = "camera_link" + mServiceId;
        //bgra8: CV_8UC4, BGR color image with an alpha channel
        sensor_msgs::msg::Image::SharedPtr msg;
        sensor_msgs::msg::CameraInfo cameraInfo;

        // if (format == InuDev::EImageFormat::eBGR)
        // {
            msg = cv_bridge::CvImage(frameHeader, "rgb8", frame).toImageMsg();
        // }
        // else
        // {
        //     msg = cv_bridge::CvImage(frameHeader, "rgba8", frame).toImageMsg();
        // }

        fillCameraInfoFromWebcam(iFrame->FrameIndex, width, height);
        Imagepublisher.publish(*msg, mCi);

    }

    InuDev::CInuError CRosWebcamPublisher::RegisterCallback()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CImageStream::CallbackFunction callback = std::bind(&CRosWebcamPublisher::FrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        return std::static_pointer_cast<InuDev::CImageStream>(stream)->Register(callback);
    }

    InuDev::CInuError CRosWebcamPublisher::UnregisterCallback()
    {
        return std::static_pointer_cast<InuDev::CImageStream>(stream)->Register(nullptr);
    }

    int CRosWebcamPublisher::GetNumSubscribers()
    {
        publisherPrev = publisherCurr;
        publisherCurr = Imagepublisher.getNumSubscribers();

        return publisherCurr;
    }

    InuDev::CInuError CRosWebcamPublisher::InitStream()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CInuError err(0);

	if (mPostProcessing != 0)
        {
            if(mPostProcessing | InuDev::CImageStream::EPostProcessing::eRegistered && mRegistrationChannel != -1)
            {
                #ifdef _GE428_
                err = std::static_pointer_cast<InuDev::CImageStreamExt>(stream)->Init((InuDev::CImageStream::EOutputFormat)mOutputFormat,(InuDev::CImageStreamExt::EPostProcessing)mPostProcessing, mRegistrationChannel);
                #else
                err = std::static_pointer_cast<InuDev::CImageStreamExt>(stream)->Init((InuDev::CImageStream::EOutputFormat)mOutputFormat,(InuDev::CImageStreamExt::EPostProcessingExt)mPostProcessing, mRegistrationChannel);
                #endif
                RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << "Starting RGB with post processing : " << mPostProcessing << " With registration channel: " << mRegistrationChannel << "With output format: " << mOutputFormat);
            }
            else
            {
                #ifdef _GE428_
                err = std::static_pointer_cast<InuDev::CImageStreamExt>(stream)->Init((InuDev::CImageStream::EOutputFormat)mOutputFormat,(InuDev::CImageStreamExt::EPostProcessing)mPostProcessing, InuDev::DEFAULT_CHANNEL_ID);
                #else
                err = std::static_pointer_cast<InuDev::CImageStreamExt>(stream)->Init((InuDev::CImageStream::EOutputFormat)mOutputFormat,(InuDev::CImageStreamExt::EPostProcessingExt)mPostProcessing, InuDev::DEFAULT_CHANNEL_ID);
                #endif
                RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << "Starting RGB with post processing: " << mPostProcessing << "With output format: " << mOutputFormat);
            }
        }
        else
        {
            err = std::static_pointer_cast<InuDev::CImageStream>(stream)->Init((InuDev::CImageStream::EOutputFormat)mOutputFormat);
            RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << " Starting RGB with output format: " << mOutputFormat);
        }

    if (err != InuDev::EErrorCode::eOK)
    {
        RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
        return err;
    }
        return InuDev::EErrorCode::eOK;
}


    InuDev::CInuError CRosWebcamPublisher::StartStream()
    {
        // Load default params from server

        if(sensor->dynamicParams.find("RGBChannel") != sensor->dynamicParams.end())
        {
            mParamsChannelID = sensor->dynamicParams.find("RGBChannel")->second.as_int();
        }

        if(sensor->dynamicParams.find("RGBOutputFormat") != sensor->dynamicParams.end())
        {
            mOutputFormat = sensor->dynamicParams.find("RGBOutputFormat")->second.as_int();
        }

        if(sensor->dynamicParams.find("RGBPostProcessing") != sensor->dynamicParams.end())
        {
            mPostProcessing = sensor->dynamicParams.find("RGBPostProcessing")->second.as_int();
        }

        if(sensor->dynamicParams.find("RGBRegistrationChannel") != sensor->dynamicParams.end())
        {
            mRegistrationChannel = sensor->dynamicParams.find("RGBRegistrationChannel")->second.as_int();
        }

        if (mParamsChannelID == -1)
        {
            stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateImageStreamExt());
        }
        else
        {
            stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateImageStreamExt(mParamsChannelID));
        }


        InuDev::EErrorCode err = InitStream();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        err = CRosPublisher::StartStream();

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
