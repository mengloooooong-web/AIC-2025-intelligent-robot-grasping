/*
 * File - ros_video_publishers_base.cpp
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
//#include <sensor_msgs/distortion_models.h>
#include <opencv2/opencv.hpp>

/**
 * \endcond
 */

#include "ros_video_publishers_base.h"

#include "inuros2_node_factory.h"

/**
 * \file ros_video_publishers_base.cpp
 *
 * \brief CRosVideoPublishersBase
 *
 * This is the base class for all publishers sharing the video sensors (depth, disparity, video)
 *
 */

namespace __INUROS__NAMESPACE__
{
    CRosVideoPublishersBase::CRosVideoPublishersBase(rclcpp::Node::SharedPtr& _node, std::string _str, CRosSensor* _rosSensor)
        : CRosPublisher(_node, _str, _rosSensor)
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
    }

    void CRosVideoPublishersBase::fillCameraInfoFromVideo(unsigned int frameIndex, int width, int height, int bRightImage)
    {
        std::shared_ptr<InuDev::CCalibrationData> cd = sensor->GetCalibrationData(mCurrentTemprature);

        mCi.header.frame_id  = getName();
        //mCi.header.seq       = frameIndex;
        mCi.width            = width;
        mCi.height           = height;

        //mCi.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        mCi.distortion_model = "plumb_bob";
        if (!mSensors.empty())
        {
            auto c = cd->Sensors[mSensors.at(0)].VirtualCamera.Intrinsic;

            if (!bRightImage)
            {
                c = cd->Sensors[mSensors.at(1)].VirtualCamera.Intrinsic;
            }

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

            if (!bRightImage)
            {
                auto m = cd->Baselines.begin();

                mCi.p[ 3] = m->second;
            }
        }
    }
}
