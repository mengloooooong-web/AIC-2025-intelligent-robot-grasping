#pragma once

/*
 * File - ros_depth_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_video_publishers_base.h"

#include "DepthStreamExt.h"
#include "depth2pc.h"
/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_depth_publisher.h
 *
 * \brief Depth publisher
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS depth publisher
     */
    class CRosDepthPublisher : public CRosVideoPublishersBase
    {
        image_transport::Publisher publisher;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pcl;
        bool mDept2PcSupportted = false;

        void FrameCallback(std::shared_ptr<InuDev::CDepthStream> iStream, std::shared_ptr<const InuDev::CImageFrame> iFrame, InuDev::CInuError iError);
        InuDev::CInuError RegisterCallback();
        InuDev::CInuError UnregisterCallback();

    protected:

        /**
         * \brief StartStream
         *
         * Start depth stream
         */
        virtual InuDev::CInuError StartStream() override;
        virtual InuDev::CInuError InitStream() override;
    public:

        /**
         * \brief Get number of subscribers for publisher
         *
         * Returns number of subscibers to ROS messages this publisher publishes.
         */
        virtual int GetNumSubscribers() override;

        /**
         * \brief ROS Depth Publisher
         */
        CRosDepthPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicName = "camera/aligned_depth_to_color/image_raw");
    private:

    	Inuchip::COptData mOD;
        void LoadOD();
        void pub_pcl(const cv::Mat& dep, uint64_t curTs, bool bOrder, int rinter, int cinter);
        std::shared_ptr<InuDev::CCalibrationData> mCalibrationData;


        int settingConfidence(int newconfidence, bool direct);
        // Channel recieved from InuRosParams: -1 is default
        int mParamsChannelID;

        // OutptFornat: 0 is default (depth)
        int mOutputFormat = 0;

        // Registration Channel: - 4 is default;
        int mRegistrationChannel;

        // PostProcessing flag: 0xff default as defined by the host
        int mPostProcessing = 0xff;

        // Show Color - 0 is default
        bool mShowColor;

        // opticalCenter,FocalLength
        float x0,y0,fx,fy;

        void GetParams();

        int mSimplify2;

        // Depth Frame ID
        std::string mDepthFrameID;

        // Temporal Filter: 0 is default
        bool mTemporalFilterROS;

        // Temporal2 Filter: 0 is default
        bool mTemporal2FilterROS;
        int mTemporal2FilterAlpha;
        int mTemporal2FilterDelta;

        // Edge Filter: 0 is default
        bool mEdgePreserveFilterROS;
        int mEdgePreserveFilterAlpha;
        int mEdgePreserveFilterDelta;

        // Temporal Filter: 0 is default
        bool mTemporalFilterSDK;

        // Passthrough Filter: 0 is default
        bool mPassthroughFilterROS;

        // Passthrough Filter: 0 is default
        bool mPassthroughFilterSDK;

        // OutlierRemove Filter: 0 is default
        bool mOutlierRemoveFilterROS;

        // OutlierRemove Filter: 0 is default
        bool mOutlierRemoveFilterSDK;

        // Hole Filter: - 0 is default
        bool mHoleFillFilterROS;

        // Hole Filter: - 0 is default
        bool mHoleFillFilterSDK;

        // Temporal_BufferFrames - 0 is default
        int mTemporalBufferFrames;

        // Temporal_ProcessAllPixels - 0 is default
        bool mTemporalProcessAllPixel;

        // Temporal_Threads
        int mTemporalThreads;

        // PT_zmin - 0 is default
        int mPTzmin;

        // PT_zmax - 0 is default
        int mPTzmax;

        // PT_xmin - 0 is default
        int mPTxmin;

        // PT_xmax - 0 is default
        int mPTxmax;

        // PT_ymin - 0 is default
        int mPTymin;

        // PT_ymax - 0 is default
        int mPTymax;

        // OR_percent - 0 is default
        int mORpercent;

        // mORminDist
        int mORminDist;

        // mORmaxRemove
        int mORmaxRemove;

        // HF_maxradius - 0 is default
        int mHFmaxRadius;

        // PC_scale - 0 is default
        int mPCscale;

        // PC_ordered
        bool mPCordered;

        // depth confidence
        int mDepthConfidence;

    };
}
