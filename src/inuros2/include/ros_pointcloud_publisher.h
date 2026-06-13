#pragma once

/*
 * File - ros_pointcloud_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_publisher.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "PointCloudStream.h"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_pointcloud_publisher.h
 *
 * \brief PointCloud publisher
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS PointCloud publisher
     */
    class CRosPointcloudPublisher : public CRosPublisher
    {
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;

        void FrameCallback(std::shared_ptr<InuDev::CPointCloudStream> iStream, std::shared_ptr<const InuDev::CPointCloudFrame> iFrame, InuDev::CInuError iError);
        void RegisteredFrameCallback(std::shared_ptr<InuDev::CPointCloudStream> iStream, std::shared_ptr<const InuDev::CPointCloudFrame> iFrame, InuDev::CInuError iError);

        InuDev::CInuError RegisterCallback();
        InuDev::CInuError UnregisterCallback();

        static std::vector<const char*> fieldNames;

    protected:

        virtual InuDev::CInuError InitStream() override;
        virtual InuDev::CInuError StartStream() override;

    public:

        virtual int GetNumSubscribers() override;

        /**
         * \brief CRosPointcloudPublisher
         *
         * CRosPointcloudPublisher constructor
         */
        CRosPointcloudPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicName = "sensor/PointCloud2");

    private:
        // Point cloud format: 0 Default (XYZ), 1 Registered (XYZRGB)
        int mPointCloudFormat = 0;

        // Point cloud Registration channel. Numeric max for default.
        int mRGBRegistrationChannel = std::numeric_limits<uint32_t>::max();

        // Defines whether to convert short point to float
        bool mConvertPointCloudShortToFloat;

        // PC_scale - 1000 is default
        float mPointCloudScale;
    };


}
