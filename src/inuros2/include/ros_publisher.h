#pragma once

/*
 * File - ros_publisher.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

/**
 * \cond INTERNAL
 */

#include <string>
#include <functional>

#ifdef USE_H
  #include <image_transport/image_transport.h>
#else
  #include <image_transport/image_transport.hpp>
#endif

//#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/msg/camera_info.hpp>
//#include <image_publisher/ImagePublisherConfig.h>

/**
 * \endcond
 */

#include "InuSensorExt.h"

#include "ros_publisher_bookkeeping.h"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_publisher.h
 *
 * \brief ROS publisher
 */

namespace __INUROS__NAMESPACE__
{
    class CRosSensor;

    /**
     * \brief ROS Publisher
     *
     * ROS Publisher base class
     */
    class CRosPublisher : public CRosPublisherBookkeeping
    {
    protected:

        rclcpp::Logger logger;

        /**
         * \brief InuDev ChannelID for the associated InuDev stream
         */
        int mChannelID;

        /**
         * \brief List of InuDev sensors associated with this channel
         */
        std::vector<int> mSensors;

        /**
         * \brief Perform InuDev stream initialization releated to ROS publisher
         *
         * Each ROS publisher is associated with InuDev stream. This methos is invoked
         * to initialize the InuDev stream assosicated with this ROS publisher's data
         */
        virtual InuDev::CInuError InitStream();

        /**
         * \brief RegisterCallback
         *
         * Method to register callback for the InuDEv stream associated with this publisher
         *
         * \return InuDev error value
         */
        virtual InuDev::CInuError RegisterCallback() = 0;

        /**
         * \brief UnregisterCallback
         *
         * Method to un-regisdter InuDev callbak on the InuDev stream used by this publisher
         *
         * \return InuDev error value
         */
        virtual InuDev::CInuError UnregisterCallback() = 0;

        /**
         * \brief InuDEv stream associated with publisher
         *
         * Holds the InuDev stream used to create frames published by this Publisher
         */
        std::shared_ptr<InuDev::CBaseStream> stream;

        /**
         * \brief Check InuDev frame for errors
         *
         * \param iFrame    Frame to check
         * \param iError    Error code provided at callback
         *
         * \return true if frame is in error
         *
         * Performs basic InuDev frame validation for frames received by callbacks
         */
        bool CheckFrame(std::shared_ptr<const InuDev::CBaseFrame> iFrame, InuDev::CInuError iError);

        /**
         * \brief node
         *
         * ROS node for this nodelet
         */
        //ros::NodeHandle node;
        //rclcpp::Node node;

        /**
         * \brief cinfo
         *
         * Camera Info manager
         *
         * See ROS <a href="www.ros.org/reps/rep-0104.html">CameraInfo</a> doc for more info.
         */
        //camera_info_manager::CameraInfoManager cinfo;
        //sensor_msgs::msg::CameraInfo cinfo;

        /**
         * \brief ci
         *
         * Holds camera info for the publisher: As each publisher is associated with an image source, that
         * source's camera info is stored here.
         */
        sensor_msgs::msg::CameraInfo mCi;

        /**
         * \brief sensor
         *
         * Represents InuDev sensor
         *
         * \remark This is not a shared ptr as the lifetime of the publisher fully encompasses that of the sensor's.
         */
        CRosSensor *sensor;

        /**
         * \brief Current Temperature
         */
        int mCurrentTemprature;

        /**
         * \brief CRosPublisher
         *
         * CRosPublisher constructor.
         */
        CRosPublisher(rclcpp::Node::SharedPtr _node, std::string _str, CRosSensor* _rosSensor);
    public:
        virtual InuDev::CInuError StartStream();
        virtual InuDev::CInuError StopStream();

        virtual ~CRosPublisher();

        virtual int GetNumSubscribers() override;
    };
}
