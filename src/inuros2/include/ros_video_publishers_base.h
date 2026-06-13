#pragma once

/*
 * File - ros_video_publishers_base.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_publisher.h"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_video_publishers_base.h
 *
 * \brief Common ROS video publisher base class
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief Common ROS Video Publisher base class
     */
    class CRosVideoPublishersBase: public CRosPublisher
    {

    protected:

        /**
         * \brief fillCameraInfoFromVideo
         *
         * Fille ROS CameraInfo for frame originating from Video sensor
         */
        void fillCameraInfoFromVideo(unsigned int frameIndex, int width, int height, int bRightImage);

    public:

        /**
         * \brief CRosVideoPublishersBase
         *
         * CRosVideoPublishersBase constructor
         */
        CRosVideoPublishersBase(rclcpp::Node::SharedPtr& _node, std::string _str, CRosSensor* _rosSensor);

    };

}
