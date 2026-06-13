#pragma once

/*
 * File - ros_publisher_bookkeeping.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "InuErrorExt.h"

/**
 * \cond INTERNAL
 */

#include "config.h"

/**
 * \endcond
 */

/**
 * \file ros_publisher_bookkeeping.h
 *
 * \brief ros publisher bookkeeping
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS Publisher Bookkeeping
     *
     * Track publisher subscribers
     */
    class CRosPublisherBookkeeping
    {
        std::string str;

    protected:

        /**
         * \brief publisherCurr
         *
         * Current number of subscribers on topic
         */
        int publisherCurr;

        /**
         * \brief publisherPrev
         *
         * Previous number of subscribers on topic
         */
        int publisherPrev;

        /**
         * \brief CRosPublisherBookkeeping
         *
         * CRosPublisherBookkeeping constructor
         */
        CRosPublisherBookkeeping(const std::string _str);

    public:
        /**
         * \cond INTERNAL
         */

        /**
         * \brief getName
         *
         * This method is used throughout ROS to report the entitiy's name
         *
         * \return Emtitiy's name
         */
        const std::string getName() const;

        /**
         * \endcond
         */

        /**
         * \brief StartStream
         *
         * Start InuDev stream assosicated with publisher. This method will only be invoked
         * if the number of subscribers on topic has increased from 0.
         */
        virtual InuDev::CInuError StartStream() = 0;

        /**
         * \brief StopStream
         *
         * Stop InuDev stream associated with publisher. This method will only be invoked
         * if the number of subscribers on topic has decreased to 0.
         */
        virtual InuDev::CInuError StopStream() = 0;

        /**
         * \brief MiddleUpdate
         *
         * Operation to be performed between StartStream and StopStream. Performed periodicaly.
         */
        virtual void MiddleUpdate();


        //rclcpp::Logger logger;


        /**
         * \brief GetNumSubscribers
         *
         * Obtain the number of subscribers for the topic associated with the publisher
         *
         * \return Number of subscribers on topic
         */
        virtual int GetNumSubscribers() = 0;

        /**
         * \brief Update publisher based on subscriber count
         *
         * Invokes start/stop stream as appropriate to change in subscriber count
         */
        virtual void UpdatePublisher();

        /**
         * \brief Update publisher When recovering from error
         *
         * Invokes start/stop stream as appropriate to change in subscriber count
         */
        virtual void UpdatePublisherRecover();
    };
}
