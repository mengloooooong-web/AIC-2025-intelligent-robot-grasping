/*
 * File - ros_publisher_bookkeeping.cpp
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

/**
 * \endcond
 */

#include "ros_publisher_bookkeeping.h"

/**
 * \file ros_publisher_bookkeeping.cpp
 *
 * \brief CRosPublisherBookkeeping
 */

namespace __INUROS__NAMESPACE__
{
    CRosPublisherBookkeeping::CRosPublisherBookkeeping(const std::string _str)
        : publisherCurr(0)
        , publisherPrev(0)
        , str(_str)
    {
        // empty
    }

    /**
     * \cond INTERNAL
     */
    const std::string CRosPublisherBookkeeping::getName() const
    {
        return str;
    }
    /**
     * \endcond
     */

    void CRosPublisherBookkeeping::UpdatePublisher()
    {
        if (true
            && publisherPrev == 0
            && publisherCurr > 0
            )
        {
            //RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Startng " << getName() << " : subscribers count=" << publisherCurr);
            StartStream();
        }

        MiddleUpdate();

        if (true
            && publisherPrev > 0
            && publisherCurr == 0
            )
        {
            //RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": Stopping " << getName());

            StopStream();
        }
    }

    void CRosPublisherBookkeeping::UpdatePublisherRecover()
    {
        if (true && publisherCurr > 0)
        {

            InuDev::CInuError err=StartStream();
        }
        MiddleUpdate();
    }

    void CRosPublisherBookkeeping::MiddleUpdate()
    {
        // empty
    }
}
