/*
 * File - ros_publisher.cpp
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

/**
 * \cond INTERNAL
 */

//#include "config.h"

#include <limits>

//#include <nodelet/nodelet.h> TODO: Replace nodelet with ROS2 interface
#include <rclcpp/rclcpp.hpp>

/**
 * \endcond
 */

//#include "NotImplementedException.h"
#include "ros_sensor.h"

/**
 * \file ros_publisher.cpp
 *
 * \brief CRosPublisher
 */

namespace __INUROS__NAMESPACE__
{
    int errCount = 0;
    int bRestart = 0;

    InuDev::CInuError CRosPublisher::StartStream()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!sensor || !sensor->connected())
            return InuDev::EErrorCode::eStateError;

        // Initialize and start the stream

        InuDev::CInuError err = InitStream(); // generic!

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": Init: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        err = stream->Start();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": Start: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        err = RegisterCallback(); // generic!

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": Register callback: 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": Successfully started " << getName());

        return InuDev::EErrorCode::eOK;
    }

    InuDev::CInuError CRosPublisher::StopStream()
    {
        RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!stream)
        {
            // Here if the sensor is stopped, but the specific stream has already been stopped. OK.

            //RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << " already stopped");

            return InuDev::EErrorCode::eOK;
        }

        // UnregisterCallback();

        stream->Stop();
        stream->Terminate();

        stream = nullptr;

        return InuDev::EErrorCode::eOK;
    }

    CRosPublisher::CRosPublisher(rclcpp::Node::SharedPtr _node, std::string _str, CRosSensor* _rosSensor)
        : CRosPublisherBookkeeping(_str)
        , sensor(_rosSensor)
        , mCurrentTemprature(std::numeric_limits<int>::min())
        , mChannelID(-1)
        , logger(_node->get_logger())

    {
        RCLCPP_WARN_STREAM(logger ,__INUROS_FUNCTION_NAME__ << ": " << getName());

    }

    CRosPublisher::~CRosPublisher()
    {
        RCLCPP_INFO_STREAM(logger ,__INUROS_FUNCTION_NAME__ << ": " << getName());

        //it won't work to call StopStream in the destructor of the base class. because
        //the compiler assumes that at this point the sub class is already destructed,
        //therefore, it won't invoke any virtual function of the sub class. only base
        //class version will be called. as alternative, we must call StopStream explicitly
        //before invoking the sub class destructors
        //StopStream();
    }

    InuDev::CInuError CRosPublisher::InitStream()
    {
        RCLCPP_INFO_STREAM(logger ,__INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!stream)
        {
            RCLCPP_ERROR_STREAM(logger ,__INUROS_FUNCTION_NAME__ << ": " << getName() << " stream null");
        }

        InuDev::CInuError err = stream->Init();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger ,__INUROS_FUNCTION_NAME__ << ": " << getName() << " " << std::string(err) << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        // Every InuDev channel has a ChannelID. Get it.

        uint32_t tmp;
        err = stream->GetChannel(tmp);

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger ,__INUROS_FUNCTION_NAME__ << ": " << getName() << " " << std::string(err) << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        mChannelID = int(tmp);

        mSensors = sensor->GetSensors(mChannelID);

        return InuDev::EErrorCode::eOK;
    }

    bool CRosPublisher::CheckFrame(std::shared_ptr<const InuDev::CBaseFrame> iFrame, InuDev::CInuError iError)
    {
        if (!sensor || !sensor->connected())
            return false;

        if (iError != InuDev::EErrorCode::eOK)
        {
            errCount++;
            if (errCount > 20  && bRestart == 0)
            {
                bRestart = 1;
                errCount = 0;

                // Log restart sensor
                RCLCPP_ERROR_STREAM(logger , __INUROS_FUNCTION_NAME__ << " ============= Stop Sensor ==================\n");
                sensor->StopStream();

                // Log Start sensor
                RCLCPP_ERROR_STREAM(logger , __INUROS_FUNCTION_NAME__ << " ============= Start Sensor ==================\n");
                sensor->StartStream();


                RCLCPP_ERROR_STREAM(logger , __INUROS_FUNCTION_NAME__ << " ============= Publish Recovery ===============\n");
                UpdatePublisherRecover();

            }

            RCLCPP_ERROR_STREAM(logger ,__INUROS_FUNCTION_NAME__ << ": frame " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(iError));
            return false;
        }

        if (!iFrame->Valid)
        {
            RCLCPP_ERROR_STREAM(logger ,__INUROS_FUNCTION_NAME__ << ": frame " << getName() << " invalid");
            return false;
        }

        errCount = 0;
        bRestart = 0;
        mCurrentTemprature = iFrame->CalibrationTemperature;

        return true;
    }

    int CRosPublisher::GetNumSubscribers()
    {
        //throw NotImplementedException(__INUROS_FUNCTION_NAME__);
    }

}
