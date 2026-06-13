/*
 * File - inuros2_node_factory.cpp
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */


#include "config.h"
#include "Version.h"
#include "rclcpp/rclcpp.hpp"

//#include <nodelet/nodelet.h>
//#include <pluginlib/class_list_macros.h>


#include "inuros2_node_factory.h"

/**
 * \file inuros2_node_factory.cpp
 *
 * \brief ROS Node Factory
 */

using namespace std::chrono_literals;
namespace __INUROS__NAMESPACE__
{

    CInuDevRosNodeFactory::CInuDevRosNodeFactory(const rclcpp::NodeOptions & node_options)
#if defined(USE_LIFECYCLE_MANAGEMENT)
        : rclcpp_lifecycle::LifecycleNode("InuRos", "/", node_options)
#else
        : rclcpp::Node("InuRos", "/", node_options)
#endif
        , fps(10.0)
        , mRosSensor(nullptr)
        , logger(this->get_logger())
    {
        RCLCPP_INFO_STREAM(logger, "Creating InuRos2 Node factory.");
        Init();
    }

    CInuDevRosNodeFactory::~CInuDevRosNodeFactory()
    {
        RCLCPP_INFO_STREAM(logger," Destorying InuRos2 Node factory.");
    }

    void CInuDevRosNodeFactory::Init()
    {
        RCLCPP_INFO_STREAM(logger,"Initializing InuRos2 Node factory");
        RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": Version " << version);

        //privNh.param<int>("verbose", mVerbose, 0);
        //RCLCPP_INFO_STREAM(__INUROS_FUNCTION_NAME__ << ": param(verbose)=" << mVerbose);

        /* sensor */

        myNode = rclcpp::Node::make_shared("sensor_node");
        mRosSensor = std::make_shared<CRosSensor>(myNode, fps, "sensor_node");

#if !defined(USE_LIFECYCLE_MANAGEMENT)
        if (mRosSensor->connect() || mRosSensor->start_publish())
        {
            throw std::logic_error("CRosSensor");
        }
#endif

        timer_ = create_wall_timer(1s, std::bind(&CInuDevRosNodeFactory::timerCallback, this));

        RCLCPP_INFO_STREAM(logger,"Node factory Init done");

        //mRosSensor->getSensor()->SetProjectorLevel(InuDev::EProjectorLevel::eHigh, InuDev::EProjectors::ePatterns);

    }
    void CInuDevRosNodeFactory::timerCallback()
    {
        mRosSensor->timerCallback();
        mRosSensor->connectionCheckCallback();
        rclcpp::spin_some(myNode);
    }

#if defined(USE_LIFECYCLE_MANAGEMENT)
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CInuDevRosNodeFactory::on_configure(const rclcpp_lifecycle::State& state)
    {
        int ret = -1;
        RCLCPP_INFO_STREAM(logger, "Node factory function: " << __func__);
        if (mRosSensor)
            ret = mRosSensor->connect();
        return ret != 0 ? rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE : rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CInuDevRosNodeFactory::on_activate(const rclcpp_lifecycle::State& state)
    {
        int ret = -1;
        RCLCPP_INFO_STREAM(logger, "Node factory function: " << __func__);
        if (mRosSensor)
            ret = mRosSensor->start_publish();
        return ret != 0 ? rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE : rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CInuDevRosNodeFactory::on_deactivate(const rclcpp_lifecycle::State& state)
    {
        RCLCPP_INFO_STREAM(logger, "Node factory function: " << __func__);
        if (mRosSensor)
            mRosSensor->stop_publish();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CInuDevRosNodeFactory::on_cleanup(const rclcpp_lifecycle::State& state)
    {
        RCLCPP_INFO_STREAM(logger, "Node factory function: " << __func__);
        if (mRosSensor)
            mRosSensor->disconnect();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CInuDevRosNodeFactory::on_shutdown(const rclcpp_lifecycle::State& state)
    {
        RCLCPP_INFO_STREAM(logger, "Node factory function: " << __func__);
        if (mRosSensor) {
            mRosSensor->stop_publish();
            mRosSensor->disconnect();
            mRosSensor.reset();
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
#endif
}

RCLCPP_COMPONENTS_REGISTER_NODE(inuros2::CInuDevRosNodeFactory)
