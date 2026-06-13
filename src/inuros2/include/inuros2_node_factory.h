#pragma once

/*
 * File - inuros2_node_factory.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */

#include "ros_sensor.h"

//#include <dynamic_reconfigure/server.h>
//#include <inudev_ros_nodelet/inudev_ros_nodeletConfig.h>
#include "rclcpp_components/register_node_macro.hpp"
#if defined(USE_LIFECYCLE_MANAGEMENT)
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#endif

/**
 * \file inudev_ros_nodelet.h
 *
 * \brief ROS nodelet class
 */

namespace __INUROS__NAMESPACE__
{
    /**
     * \brief ROS inuros2_node_factory Node
     */
    class CInuDevRosNodeFactory : 
#if defined(USE_LIFECYCLE_MANAGEMENT)
        public rclcpp_lifecycle::LifecycleNode
#else
        public rclcpp::Node
#endif
    {
        rclcpp::Node::SharedPtr myNode;

        //std::shared_ptr<dynamic_reconfigure::Server<inudev_ros_nodelet::inudev_ros_nodeletConfig>> server;

        std::shared_ptr<CRosSensor> mRosSensor;

        float fps;

        void Init();

        //void callback(inudev_ros_nodelet::inudev_ros_nodeletConfig &newConfig, uint32_t level);

    public:

        CInuDevRosNodeFactory(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
        //CInuDevRosNodelet();

        virtual ~CInuDevRosNodeFactory();

        void timerCallback();


        /**
         * \brief Nodelet configuration
         *
         * Holds the nodelet's dynamic configuration.
         */
        //static inudev_ros_nodelet::inudev_ros_nodeletConfig config;

        /**
         * \brief Control inudev_ros_nodelet verbosity
         *
         * If non-zero, inudev_ros_nodelet will emit info related to frames received by callbacks
         */
        static int mVerbose;

        /**
         * \brief Get configuration object
         *
         * \return Reference for inudev_ros_nodeleteConfig object
         */
        /*const inudev_ros_nodelet::inudev_ros_nodeletConfig& getConfig()
        {
            return config;
        }
*/
private:
    rclcpp::Logger logger;

    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();

    protected:
#if defined(USE_LIFECYCLE_MANAGEMENT)
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
#endif

    };
}
