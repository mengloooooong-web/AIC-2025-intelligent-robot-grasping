#pragma once

/*
 * File - config.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014-2020 Inuitive All Rights Reserved
 *
 */


/**
 * \file config.h
 *
 * \brief inudev_ros_nodelt configuration
 */

/**
 * \brief Name of OpenCV display window
 */
#define CV_WINDOW_NAME "Display Window"

/**
 * \brief Pertty function name
 *
 * CPP construct to use when emittiy function name
 */
#define __INUROS_FUNCTION_NAME__ (__PRETTY_FUNCTION__)

/**
 * \brief Max number of stack frames in stack trace
 *
 * Sets the maximal number of stack frames to emit in NotImplementedException stack trace output
 */
#define MAX_STACK_FRAMES (100)

/**
 * \brief inudev_ros_nodelet namespece
 *
 * Specifices namespace to use
 */
#define __INUROS__NAMESPACE__ inuros2

/**
 * \brief POINTCLOUD_DUMP_FRAME
 *
 * Enable PointCloud InuSW saving to file
 */
#define POINTCLOUD_DUMP_FRAME (0)

#if POINTCLOUD_DUMP_FRAME

/**
 * \brief POINTCLOUD_DUMP_FILENAME
 *
 * Prefix to filename to store PointCloud dumps
 */
#define POINTCLOUD_DUMP_FILENAME "/tmp/InuROS_PointCloud"

#endif

/**
 * \brief ROS Image Topic
 *
 * defines the ROS Topic upon which image will be published
 *
 * \remark <b>MUST</b> end with "&#47;".
 */

#define INROS_PATH "//opt//ros//galactic//share//inudev_ros_nodelet"
