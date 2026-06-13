/*
 * File - ros_depth_publisher.cpp
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
//#include <ros/rate.h>
#include <sensor_msgs/msg/camera_info.hpp>
//#include <sensor_msgs/distortion_models.h>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
/**
 * \endcond
 */

#include "ros_depth_publisher.h"
#include "inuros2_node_factory.h"


#include "temporal_filter.h"
#include "space_filter.h"


// include PCL headers
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include "DepthStreamExt.h"

/**
 * \file ros_depth_publisher.cpp
 *
 * \brief CRosDepthPublisher
 *
 * This publisher will publish the following topics:
 *
 * - /sensor_msgs/Image/Depth/depth
 */

namespace __INUROS__NAMESPACE__ {

    Inuchip::Space_Filter sf;
    Inuchip::TemporalFast tf2;

    int mDump = 1;

    CRosDepthPublisher::CRosDepthPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicName)
        : CRosVideoPublishersBase(_node, "depth", _rosSensor)
        , mParamsChannelID(-1)
        , mOutputFormat(-1)
        , mRegistrationChannel(4)
        , mTemporalFilterROS(0)
        , mTemporalFilterSDK(0)
        , mPassthroughFilterROS(0)
        , mPassthroughFilterSDK(0)
        , mOutlierRemoveFilterROS(0)
        , mOutlierRemoveFilterSDK(0)
        , mHoleFillFilterROS(0)
        , mHoleFillFilterSDK(0)
        , mShowColor(0)
        , mSimplify2(0)
        , mDepthFrameID("")

        , mTemporalBufferFrames(6)
        , mTemporalProcessAllPixel(0)
        , mTemporalThreads(4)

        , mPTzmin(40)
        , mPTzmax(6000)
        , mPTxmin(-1)
        , mPTxmax(-1)
        , mPTymin(-1)
        , mPTymax(-1)

        , mORpercent(10)
        , mORminDist(35)

        , mHFmaxRadius(25)

        , mPCscale(1)
        , mPCordered(0)
    {
        // Get dynamic params
        GetParams();

        const std::string serviceId = _rosSensor->GetSensorId();
        if (!serviceId.empty())
        {
            iTopicName = "service" + serviceId + "/" + iTopicName;
        }
        publisher = image_transport::create_publisher(_node.get(), iTopicName);
        if (mDept2PcSupportted)
        {
          publisher_pcl = _node->create_publisher<sensor_msgs::msg::PointCloud2>("depth2pc", 50);
        }
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
    }

    const unsigned char BLut[256] =
    {

        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  44,  48,  52,  56,  60,  64,
        68,  72,  76,  80,  84,  88,  92,  96, 100, 104, 108, 112, 116, 120, 124, 128,
        132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176 ,180, 184, 188, 192,
        196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        252, 248, 244, 240, 236, 232, 228, 224, 220, 216, 212, 208, 204, 200, 196, 192,
        188, 184, 180, 176, 172, 168, 164, 160, 156, 152, 148, 144, 140, 136, 132, 128,
    };

    const unsigned char GLut[256] =
    {
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  44,  48,  52,  56,  60,  64,
        68,  72,  76,  80,  84,  88,  92,  96, 100, 104, 108, 112, 116, 120, 124, 128,
        132, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192,
        196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        252, 248, 244, 240, 236, 232, 228, 224, 220, 216, 212, 208, 204, 200, 196, 192,
        188, 184, 180, 176, 172, 168, 164, 160, 156, 152, 148, 144, 140, 136, 132, 128,
        124, 120, 116, 112, 108, 104, 100,  96,  92,  88,  84,  80,  76,  72,  68,  64,
        60,  56,  52,  48,  44,  40,  36,  32,  28,  24,  20,  16,  12,   8,   4,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    };

    const unsigned char RLut[256] =
    {
        0, 136, 140, 144, 148, 152, 156, 160, 164, 168, 172, 176, 180, 184, 188, 192,
        196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
        252, 248, 244, 240, 236, 232, 228, 224, 220, 216, 212, 208, 204, 200, 196, 192,
        188, 184, 180, 176, 172, 168, 164, 160, 156, 152, 148, 144, 140, 136, 132, 128,
        124, 120, 116, 112, 108, 104, 100,  96,  92,  88,  84,  80,  76,  72,  68,  64,
        60,  56,  52,  48,  44,  40,  36,  32,  28,  24,  20,  16,  12,   8,   4,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    };

    cv::Mat Short2RGB(const cv::Mat& imIn)
    {
        cv::Mat imOut(imIn.size(), CV_8UC3);
        int maxDepth = 6000;//4000
        int minDepth = 0;
        float nomrmalizeFactor = (maxDepth - minDepth);

        for (int r = 0; r < imIn.rows; r++)
        {
            const ushort* in_ptr = imIn.ptr<ushort>(r);
            cv::Vec3b* out_ptr = imOut.ptr<cv::Vec3b>(r);
            for (int c = 0; c < imIn.cols; c++)
            {
                int noramlizedZ = int(255.0f * (in_ptr[c] - minDepth) / nomrmalizeFactor + 0.5f);
                out_ptr[c][0] = BLut[noramlizedZ];
                out_ptr[c][1] = GLut[noramlizedZ];
                out_ptr[c][2] = RLut[noramlizedZ];

            }
        }

        return imOut;
    }

    void CRosDepthPublisher::LoadOD()
    {
        int iSen = 0;

        mOD.focalDepth[0] = mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.FocalLength[0];
        mOD.focalDepth[1] = mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.FocalLength[1];

        mOD.centerDepth[0] = mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.OpticalCenter[0];
        mOD.centerDepth[1] = mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.OpticalCenter[1];

        x0 =mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.OpticalCenter[0];
        y0 =mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.OpticalCenter[1];
        fx =1.0F / mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.FocalLength[0];
        fy =1.0F / mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.FocalLength[1];

        mOD.baselineDepth = mCalibrationData->Baselines[std::pair<int,int>(0,1)];


        //
        iSen = 2;

        // auto calibrationData = std::make_shared<InuDev::CCalibrationData>();
        sensor->getSensor()->GetCalibrationData(*mCalibrationData.get(), 4);
        // mCalibrationData = calibrationData; // shared_ptr assignemt

        if( std::stoi((sensor->calibrationVersion).substr(2,2)) < 19){

            if(mRegistrationChannel != -1)
            {
                #ifdef _GE428_
                    x0 =mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.OpticalCenter[0];
                    y0 =mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.OpticalCenter[1];
                    fx =1.0F / mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.FocalLength[0];
                    fy =1.0F / mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.FocalLength[1];
                #else
                    x0 =mCalibrationData->Sensors[iSen].RealCamera.Intrinsic.OpticalCenter[0];
                    y0 =mCalibrationData->Sensors[iSen].RealCamera.Intrinsic.OpticalCenter[1];
                    fx =1.0F / mCalibrationData->Sensors[iSen].RealCamera.Intrinsic.FocalLength[0];
                    fy =1.0F / mCalibrationData->Sensors[iSen].RealCamera.Intrinsic.FocalLength[1];
                #endif

            }

            mOD.focalWeb[0] = mCalibrationData->Sensors[iSen].RealCamera.Intrinsic.FocalLength[0];
            mOD.focalWeb[1] = mCalibrationData->Sensors[iSen].RealCamera.Intrinsic.FocalLength[1];
            mOD.centerWeb[0] = mCalibrationData->Sensors[iSen].RealCamera.Intrinsic.OpticalCenter[0];
            mOD.centerWeb[1] = mCalibrationData->Sensors[iSen].RealCamera.Intrinsic.OpticalCenter[1];

            for (size_t i = 0; i < mCalibrationData->Sensors[iSen].RealCamera.Intrinsic.LensDistortion.size(); i++ )
                mOD.KdWeb[i] = mCalibrationData->Sensors[iSen].RealCamera.Intrinsic.LensDistortion[i];



            mOD.rotAngle.x = mCalibrationData->Sensors[iSen].RealCamera.Extrinsic.Rotation[0];
            mOD.rotAngle.y = mCalibrationData->Sensors[iSen].RealCamera.Extrinsic.Rotation[1];
            mOD.rotAngle.z = mCalibrationData->Sensors[iSen].RealCamera.Extrinsic.Rotation[2];
            // mOD.RotRightWeb = OptData::buildRotation3D(mOD.rotAngle);

            mOD.WebcamTranslate[0] = mCalibrationData->Sensors[iSen].RealCamera.Extrinsic.Translation[0];
            mOD.WebcamTranslate[1] = mCalibrationData->Sensors[iSen].RealCamera.Extrinsic.Translation[1];
            mOD.WebcamTranslate[2] = mCalibrationData->Sensors[iSen].RealCamera.Extrinsic.Translation[2];
        }else{
            if(mRegistrationChannel != -1)
            {
                x0 =mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.OpticalCenter[0];
                y0 =mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.OpticalCenter[1];
                fx =1.0F / mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.FocalLength[0];
                fy =1.0F / mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.FocalLength[1];
            }

            mOD.focalWeb[0] = mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.FocalLength[0];
            mOD.focalWeb[1] = mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.FocalLength[1];
            mOD.centerWeb[0] = mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.OpticalCenter[0];
            mOD.centerWeb[1] = mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.OpticalCenter[1];

            for (size_t i = 0; i < mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.LensDistortion.size(); i++ )
                mOD.KdWeb[i] = mCalibrationData->Sensors[iSen].VirtualCamera.Intrinsic.LensDistortion[i];



            mOD.rotAngle.x = mCalibrationData->Sensors[iSen].VirtualCamera.Extrinsic.Rotation[0];
            mOD.rotAngle.y = mCalibrationData->Sensors[iSen].VirtualCamera.Extrinsic.Rotation[1];
            mOD.rotAngle.z = mCalibrationData->Sensors[iSen].VirtualCamera.Extrinsic.Rotation[2];
            // mOD.RotRightWeb = OptData::buildRotation3D(mOD.rotAngle);

            mOD.WebcamTranslate[0] = mCalibrationData->Sensors[iSen].VirtualCamera.Extrinsic.Translation[0];
            mOD.WebcamTranslate[1] = mCalibrationData->Sensors[iSen].VirtualCamera.Extrinsic.Translation[1];
            mOD.WebcamTranslate[2] = mCalibrationData->Sensors[iSen].VirtualCamera.Extrinsic.Translation[2];

        }

        // mOD.uvWeb[0] = mOD.uvWeb[1] = 0;
        mOD.Init();
    }

    void CRosDepthPublisher::pub_pcl(const cv::Mat& dep, uint64_t curTs, bool bOrder, int rinter, int cinter)
    {
        // sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2);
        sensor_msgs::msg::PointCloud2 cloudMsg;

        cv::Mat imgWeb;

        float fScale = 1.0F;
        if (mPCscale > 1) {
            fScale = 1.0F / mPCscale;
        }
        sensor->GetWebImage(imgWeb, curTs);
        int rtotal = 0;
        int ctotal = 0;

        // if (imgWeb.empty())
        if (true)
        {
#ifdef SPEED_OPT
            // RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << "=============  mat dimention: " << dep.rows << " " << dep.cols);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(dep.cols, dep.rows));
            int gidx = 0;
#else
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
#endif

            for (int y = 0; y < dep.rows; y++)
            {
                rtotal++;
                for (int x = 0; x < dep.cols; x++)
                {
                    if (y == 0) ctotal++;

                    int z = dep.at<ushort>(y, x);
                    if (z == 0 && !bOrder) continue;
                    int xw = z * (x * cinter - x0) * fx;
                    int yw = z * (y * rinter - y0) * fy;
#ifdef SPEED_OPT
                    //cloud->points.emplace_back(xw*fScale,yw*fScale,z*fScale);
                    cloud->points[gidx].x = xw * fScale;
                    cloud->points[gidx].y = yw * fScale;
                    cloud->points[gidx++].z = z * fScale;
#else
                    cloud->points.emplace_back(pcl::PointXYZ(xw * fScale, yw * fScale, z * fScale));
#endif
                }
            }
#ifdef SPEED_OPT
            cloud->points.erase(cloud->points.begin() + gidx, cloud->points.end());
            if (bOrder)
            {
                cloud->width = ctotal;
                cloud->height = rtotal;
            }
            else
            {
                cloud->width = gidx;
                cloud->height = 1;
            }
#endif

            pcl::toROSMsg(*cloud, cloudMsg);

            cloudMsg.width = gidx;
            cloudMsg.height = 1;

            cloudMsg.point_step = 16;
            cloudMsg.row_step = 16;
            cloudMsg.is_dense = true; // all data points are valid
            cloudMsg.is_bigendian = false;
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#ifdef SPEED_OPT
            cloud->points.reserve(dep.rows * dep.cols);
#endif

            for (int row = 0; row < dep.rows; row++)
            {
                rtotal++;
                for (int col = 0; col < dep.cols; col++)
                {
                    if (row == 0) ctotal++;
                    float z = dep.at<ushort>(row, col);
                    float x3d = 0, y3d = 0;
                    bool bzero = false;
                    int X = col,Y=row;

                    if (z == 0)
                    {
                        if (!bOrder)
                            continue;
                        else
                            bzero = true;
                    }
                    cv::Vec3b v3(255, 255, 255);
                    if (!bzero)
                    {
                        // if(mOutputFormat == 4)
                        {
                            x3d = z*((float)col - x0)*fx;
                            y3d = z*((float)row - y0)*fy;
                        }
                        if(mRegistrationChannel == -1)
                        {
                          // cv::Point2f uv = OptData::Depth2Web(col * cinter, row * rinter, z, mOD, x3d, y3d);
                            cv::Point3f uv = Inuchip::Depth2Web_Pt(col*cinter, row*rinter, z, mOD, x3d, y3d);

                          X = static_cast<int>(uv.x);
                          Y = static_cast<int>(uv.y);
                        }

                        if ((X >= 0 && X < imgWeb.cols) && (Y >= 0 && Y < imgWeb.rows))
                        {
                            if (imgWeb.type() == CV_8UC4)
                            {
                                cv::Vec4b v4 = imgWeb.at<cv::Vec4b>(Y, X);
                                v3[0] = v4[0]; v3[1] = v4[1]; v3[2] = v4[2];
                            }
                            else
                                v3 = imgWeb.at<cv::Vec3b>(Y, X);
                        }
                        else
                        {
                            if (!bOrder)
                                continue;
                            else
                                bzero = true;
                        }
                    }
                    if (bzero)
                    {
                        x3d = 0;
                        y3d = 0;
                        z = 0;
                    }
                    pcl::PointXYZRGB pt(v3[0], v3[1], v3[2]);
                    pt.x = x3d * fScale;
                    pt.y = y3d * fScale;
                    pt.z = z * fScale;

                    cloud->points.emplace_back(pt);
                }

            }

            pcl::toROSMsg(*cloud, cloudMsg);
            cloudMsg.point_step = 16;
            cloudMsg.row_step = 16;
            cloudMsg.is_dense = true; // all data points are valid
            cloudMsg.is_bigendian = false;
        }

        cloudMsg.header.frame_id = "camera_link" + mDepthFrameID;
        cloudMsg.header.stamp = rclcpp::Time(curTs / 1000000000ll, curTs % 1000000000ll);

        if (bOrder)
        {
            cloudMsg.width = ctotal;
            cloudMsg.height = rtotal;
        }

        publisher_pcl->publish(cloudMsg);

    }

    void CRosDepthPublisher::FrameCallback(std::shared_ptr<InuDev::CDepthStream> iStream, std::shared_ptr<const InuDev::CImageFrame> iFrame, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!CheckFrame(iFrame, iError))
        {
            return;
        }

        InuDev::EImageFormat format = (InuDev::EImageFormat)iFrame->Format();

        if (format != InuDev::EImageFormat::eDepth)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": Depth Frame's format (" << format << ") is not eDepth");
            return;
        }

        int width = iFrame->Width();
        int height = iFrame->Height();
        int bpp = iFrame->BytesPerPixel();
        const InuDev::byte* data = iFrame->GetData();

        cv::Mat frame(height, width, CV_16UC1, (char*)data);

        int rinter = 1;
        int cinter = 1;
        if (mSimplify2)
        {
            rinter = 2;
            cinter = 2;
        }

        if ((rinter > 1 && cinter > 1) && width > 10 && height > 10)
        {
            // cv::pyrDown(frame,frame, cv::Size(int(width/cinter),int(height/rinter)));
            cv::resize(frame, frame, cv::Size(int(width / cinter), int(height / rinter)), 0, 0, cv::INTER_NEAREST);
        }

        //first temporal
        if (mTemporalFilterROS)
        {
            Inuchip::TemporalFilter tf(mTemporalBufferFrames,
                mTemporalProcessAllPixel,
                mTemporalThreads);
            //TemporalFilter tf;
            frame = tf.process_frame(frame);
        }

        // if (mTemporal2FilterROS)
        if(sensor->dynamicParams.find("Temporal2_Filter_ROS")->second.as_bool())
        {
          frame = tf2.process_frame(frame,(sensor->dynamicParams.find("Temporal2_Filter_alpha")->second.as_int())*0.01f,//mTemporal2FilterAlpha
                                            sensor->dynamicParams.find("Temporal2_Filter_delta")->second.as_int());//mTemporal2FilterDelta
        }

        if (mPassthroughFilterROS)
        {
            frame = sf.process_passthrough(frame,mPTzmin,mPTzmax
                                                ,mPTxmin,mPTxmax
                                                ,mPTymin,mPTymax);
        }

        // if (mOutlierRemoveFilterROS)
        if(sensor->dynamicParams.find("Outlier_Remove_Fitler_ROS")->second.as_bool())
        {
            frame = sf.process_outlier_remove(frame, (sensor->dynamicParams.find("OR_percent")->second.as_int())*0.001f// mORpercent * 0.01f
                                                   ,sensor->dynamicParams.find("OR_min_dist")->second.as_int()         // mORminDist
                                                   ,sensor->dynamicParams.find("OR_max_remove")->second.as_int());
        }

        //add HoleFile
        if (mHoleFillFilterROS)
        {
            frame = sf.process_hole_fill(frame, mHFmaxRadius);
        }

        // if (mEdgePreserveFilterROS)
        if(sensor->dynamicParams.find("Edge_Preserve_ROS")->second.as_bool())
        {
          // frame = sf.process_edge_preserve(frame,mEdgePreserveFilterAlpha*0.01f,mEdgePreserveFilterDelta);
          frame = sf.process_edge_preserve(frame,(sensor->dynamicParams.find("Edge_Preserve_alpha")->second.as_int())*0.01f,sensor->dynamicParams.find("Edge_Preserve_delta")->second.as_int());
        }

        if (mDept2PcSupportted)
        {
          pub_pcl(frame, iFrame->Timestamp, mPCordered, rinter, cinter);
        }
        if (mShowColor)
            frame = Short2RGB(frame);

        std_msgs::msg::Header frameHeader;
        frameHeader.stamp = mCi.header.stamp = rclcpp::Time(iFrame->Timestamp / 1000000000ll, iFrame->Timestamp % 1000000000ll);
        //frameHeader.stamp = mCi.header.stamp = rclcpp::Clock().now();
        frameHeader.frame_id = "camera_link" + mDepthFrameID;

        if (mShowColor == false) {
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(frameHeader, "mono16", frame).toImageMsg();
            publisher.publish(msg);
        }
        else {
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(frameHeader, "bgr8", frame).toImageMsg();
            publisher.publish(msg);
        }
    }

    InuDev::CInuError CRosDepthPublisher::RegisterCallback()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CDepthStream::CallbackFunction callback = std::bind(&CRosDepthPublisher::FrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        return std::static_pointer_cast<InuDev::CDepthStream>(stream)->Register(callback);
    }

    InuDev::CInuError CRosDepthPublisher::UnregisterCallback()
    {
        return std::static_pointer_cast<InuDev::CDepthStream>(stream)->Register(nullptr);
    }

    int CRosDepthPublisher::GetNumSubscribers()
    {
        publisherPrev = publisherCurr;
        publisherCurr = publisher.getNumSubscribers();
        if (mDept2PcSupportted)
        {
            publisherCurr += publisher_pcl->get_subscription_count();
        }

        return publisherCurr;
    }

    InuDev::CInuError CRosDepthPublisher::InitStream()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        InuDev::CInuError err(0);

        int tmpPostProcessing = 0;

#ifndef _GE428_
        if(mRegistrationChannel != -1)
        {
          if(!(tmpPostProcessing & InuDev::CDepthStream::EPostProcessing::eRegistration))
          {
            tmpPostProcessing += InuDev::CDepthStream::EPostProcessing::eRegistration;
          }
        }
#endif

        if (mHoleFillFilterSDK)
        {
            if (!(tmpPostProcessing & InuDev::CDepthStream::EPostProcessing::eHoleFill))
            {
                tmpPostProcessing += InuDev::CDepthStream::EPostProcessing::eHoleFill;
            }
        }

        if (mOutlierRemoveFilterSDK)
        {
            if (!(tmpPostProcessing & InuDev::CDepthStream::EPostProcessing::eOutlierRemove))
            {
                tmpPostProcessing += InuDev::CDepthStream::EPostProcessing::eOutlierRemove;
            }
        }

        if (mTemporalFilterSDK)
        {
            if (!(tmpPostProcessing & InuDev::CDepthStream::EPostProcessing::eStaticTemporal))
            {
                tmpPostProcessing += InuDev::CDepthStream::EPostProcessing::eStaticTemporal;
            }
        }


        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " tmpPostProcessing: " << tmpPostProcessing);

        if (mParamsChannelID == -1 && mOutputFormat == 0 && tmpPostProcessing == 0)
        {
            InuDev::CInuError err = std::static_pointer_cast<InuDev::CDepthStream>(stream)->Init();
            RCLCPP_INFO_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName() << " Error : " << int(err) );
        }
        else
        {
            #ifdef _GE428_
                err = std::static_pointer_cast<InuDev::CDepthStreamExt>(stream)->Init((InuDev::CDepthStream::EOutputFormat)mOutputFormat, (unsigned int)tmpPostProcessing);
            #else
            if (mRegistrationChannel != -1)
            {
                err = std::static_pointer_cast<InuDev::CDepthStreamExt>(stream)->Init((InuDev::CDepthStream::EOutputFormat)mOutputFormat, (unsigned int)tmpPostProcessing,(unsigned int)mRegistrationChannel);
            }
            else
            {
                err = std::static_pointer_cast<InuDev::CDepthStreamExt>(stream)->Init((InuDev::CDepthStream::EOutputFormat)mOutputFormat, (unsigned int)tmpPostProcessing);
            }
            #endif


            if (mHoleFillFilterSDK)
            {
                RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " Activate hole fill from SDK with max radius: " << mHFmaxRadius);
                InuDev::CHoleFillFilterParams params;
                params.MaxRadius = mHFmaxRadius;
                err = std::static_pointer_cast<InuDev::CDepthStreamExt>(stream)->SetHoleFillParams(params);
            }

            if (mOutlierRemoveFilterSDK)
            {
                RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " Activate outlier remove from SDK with precent: " << mORpercent);
                InuDev::COutlierRemoveParams params;
                params.MaxPercent = mORpercent * 0.01f;
                params.MinDist = mORminDist;
                err = std::static_pointer_cast<InuDev::CDepthStreamExt>(stream)->SetOutlierRemoveParams(params);
            }

            if (mTemporalFilterSDK)
            {
                RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " Activate static temporal filter from SDK with num threads: " << mTemporalThreads);
                InuDev::CStaticTemporalFilterParams params;
                params.FilterLength = mTemporalBufferFrames;
                params.BAll = mTemporalProcessAllPixel;
                params.ThreadNum = mTemporalThreads;
                err = std::static_pointer_cast<InuDev::CDepthStreamExt>(stream)->SetStaticTemporalFilterParams(params);
            }
        }

        if (mPassthroughFilterSDK)
        {
            InuDev::CDisparityParams params;
            err = std::static_pointer_cast<InuDev::CDepthStreamExt>(stream)->GetDisparityParams(params);
            if (err != InuDev::EErrorCode::eOK)
            {
                RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " GetDisparityParams return an error: " << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
                return err;
            }
            if (mPTzmax != -1)
            {
                params.MaxDistance = mPTzmax;
            }

            if (mPTzmin != -1)
            {
                params.MinDistance = mPTzmin;
            }


            if (mPTzmin != -1 || mPTzmax != -1)
            {
                RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " Activate static pass through filter from SDK with mPTzmax : " << params.MaxDistance);
                err = std::static_pointer_cast<InuDev::CDepthStreamExt>(stream)->SetDisparityParams(params);
            }
        }

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        return InuDev::EErrorCode::eOK;
    }

    InuDev::CInuError CRosDepthPublisher::StartStream()
    {
        if (!sensor || !sensor->connected())
            return InuDev::EErrorCode::eStateError;

        // Get dynamic params
        GetParams();

        RCLCPP_WARN_STREAM(logger, "Starting Depth with channel: " << mParamsChannelID);

        if (mParamsChannelID == -1)
        {
            stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateDepthStreamExt());
        }
        else
        {
            stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreateDepthStreamExt(mParamsChannelID));
        }

        if (!stream)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": failed creating " << getName());
        }

        InuDev::EErrorCode err = CRosPublisher::StartStream();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        settingConfidence(mDepthConfidence,true);

        uint32_t channelID;
        err = stream->GetChannel(channelID);

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        mChannelID = int(channelID);

        auto calibrationData = std::make_shared<InuDev::CCalibrationData>();
        err = sensor->getSensor()->GetCalibrationData(*calibrationData.get(), mChannelID);

        mCalibrationData = calibrationData; // shared_ptr assignemt
        LoadOD();

        mSensors = sensor->GetSensors(mChannelID);

        return InuDev::EErrorCode::eOK;
    }

    int CRosDepthPublisher::settingConfidence(int newconfidence, bool direct)
    {
      if(newconfidence != mDepthConfidence || direct)
        {
            mDepthConfidence = newconfidence;
            sensor->getSensor()->SetSensorRegister(0x08010ae0, (mDepthConfidence*2+1)<<24, 1, InuDev::eLeft);
            sensor->getSensor()->SetSensorRegister(0x080109e0, (mDepthConfidence*2+1)<<24, 1, InuDev::eRight);
            // NODELET_ERROR_STREAM(__INUROS_FUNCTION_NAME__ << ": " << getName() << " mDepthconfidence = " << mDepthconfidence);
        }
        return 0;

    }

    void CRosDepthPublisher::GetParams()
    {
        // if (sensor->dynamicParams.find("DepthChannel") != sensor->dynamicParams.end())
        {
            mParamsChannelID = sensor->dynamicParams.find("DepthChannel")->second.as_int();
        }
        // if (sensor->dynamicParams.find("DepthOutputFormat") != sensor->dynamicParams.end())
        {
            mOutputFormat = sensor->dynamicParams.find("DepthOutputFormat")->second.as_int();
        }
        // if (sensor->dynamicParams.find("DepthRegistarionChannel") != sensor->dynamicParams.end())
        {
            mRegistrationChannel = sensor->dynamicParams.find("DepthRegistarionChannel")->second.as_int();
        }
        // if (sensor->dynamicParams.find("Show_Color") != sensor->dynamicParams.end())
        {
            mShowColor = sensor->dynamicParams.find("Show_Color")->second.as_bool();
        }
        // if (sensor->dynamicParams.find("ServiceID") != sensor->dynamicParams.end())
        {
            mDepthFrameID = sensor->dynamicParams.find("ServiceID")->second.as_string();
        }
        // if (sensor->dynamicParams.find("Simplify_2") != sensor->dynamicParams.end())
        {
            mSimplify2 = sensor->dynamicParams.find("Simplify_2")->second.as_bool();
        }

        // Temporal_Filter
        // if (sensor->dynamicParams.find("Temporal_Filter_ROS") != sensor->dynamicParams.end())
        {
            mTemporalFilterROS = sensor->dynamicParams.find("Temporal_Filter_ROS")->second.as_bool();
        }

        // Temporal2_Filter
        // if (sensor->dynamicParams.find("Temporal2_Filter_ROS") != sensor->dynamicParams.end())
        {
            mTemporal2FilterROS = sensor->dynamicParams.find("Temporal2_Filter_ROS")->second.as_bool();
        }

        // Temporal2_alpha
        // if (sensor->dynamicParams.find("Temporal2_Filter_alpha") != sensor->dynamicParams.end())
        {
            mTemporal2FilterAlpha = sensor->dynamicParams.find("Temporal2_Filter_alpha")->second.as_int();
        }

        // Temporal2_delta
        // if (sensor->dynamicParams.find("Temporal2_Filter_delta") != sensor->dynamicParams.end())
        {
            mTemporal2FilterDelta = sensor->dynamicParams.find("Temporal2_Filter_delta")->second.as_int();
        }

        // Edge_Preserve_Filter
        // if (sensor->dynamicParams.find("Edge_Preserve_ROS") != sensor->dynamicParams.end())
        {
            mEdgePreserveFilterROS = sensor->dynamicParams.find("Edge_Preserve_ROS")->second.as_bool();
        }

        // Edge_Preserve_alpha
        // if (sensor->dynamicParams.find("Edge_Preserve_alpha") != sensor->dynamicParams.end())
        {
            mEdgePreserveFilterAlpha = sensor->dynamicParams.find("Edge_Preserve_alpha")->second.as_int();
        }

        // Edge_Preserve_delta
        // if (sensor->dynamicParams.find("Edge_Preserve_delta") != sensor->dynamicParams.end())
        {
            mEdgePreserveFilterDelta = sensor->dynamicParams.find("Edge_Preserve_delta")->second.as_int();
        }

        // if (sensor->dynamicParams.find("Temporal_Filter_SDK") != sensor->dynamicParams.end())
        {
            mTemporalFilterSDK = sensor->dynamicParams.find("Temporal_Filter_SDK")->second.as_bool();
        }

        // Passthrough_Filter
        // if (sensor->dynamicParams.find("Passthrough_Filter_ROS") != sensor->dynamicParams.end())
        {
            mPassthroughFilterROS = sensor->dynamicParams.find("Passthrough_Filter_ROS")->second.as_bool();
        }

        // if (sensor->dynamicParams.find("Passthrough_Filter_SDK") != sensor->dynamicParams.end())
        {
            mPassthroughFilterSDK = sensor->dynamicParams.find("Passthrough_Filter_SDK")->second.as_bool();
        }

        // Outlier removal
        // if (sensor->dynamicParams.find("Outlier_Remove_Fitler_ROS") != sensor->dynamicParams.end())
        {
            mOutlierRemoveFilterROS = sensor->dynamicParams.find("Outlier_Remove_Fitler_ROS")->second.as_bool();
        }

        // if (sensor->dynamicParams.find("Outlier_Remove_Fitler_SDK") != sensor->dynamicParams.end())
        {
            mOutlierRemoveFilterSDK = sensor->dynamicParams.find("Outlier_Remove_Fitler_SDK")->second.as_bool();
        }

        // Hole Fill Fitler
        // if (sensor->dynamicParams.find("Hole_Fill_Filter_ROS") != sensor->dynamicParams.end())
        {
            mHoleFillFilterROS = sensor->dynamicParams.find("Hole_Fill_Filter_ROS")->second.as_bool();
        }

        // if (sensor->dynamicParams.find("Hole_Fill_Filter_SDK") != sensor->dynamicParams.end())
        {
            mHoleFillFilterSDK = sensor->dynamicParams.find("Hole_Fill_Filter_SDK")->second.as_bool();
        }

        // Temporal_BufferFrames
        // if (sensor->dynamicParams.find("Temporal_BufferFrames") != sensor->dynamicParams.end())
        {
            mTemporalBufferFrames = sensor->dynamicParams.find("Temporal_BufferFrames")->second.as_int();
        }
        // Temporal_ProcessAllPixel
        // if (sensor->dynamicParams.find("Temporal_ProcessAllPixel") != sensor->dynamicParams.end())
        {
            mTemporalProcessAllPixel = sensor->dynamicParams.find("Temporal_ProcessAllPixel")->second.as_bool();
        }

        // Temporal_Threads
        // if (sensor->dynamicParams.find("Temporal_Threads") != sensor->dynamicParams.end())
        {
            mTemporalThreads = sensor->dynamicParams.find("Temporal_Threads")->second.as_int();
        }

        //PT_zmin
        // if (sensor->dynamicParams.find("PT_zmin") != sensor->dynamicParams.end())
        {
            mPTzmin = sensor->dynamicParams.find("PT_zmin")->second.as_int();
        }

        //PT_zmax
        // if (sensor->dynamicParams.find("PT_zmax") != sensor->dynamicParams.end())
        {
            mPTzmax = sensor->dynamicParams.find("PT_zmax")->second.as_int();
        }

        //PT_xmin
        // if (sensor->dynamicParams.find("PT_xmin") != sensor->dynamicParams.end())
        {
            mPTxmin = sensor->dynamicParams.find("PT_xmin")->second.as_int();
        }

        //PT_xmax
        // if (sensor->dynamicParams.find("PT_xmax") != sensor->dynamicParams.end())
        {
            mPTxmax = sensor->dynamicParams.find("PT_xmax")->second.as_int();
        }

        //PT_ymin
        // if (sensor->dynamicParams.find("PT_ymin") != sensor->dynamicParams.end())
        {
            mPTymin = sensor->dynamicParams.find("PT_ymin")->second.as_int();
        }

        //PT_ymax
        // if (sensor->dynamicParams.find("PT_ymax") != sensor->dynamicParams.end())
        {
            mPTymax = sensor->dynamicParams.find("PT_ymax")->second.as_int();
        }

        // OR_percent
        // if (sensor->dynamicParams.find("OR_percent") != sensor->dynamicParams.end())
        {
            mORpercent = sensor->dynamicParams.find("OR_percent")->second.as_int();
        }
        // OR_min_dist
        // if (sensor->dynamicParams.find("OR_min_dist") != sensor->dynamicParams.end())
        {
            mORminDist = sensor->dynamicParams.find("OR_min_dist")->second.as_int();
        }

        {
            mORmaxRemove = sensor->dynamicParams.find("OR_max_remove")->second.as_int();
        }

        // HF_maxradius
        // if (sensor->dynamicParams.find("HF_maxradius") != sensor->dynamicParams.end())
        {
            mHFmaxRadius = sensor->dynamicParams.find("HF_maxradius")->second.as_int();
        }
        // PC_scale
        // if (sensor->dynamicParams.find("PC_scale") != sensor->dynamicParams.end())
        {
            mPCscale = sensor->dynamicParams.find("PC_scale")->second.as_int();
        }
        // PC_ordered
        // if (sensor->dynamicParams.find("PC_ordered") != sensor->dynamicParams.end())
        {
            mPCordered = sensor->dynamicParams.find("PC_ordered")->second.as_bool();
        }

        // activate depth 2 point cloud
        // if (sensor->dynamicParams.find("Dept2PcSupportted") != sensor->dynamicParams.end())
        {
            mDept2PcSupportted = sensor->dynamicParams.find("Dept2PcSupportted")->second.as_bool();
        }

        {
            mDepthConfidence = sensor->dynamicParams.find("Confidence")->second.as_int();
        }
    }

}
