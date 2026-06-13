/*
 * File - ros_pointcloud_publisher.cpp
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

#include <fstream>
#include <iostream>

#if POINTCLOUD_DUMP_FRAME
#include <experimental/filesystem>
#endif

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/opencv.hpp>

/**
 * \endcond
 */

#include "ros_pointcloud_publisher.h"

#include "inuros2_node_factory.h"

/**
 * \file ros_pointcloud_publisher.cpp
 *
 * \brief CRosPointcloudPublisher
 */

namespace __INUROS__NAMESPACE__
{
    std::vector<const char *> CRosPointcloudPublisher::fieldNames = { "x", "y", "z" };

    CRosPointcloudPublisher::CRosPointcloudPublisher(rclcpp::Node::SharedPtr& _node, CRosSensor* _rosSensor, std::string iTopicName)
        : CRosPublisher(_node, "pointcloud", _rosSensor)
    {
        const std::string serviceId = _rosSensor->GetSensorId();
        if (!serviceId.empty())
        {
            iTopicName = "service" + serviceId + "/" + iTopicName;
        }
        publisher = _node->create_publisher<sensor_msgs::msg::PointCloud2>(iTopicName, 50);

        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
    }

    void CRosPointcloudPublisher::FrameCallback(std::shared_ptr<InuDev::CPointCloudStream> iStream, std::shared_ptr<const InuDev::CPointCloudFrame> iFrame, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
        if (!CheckFrame(iFrame, iError))
        {
            return;
        }

        InuDev::CPointCloudFrame::EFormat format = (InuDev::CPointCloudFrame::EFormat)iFrame->GetFormat();
	//RCLCPP_DEBUG_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName()<< " format " <<format<<" mConvertPointCloudShortToFloat: " << mConvertPointCloudShortToFloat);
		
        if (format != InuDev::CPointCloudFrame::EFormat::e3DPoints && format != InuDev::CPointCloudFrame::EFormat::e3DShortPoints)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": Frame's format (" << format << ") is not ePointCloud");
            return;
        }

        int numPoints = iFrame->GetNumOfPoints();

        //fillCameraInfoFromVideo(iFrame->FrameIndex, width, height, true);

        sensor_msgs::msg::PointCloud2 cloudMsg;

        cloudMsg.header.stamp.sec = iFrame->Timestamp / 1000000000ll;

        cloudMsg.header.frame_id = "map";

        // "A PointCloud2 can be organized or unorganized. Unorganized means that height == 1 (and
        // therefore width == number of points, since height * width must equal number of points).
        // Organized means something like height == 240, width == 320, and that's usual for point
        // clouds that come from a 3D camera. The useful thing about this is that there is a
        // one-to-one mapping between a pixel and a point, which allows more efficient algorithms."
        //
        // https://answers.ros.org/question/234455/pointcloud2-and-pointfield/

        cloudMsg.height = 1; // unorganized
        cloudMsg.width = numPoints;

        cloudMsg.fields.resize(fieldNames.size());
	
                                
        if(format == InuDev::CPointCloudFrame::EFormat::e3DShortPoints && !mConvertPointCloudShortToFloat)
        {
        	for (std::vector<char *>::size_type fldIndex=0; fldIndex < fieldNames.size(); fldIndex++)
        	{
        	    auto &fld(cloudMsg.fields[fldIndex]);
	            fld.name     = fieldNames[fldIndex];
	            fld.offset   = fldIndex*sizeof(short);
	            fld.datatype = sensor_msgs::msg::PointField::INT16;
	            fld.count    = 1;
	        }
	        
	        cloudMsg.point_step = 3 * sizeof(short);
        }
        else
        {
        	for (std::vector<char *>::size_type fldIndex=0; fldIndex < fieldNames.size(); fldIndex++)
        	{
            	    auto &fld(cloudMsg.fields[fldIndex]);
	            fld.name     = fieldNames[fldIndex];
          	    fld.offset   = fldIndex*sizeof(float);
	            fld.datatype = sensor_msgs::msg::PointField::FLOAT32;
        	    fld.count    = 1;
        	}
        	
        	cloudMsg.point_step = 3 * sizeof(float);
        }
        
        
        cloudMsg.row_step = cloudMsg.point_step;
        

        // "A PointCloud2 can be dense or non-dense. Dense means that there are no invalid points,
        // non-dense means the opposite. An invalid point is one that has all x/y/z values set to
        // std::numeric_limits<float>::quiet_NaN()."

        cloudMsg.is_dense = true; // all data points are valid
        cloudMsg.is_bigendian = false;
	
	
	if(format == InuDev::CPointCloudFrame::EFormat::e3DShortPoints && mConvertPointCloudShortToFloat)
	{
		cloudMsg.data.resize(cloudMsg.point_step * numPoints);
		
		
		short* dataArr = (short*)iFrame->GetData();
		float* tmpFloatArr = (float*)&(cloudMsg.data[0]); 
		
		for(int i=0;i<numPoints*3;i+=3)
		{
			tmpFloatArr[i] =   (float)dataArr[i]; 
			tmpFloatArr[i+1] = (float)dataArr[i+1];
			tmpFloatArr[i+2] = (float)dataArr[i+2];		
		}
	}
	else
	{
		cloudMsg.data.resize(cloudMsg.point_step * numPoints);
		std::memcpy(cloudMsg.data.data(), iFrame->GetData(), cloudMsg.point_step * numPoints);
	}

#if POINTCLOUD_DUMP_FRAME

        if (CInuDevRosNodelet::mLogPointcloud)
        {
            // print using 'od --format=f4 --width=12 /tmp/aaaaa.bin'

            std::fstream outFile;

            std::experimental::filesystem::path fileNameTemp(POINTCLOUD_DUMP_FILENAME ".bin.tmp");
            outFile.open(std::string(fileNameTemp), std::ios::out);

            outFile.write(reinterpret_cast<const char *>(iFrame->GetData()), numPoints*3*sizeof(float));

            outFile.close();

            std::experimental::filesystem::path fileName(POINTCLOUD_DUMP_FILENAME "_");
            fileName += std::to_string(iFrame->FrameIndex);
            fileName += ".bin";

            RCLCPP_INFO_STREAM(logger,"filename=" << std::string(fileName));

            std::experimental::filesystem::rename(fileNameTemp, fileName);
        }

#endif

        publisher->publish(cloudMsg);

    }

    void CRosPointcloudPublisher::RegisteredFrameCallback(std::shared_ptr<InuDev::CPointCloudStream> iStream, std::shared_ptr<const InuDev::CPointCloudFrame> iFrame, InuDev::CInuError iError)
    {
        RCLCPP_DEBUG_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": " << getName());

        if (!CheckFrame(iFrame, iError))
        {
            return;
        }

        InuDev::CPointCloudFrame::EFormat format = (InuDev::CPointCloudFrame::EFormat)iFrame->GetFormat();

        if (format != InuDev::CPointCloudFrame::EFormat::e3DPoints && format != InuDev::CPointCloudFrame::EFormat::e3DPointsRGB)
        {
            RCLCPP_ERROR_STREAM(logger,__INUROS_FUNCTION_NAME__ << ": Frame's format (" << format << ") is not ePointCloud");
            return;
        }

        int numPoints = iFrame->GetNumOfPoints();

        //fillCameraInfoFromVideo(iFrame->FrameIndex, width, height, true);

        sensor_msgs::msg::PointCloud2 cloudMsg;

        sensor_msgs::PointCloud2Modifier modifier(cloudMsg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(iFrame->BufferSize());
        cloudMsg.header.stamp.sec  = iFrame->Timestamp / 1000000000ll;

        cloudMsg.header.frame_id = "map";

        // "A PointCloud2 can be organized or unorganized. Unorganized means that height == 1 (and
        // therefore width == number of points, since height * width must equal number of points).
        // Organized means something like height == 240, width == 320, and that's usual for point
        // clouds that come from a 3D camera. The useful thing about this is that there is a
        // one-to-one mapping between a pixel and a point, which allows more efficient algorithms."
        //
        // https://answers.ros.org/question/234455/pointcloud2-and-pointfield/

        cloudMsg.height = 1; // unorganized
        cloudMsg.width = numPoints;
        bool isWithRGB = format == InuDev::CPointCloudFrame::EFormat::e3DPointsRGB;

        if(isWithRGB)
        {
            fieldNames = { "x", "y", "z", "r", "g", "b"};
        }
        cloudMsg.fields.resize(fieldNames.size());

        for (std::vector<char *>::size_type fldIndex=0; fldIndex < fieldNames.size(); fldIndex++)
        {
            auto &fld(cloudMsg.fields[fldIndex]);

            fld.name     = fieldNames[fldIndex];
            fld.offset   = fldIndex*sizeof(float);
            fld.datatype = sensor_msgs::msg::PointField::FLOAT32;
            fld.count    = 1;
        }

        // "A PointCloud2 can be dense or non-dense. Dense means that there are no invalid points,
        // non-dense means the opposite. An invalid point is one that has all x/y/z values set to
        // std::numeric_limits<float>::quiet_NaN()."

        cloudMsg.is_dense = true; // all data points are valid
        cloudMsg.is_bigendian = false;
        cloudMsg.point_step = addPointField(cloudMsg, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32, cloudMsg.point_step);

        cloudMsg.row_step = cloudMsg.width * cloudMsg.point_step;
        cloudMsg.data.resize(cloudMsg.height * cloudMsg.row_step);

        const InuDev::CPointCloudFrame::C3DRGBPixel *xyzrgbFrame = iFrame->Get3DRGBData();

        for(int i = 0 ; i < numPoints - 1; i++)
        {
            const float values[] = { xyzrgbFrame[i].X(), xyzrgbFrame[i].Y(), xyzrgbFrame[i].Z(), (float)xyzrgbFrame[i].B, (float)xyzrgbFrame[i].G, (float)xyzrgbFrame[i].R};
            uint32_t index = i * cloudMsg.point_step + sizeof(float);
            std::memcpy(&cloudMsg.data[index], &values[0], sizeof(float));
            index += sizeof(float);
            std::memcpy(&cloudMsg.data[index], &values[1], sizeof(float));
            index += 2 * sizeof(float);
            std::memcpy(&cloudMsg.data[index], &values[2], sizeof(float));
            index += 3 * sizeof(float);
            std::memcpy(&cloudMsg.data[index], &values[3], sizeof(float));
            index += 4 * sizeof(float);
            std::memcpy(&cloudMsg.data[index], &values[4], sizeof(float));
            index += 5 * sizeof(float);
            std::memcpy(&cloudMsg.data[index], &values[5], sizeof(float));
        }

#if POINTCLOUD_DUMP_FRAME

        if (CInuDevRosNodelet::mLogPointcloud)
        {
            // print using 'od --format=f4 --width=12 /tmp/aaaaa.bin'

            std::fstream outFile;

            std::experimental::filesystem::path fileNameTemp(POINTCLOUD_DUMP_FILENAME ".bin.tmp");
            outFile.open(std::string(fileNameTemp), std::ios::out);

            outFile.write(reinterpret_cast<const char *>(iFrame->GetData()), numPoints*(isWithRGB?7:3)*sizeof(float));

            outFile.close();

            std::experimental::filesystem::path fileName(POINTCLOUD_DUMP_FILENAME "_");
            fileName += std::to_string(iFrame->FrameIndex);
            fileName += ".bin";

            RCLCPP_INFO_STREAM(logger, "filename=" << std::string(fileName));

            std::experimental::filesystem::rename(fileNameTemp, fileName);
        }

#endif

        publisher->publish(cloudMsg);

    }

    InuDev::CInuError CRosPointcloudPublisher::RegisterCallback()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());
        InuDev::CPointCloudStream::CallbackFunction callback;
        if(mPointCloudFormat)
        {
            callback = std::bind(&CRosPointcloudPublisher::RegisteredFrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        }
        else
        {
            callback = std::bind(&CRosPointcloudPublisher::FrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        }

        return std::static_pointer_cast<InuDev::CPointCloudStream>(stream)->Register(callback);
    }

    InuDev::CInuError CRosPointcloudPublisher::UnregisterCallback()
    {
        return std::static_pointer_cast<InuDev::CPointCloudStream>(stream)->Register(nullptr);
    }

    int CRosPointcloudPublisher::GetNumSubscribers()
    {
        publisherPrev = publisherCurr;
        publisherCurr = publisher->get_subscription_count();

        return publisherCurr;
    }

    InuDev::CInuError CRosPointcloudPublisher::InitStream()
    {
        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName());

        if (sensor->dynamicParams.find("PointCloudFormat") != sensor->dynamicParams.end())
        {
            mPointCloudFormat = sensor->dynamicParams.find("PointCloudFormat")->second.as_int();
        }
        if (sensor->dynamicParams.find("PointCloudRegistrationChannel") != sensor->dynamicParams.end())
        {
            mRGBRegistrationChannel = sensor->dynamicParams.find("PointCloudRegistrationChannel")->second.as_int();
        }
        if (sensor->dynamicParams.find("ConvertPointCloudShortToFloat") != sensor->dynamicParams.end())
        {
            mConvertPointCloudShortToFloat = sensor->dynamicParams.find("ConvertPointCloudShortToFloat")->second.as_bool();
        }

        RCLCPP_INFO_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " mPointCloudFormat:  " << mPointCloudFormat);
        InuDev::CInuError err = InuDev::EErrorCode::eOK;
        if(mPointCloudFormat)
        {
            err = std::static_pointer_cast<InuDev::CPointCloudStream>(stream)->Init((InuDev::CBasePointCloudStream::ERegistationType)mPointCloudFormat, mRGBRegistrationChannel);
        }
        else
        {
            err = std::static_pointer_cast<InuDev::CPointCloudStream>(stream)->Init();
        }

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        return InuDev::EErrorCode::eOK;
    }

    InuDev::CInuError CRosPointcloudPublisher::StartStream()
    {
        if (!sensor || !sensor->connected())
            return InuDev::EErrorCode::eStateError;

        stream = std::static_pointer_cast<InuDev::CBaseStream>(sensor->getSensor()->CreatePointCloudStream());

        if (!stream)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": failed creating " << getName());
        }

        float pcScale = 1000.0F;
        if (sensor->dynamicParams.find("PC_scale") != sensor->dynamicParams.end())
        {
            pcScale = (float)sensor->dynamicParams.find("PC_scale")->second.as_int();
        }

        mPointCloudScale = 1.0F;
        if (pcScale > 1.0F)
            mPointCloudScale = 1.0F / pcScale;

        InuDev::EErrorCode err = CRosPublisher::StartStream();

        if (err != InuDev::EErrorCode::eOK)
        {
            RCLCPP_ERROR_STREAM(logger, __INUROS_FUNCTION_NAME__ << ": " << getName() << " 0x" << std::hex << std::setw(8) << std::setfill('0') << int(err));
            return err;
        }

        return InuDev::EErrorCode::eOK;
    }

}
