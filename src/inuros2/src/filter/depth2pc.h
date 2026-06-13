#ifndef __DEPTH2PC_HPP__
#define __DEPTH2PC_HPP__

#include <fstream>
#include <iostream>
#include <memory>

#include <opencv2/opencv.hpp>
namespace Inuchip
{
	class COptData {

	public:
		/*********** Internal Right Camera Parameters ***************/
		float focalDepth[2];
		float centerDepth[2];

		float inversefocalDepth[2];

		/*********** Internal Web Camera Parameters ***************/
		float focalWeb[2];
		float centerWeb[2];
		float uvWeb[2];

		/****************Web Distortion parameters****************************/
		float KdWeb[5];
		cv::Size sizeWeb;

		/*********** ROTATION Right to Web ***************/
		cv::Mat rotRightWeb;
		cv::Point3f rotAngle;

		float WebcamTranslate[3];

		float baselineDepth;

		float r[14];
		int rfix[23];

	public:
		COptData();
		void DumpYaml(const std::string& fname);
		void LoadYaml(const std::string& fname);

		//cv::Mat buildRotation3D(cv::Point3f& angle);
		void Init();
		void calcR();
		void calcR_fix();
	};
	cv::Point3f Depth2Web_Pt(float Ix, float Iy, float Z, const COptData& OD, float& x3D, float& y3D);
	cv::Point3f Depth2Web_Pt_Dist(float Ix, float Iy, float Z, const COptData& OD, float& x3D, float& y3D);
	std::shared_ptr<cv::Mat> Depth2Color(const cv::Mat& imgDepth, COptData& mOD, bool closeHole = true, int zMin = 100, int zMax = 8000, float depthscale_in = 1.0, float depthscale_out = 1.0);
	std::shared_ptr<cv::Mat> Depth2Colorfix(const cv::Mat& imgDepth, COptData& mOD, cv::Mat& imgVal, cv::Mat& imgIdx, bool closeHole = true, int zMin = 100, int zMax = 8000, float depthscale_in = 1.0, float depthscale_out = 1.0);
	std::shared_ptr<cv::Mat> Color2Depth(const cv::Mat& imgDepth, const cv::Mat& imgWeb, COptData& mOD, int zMin = 100, int zMax = 8000);

	void Depth2Ply(const cv::Mat& imgDep, const cv::Mat& imgRgb, const COptData& mOD, std::string& sBuf, float depthscale = 1.0F);

	std::shared_ptr<cv::Mat> Depth2Color_rs(const cv::Mat& imgDepth, COptData& mOD, int zMin = 100, int zMax = 8000);
}

#endif
