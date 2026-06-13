#ifndef __SPACE_FILTER_H__
#define __SPACE_FILTER_H__

#define SPEED_OPT

#pragma once
#include <algorithm>
#include <map>
#include <vector>
#include <cmath>
#include "opencv2/opencv.hpp"
namespace Inuchip
{

	class Space_Filter
	{
	public:
		Space_Filter();
		cv::Mat process_outlier_remove(const cv::Mat& frame, float fMaxPercent = 0.1f, int min_dist = 30, int maxremove = 50);
		cv::Mat process_hole_fill(const cv::Mat& frame, int max_radius = 25);
		cv::Mat process_passthrough(const cv::Mat& frame, int zMin = 40, int zMax = 8000, int xMin = -1, int xMax = -1, int yMin = -1, int yMax = -1);

		cv::Mat guassian_filter(const cv::Mat& img, int ksize = 5, double sigma = 0.1);
		cv::Mat anisotropic_filter(const cv::Mat& img, int iterationSum = 5, float k = 30, float lambda = 0.2);

		cv::Mat process_edge_preserve(const cv::Mat& f, float alpha = 0.5f, int delta = 20, int iter = 2, int hole_radius = 0);
		cv::Mat fast_global_smooth(const cv::Mat& image, const cv::Mat& guide, double sigma=0.2, double lambda=900, int solver_iteration=3, double solver_attenuation=4);

	protected:

		// Main functions


		void filter_horizontal(cv::Mat* image_data, float alpha, uint16_t deltaZ, int hole_radius);
		void filter_vertical(cv::Mat* image_data, float alpha, uint16_t deltaZ);

		void prepareKernelLUT(double sigma, int KernelLUTLen);
		void solve_three_point_laplacian_matrix(double x[], const size_t N, const double a[], const double b[], double c[]);


	private:

		std::vector<double>		_KernelLUT;
		double _sigma;
		int _nChannels_guide; // different from depth and rgb image
		int MAXKERNELLUTLEN = 20000;
	};

}

#endif
