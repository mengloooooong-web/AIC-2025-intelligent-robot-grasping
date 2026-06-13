#ifndef __HOLE_FILL_H__
#define __HOLE_FILL_H__

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

using namespace std;
using namespace cv;

class HoleFill
{
public:

    HoleFill(int max_radius = 20);
    cv::Mat process_frame(const cv::Mat& frame);

private:
    int max_radius;
    /*
    :param max_radius : float, maximum radius for the neighboring area
    */

};

HoleFill::HoleFill(int max_radius)
	:max_radius(max_radius)
{ }


cv::Mat HoleFill::process_frame(const cv::Mat& frame)
{

    cv::Mat retImg = frame.clone();
    cv::Mat flag(frame.size(), CV_8UC1, char(0));
    for (int r = 0; r < retImg.rows; r++)
    {
        for (int c = 0; c < retImg.cols; c++)
        {
            if (retImg.at<ushort>(r, c) > 0) continue;
            if (flag.at<uchar>(r, c) == 1) continue;
            vector<cv::Point2i> vHoles;
            float fMean = 0.0f;
            int nMean = 0;
            vHoles.emplace_back(Point2i(r, c));
            flag.at<uchar>(r, c) = 1;

            int idx = 0;
            while (idx < vHoles.size())
            {
                Point2i pCur = vHoles[idx];
                for (int rc = -1; rc <= 1; rc++)
                {
                    int rx = rc + pCur.x;
                    if (rx < 0 || rx >= retImg.rows)
                        continue;
                    for (int cc = -1; cc <= 1; cc++)
                    {
                        int ry = cc + pCur.y;
                        if (ry < 0 || ry >= retImg.cols)
                            continue;

                        if (retImg.at<ushort>(rx, ry) == 0 && flag.at<uchar>(rx, ry) == 0)
                        {
                            vHoles.emplace_back(Point2i(rx, ry));
                            flag.at<uchar>(rx, ry) = 1;
                        }
                        else if (frame.at<ushort>(rx, ry) > 0)
                        {
                            nMean++;
                            fMean = fMean * (nMean - 1) + frame.at<ushort>(rx, ry);
                            fMean /= nMean;
                        }
                    }
                }
                idx++;
            }
            //we get the holes
            cv::Rect rcHole = cv::boundingRect(vHoles);
            if (rcHole.width > max_radius || rcHole.height > max_radius)
                continue;

            for (vector<Point2i>::iterator it = vHoles.begin(); it != vHoles.end(); it++)
            {
                retImg.at<ushort>((*it).x, (*it).y) = (int)fMean;
            }

        }
    }

    return retImg;
}



#endif