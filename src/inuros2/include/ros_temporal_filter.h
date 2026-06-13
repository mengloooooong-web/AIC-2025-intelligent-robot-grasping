/*
 * MedianFilter.hpp
 *
 * Class to implement a pixelwise median filter.
 */

#ifndef __TEMPORAL_FILTER_HPP__
#define __TEMPORAL_FILTER_HPP__

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

#include <thread>
#include <numeric>

using namespace std;
using namespace cv;

// Class definition
class TemporalFilter
{
public:
    /*
    * filter_length(6): how many previous frames are used for filter
    * bALL(false):      whether process every frame, false just invalid pixels
    * nThread(1):       how many threads are used
    */
    TemporalFilter(int filter_length=6,bool bAll=false, int nThread=1);
    cv::Mat process_frame(const cv::Mat &frame);

protected:
    void filter_worker(cv::Mat& imgDst, int nbegin, int nend);

    void init_median_lists(int size);
    cv::Mat mFrame;
    int mnFilterlength;
    int mnImgSize;
    bool mbInit;
    bool mbAll;
    int mnFrameid;
    int mnThreadNum;
    std::vector< std::vector<ushort> > mvHistoryLists;
};

TemporalFilter::TemporalFilter(int filter_length, bool bAll, int nThread)
    : mnFilterlength(filter_length)
    , mnImgSize(0)
    , mbInit(false)
    , mbAll(bAll)
    , mnFrameid(0)
    , mnThreadNum(nThread)
{ }


void TemporalFilter::init_median_lists(int size)
{
    this->mnImgSize = size;

    std::vector<ushort> vec(mnFilterlength, 0);
    mvHistoryLists = std::vector< std::vector<ushort> >(size, vec);

    for (auto vec : mvHistoryLists)
        for (auto pix : vec)
            pix = std::rand();

    mnFrameid = 0;
    mbInit = true;
}

void TemporalFilter::filter_worker(cv::Mat &imgDst, int nbegin, int nend)
{
    cv::MatConstIterator_<ushort> bgr_it = this->mFrame.begin<ushort>() + nbegin,
        bgr_end = this->mFrame.begin<ushort>() + nend;// frame.end  <ushort>();

    std::vector<std::vector<ushort>>::iterator his_it = mvHistoryLists.begin() + nbegin;
    cv::MatIterator_<ushort> dst_it = imgDst.begin<ushort>() + nbegin;

    for (; bgr_it != bgr_end; bgr_it++, dst_it++, his_it++)
    {
        (*his_it)[mnFrameid] = *bgr_it;
        ushort cur = *bgr_it;

        if (cur > 0 && !mbAll)
        {
            *dst_it = cur;
            continue;
        }
        double sum = 0;
        int nItem = 0;
        for (int m = 0; m < (*his_it).size(); m++)
        {
            if ((*his_it)[m] > 0)
            {
                sum += (*his_it)[m];
                nItem++;
            }

        }
        if (nItem > 0)
        {
            sum /= nItem;
            *dst_it = (int)sum;
        }
        else
            *dst_it = cur;
    }
}
cv::Mat TemporalFilter::process_frame(const cv::Mat& frame)
{

    // Initialize the filter if we haven't yet

    mFrame = frame;

    if (!mbInit)
        init_median_lists((int)frame.total());

    cv::Mat retDst(mFrame.size(), mFrame.type());

    if (mnThreadNum == 1)
    {
        filter_worker(retDst, 0, mnImgSize);
    }
    else
    {
        std::vector<std::thread*> vThreads;
        int nInter = mnImgSize / mnThreadNum;
        int b0 = 0;
        int b1 = nInter;
        for (int i = 0; i < mnThreadNum; i++)
        {
            std::thread* th1 = new std::thread(&TemporalFilter::filter_worker, this, std::ref(retDst), b0, b1);
            vThreads.push_back(th1);
            b0 = b1;
            b1 = b1 + nInter;
            if (i == mnThreadNum - 2)
                b1 = mnImgSize;
        }
        for (int i = 0; i < mnThreadNum; i++)
        {
            vThreads[i]->join();
        }
        for (int i = 0; i < mnThreadNum; i++)
        {
            delete vThreads[i];
        }
    }

    mnFrameid = (mnFrameid + 1) % mnFilterlength;

    return retDst;
}

#endif
