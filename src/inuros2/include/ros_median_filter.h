/*
 * MedianFilter.hpp
 *
 * Class to implement a pixelwise median filter.
 */

#ifndef __MEDIAN_FILTER_HPP__
#define __MEDIAN_FILTER_HPP__

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

#include <thread>

#include <omp.h>
#define ompfor __pragma(omp parallel for) for
#define omplock __pragma(omp critical)
#define ompatomic __pragma(omp atomic)


// Class definition

class MedianFilter
{
public:

    MedianFilter(int filter_length);
    cv::Mat process_frame(const cv::Mat &frame);
    void init_median_lists(int size);
public:

    cv::Mat mframe;
    int filter_length;
    int size;
    bool pop_front;
    bool is_init;
    int cur_id;
    std::vector< std::vector<ushort> > median_lists;
};


// MedianFilter public methods

MedianFilter::MedianFilter(int filter_length)
    : filter_length(filter_length)
    , size(0)
    , is_init(false)
    , cur_id(0)
{ }


void Do_Median(MedianFilter *mf, cv::Mat &dst, int nbegin,int nend)
{

    cv::MatConstIterator_<ushort> bgr_it = mf->mframe.begin<ushort>() + nbegin,
        bgr_end = mf->mframe.begin<ushort>() + nend;// frame.end  <ushort>();

    std::vector<std::vector<ushort>>::iterator med_it = mf->median_lists.begin() + nbegin;

    // Matrix to stuff the "current frame" into.
    cv::MatIterator_<ushort> dst_it = dst.begin<ushort>()+nbegin;

    for (; bgr_it != bgr_end; bgr_it++, dst_it++, med_it++)
    {
        if (mf->pop_front)
            (*med_it)[0] = *bgr_it;
        else
            (*med_it)[mf->filter_length - 1] = *bgr_it;


        int med_idx = (*med_it).size() / 2;

        std::nth_element((*med_it).begin(), (*med_it).begin() + med_idx, (*med_it).end());

        *dst_it = (*med_it)[med_idx];
    }


}
cv::Mat MedianFilter::process_frame(const cv::Mat &frame)
{

    // Initialize the filter if we haven't yet
    mframe = frame;

    if ( ! is_init )
        init_median_lists(frame.total());

    assert( median_lists.size() == frame.total() );

    // Init iterators
    cv::MatConstIterator_<ushort> bgr_it = frame.begin<ushort>(),
        bgr_end = frame.end  <ushort>();

    std::vector<std::vector<ushort>>::iterator med_it = median_lists.begin();

    // Matrix to stuff the "current frame" into.
    cv::Mat dst(frame.rows, frame.cols, CV_16UC1);
    cv::MatIterator_<ushort> dst_it = dst.begin<ushort>();

    auto t1 = std::chrono::steady_clock::now();

#if 1
    std::vector<std::thread*> vThreads;
    int nthread = 4;
    int nInter = median_lists.size() / nthread;
    int b0 = 0;
    int b1 = nInter;
    for (int i = 0; i < nthread; i++)
    {
        std::thread *th1  = new std::thread(Do_Median, this, std::ref(dst), b0, b1);
        vThreads.push_back(th1);
        b0 = b1;
        b1 = b1 + nInter;
        if (i == nthread - 2)
            b1 = median_lists.size();
    }
    for (int i = 0; i < nthread; i++)
    {
        vThreads[i]->join();
    }
    for (int i = 0; i < nthread; i++)
    {
        delete vThreads[i];
    }

    //Do_Median(bgr_it, bgr_end, dst_it, med_it, pop_front, filter_length);
#else
    // Iterate through all pixels in each image together
    for( ; bgr_it != bgr_end; bgr_it++, dst_it++, med_it++)
    {
        if (1)
        {
            if (pop_front)
                (*med_it)[0] = *bgr_it;
            else
                (*med_it)[filter_length - 1] = *bgr_it;


            int med_idx = (*med_it).size() / 2;

            std::nth_element((*med_it).begin(), (*med_it).begin() + med_idx, (*med_it).end());

            *dst_it = (*med_it)[med_idx];
        }
        else
        {
            //input new data
            (*med_it)[cur_id] = *bgr_it;

            std::vector<ushort> vtmp(*med_it);
            std::sort(vtmp.begin(), vtmp.end());

            ushort vmin = vtmp.back();
            ushort vmax = 0;
            for (int m = 0; m < vtmp.size();m++)
            {
                if (vtmp[m] == 0)
                    continue;
                if (vtmp[m] > vmax) vmax = vtmp[m];
                if (vtmp[m] < vmin) vmin = vtmp[m];
            }
            if (vmax - vmin > 200)
                *dst_it = *bgr_it;
            else
                *dst_it = vtmp[vtmp.size() / 2];
        }
    }

#endif

    pop_front = ! pop_front;
    cur_id = (cur_id + 1) % filter_length;

    double dr_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count();

    std::cout << "time(ms): " << dr_ms << std::endl;
    return dst;
}


void MedianFilter::init_median_lists(int size)
{
    this->size = size;

    std::vector<ushort> vec(filter_length, 0);
    median_lists = std::vector< std::vector<ushort> >(size, vec);

    for (auto vec : median_lists)
        for (auto pix : vec)
            pix = std::rand();

    cur_id = 0;
    is_init = true;
}


#endif
