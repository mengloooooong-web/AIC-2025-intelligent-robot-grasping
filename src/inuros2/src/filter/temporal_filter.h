/*
 * Temporal Filter.hpp
 *
 *
 */

#ifndef __TEMPORAL_FILTER_HPP__
#define __TEMPORAL_FILTER_HPP__

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

#include <thread>
#include <numeric>


namespace Inuchip
{
    class TemporalFast {
    public:
        TemporalFast(uint8_t mode = 2);
        cv::Mat process_frame(const cv::Mat &f, float alpha = 0.4f, uint16_t delta = 20);
        cv::Mat process_frame_fix(const cv::Mat &f, float alpha = 0.4f, uint16_t delta = 20);

    private:
        void temporal_smooth(
            cv::Mat &frame_data,
            cv::Mat &_last_frame_data,
            cv::Mat &history,
            float alpha,
            uint16_t delta
        );
        void temporal_smooth_fix(
            cv::Mat &frame_data,
            cv::Mat &_last_frame_data,
            cv::Mat &history,
            float alpha,
            uint16_t delta
        );

        void calc_map(uint8_t mode);
        void setMode(uint8_t mode);

        size_t _width, _height;
        cv::Mat _last_frame;
        cv::Mat _history;
        uint8_t _cur_frame_index;
        std::array<uint8_t, 256> _persistence_map;
    };
    // Class definition

    class TemporalFilter {
    public:
        /*
            * filter_length(8): how many previous frames are used for filter
            * bALL(false):      whether process every frame, false just invalid pixels
            * nThread(1):       how many threads are used
            */
        TemporalFilter(int filter_length = 8, bool bAll = false, int nThread = 1);
        cv::Mat process_frame(const cv::Mat &frame);

    protected:
        void filter_worker(cv::Mat &imgDst, int nbegin, int nend);

        void init_median_lists(int size);
        cv::Mat mFrame;
        int mnFilterlength;
        int mnImgSize;
        bool mbInit;
        bool mbAll;
        int mnFrameid;
        int mnThreadNum;
        std::vector<std::vector<ushort>> mvHistoryLists;
    };
}


#endif
