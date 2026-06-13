#include "temporal_filter.h"

using namespace std;
using namespace cv;

namespace Inuchip
{
    TemporalFast::TemporalFast(uint8_t mode) {
        setMode(mode);
    }

    void TemporalFast::setMode(uint8_t mode) {
        calc_map(mode);
        _cur_frame_index = 0;
        _last_frame.release();
        _history.release();
        _width = 0;
        _height = 0;
    }

    cv::Mat TemporalFast::process_frame(const cv::Mat &f, float alpha, uint16_t delta) {
        if (f.cols != _width || f.rows != _height) {
            _cur_frame_index = 0;
            _last_frame.release();
            _history.release();
            _width = f.cols;
            _height = f.rows;
            _last_frame = cv::Mat(_height, _width, CV_16UC1);
            _history = cv::Mat(_height, _width, CV_8UC1);
        }

        cv::Mat ret;
        f.copyTo(ret);

        temporal_smooth(ret, _last_frame, _history, alpha, delta);

        return ret;
    }

    cv::Mat TemporalFast::process_frame_fix(const cv::Mat &f, float alpha, uint16_t delta) {
        if (f.cols != _width || f.rows != _height) {
            _cur_frame_index = 0;
            _last_frame.release();
            _history.release();
            _width = f.cols;
            _height = f.rows;
            _last_frame = cv::Mat(_height, _width, CV_16UC1);
            _history = cv::Mat(_height, _width, CV_8UC1);
        }

        cv::Mat ret;
        f.copyTo(ret);

        temporal_smooth_fix(ret, _last_frame, _history, alpha, delta);

        return ret;
    }

    void TemporalFast::temporal_smooth(
        cv::Mat &frame_data,
        cv::Mat &_last_frame_data,
        cv::Mat &history,
        float alpha,
        uint16_t delta
    ) {
        unsigned char mask = 1 << _cur_frame_index;

        uint16_t cur_val, prev_val;
        uint16_t height = frame_data.rows;
        uint16_t width = frame_data.cols;

        uint16_t *pCur = frame_data.ptr<ushort>(0);
        uint16_t *pPrev = _last_frame_data.ptr<ushort>(0);
        uint8_t *pHist = history.ptr<uint8_t>(0);
        // use history data to update
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                cur_val = pCur[i * width + j];
                prev_val = pPrev[i * width + j];
                if (cur_val) {
                    if (!prev_val) {
                        pPrev[i * width + j] = cur_val;
                        pHist[i * width + j] = mask;
                    } else { // old and new val
                        uint16_t diff =
                            cur_val > prev_val ? (cur_val - prev_val) : (prev_val - cur_val);

                        if (diff < delta) { // old and new val agree
                            pHist[i * width + j] |= mask;
                            float filtered = alpha * cur_val + (1 - alpha) * prev_val;
                            uint16_t result = static_cast<uint16_t>(filtered + 0.5f);
                            pCur[i * width + j] = result;
                            pPrev[i * width + j] = result;
                        } else {
                            pPrev[i * width + j] = cur_val;
                            pHist[i * width + j] = mask;
                        }
                    }
                } else { // insert current value
                    if (prev_val) {
                        unsigned char hist = pHist[i * width + j];
                        unsigned char classification = _persistence_map[hist];
                        if (classification & mask) {
                            pCur[i * width + j] = prev_val;
                        }
                    }
                    pHist[i * width + j] &= ~mask;
                }
            }
        }
        _cur_frame_index = (_cur_frame_index + 1) % 8; // at end of cycle
    }

    void TemporalFast::temporal_smooth_fix(
        cv::Mat &frame_data,
        cv::Mat &_last_frame_data,
        cv::Mat &history,
        float alpha,
        uint16_t delta
    ) {
        unsigned char mask = 1 << _cur_frame_index;

        uint32_t alpha_fix = alpha * (1 << 16);

        uint16_t cur_val, prev_val;
        uint16_t height = frame_data.rows;
        uint16_t width = frame_data.cols;

        uint16_t *pCur = frame_data.ptr<ushort>(0);
        uint16_t *pPrev = _last_frame_data.ptr<ushort>(0);
        uint8_t *pHist = history.ptr<uint8_t>(0);
        // use history data to update
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                cur_val = pCur[i * width + j];
                prev_val = pPrev[i * width + j];
                if (cur_val) {
                    if (!prev_val) {
                        pPrev[i * width + j] = cur_val;
                        pHist[i * width + j] = mask;
                    } else { // old and new val
                        uint16_t diff =
                            cur_val > prev_val ? (cur_val - prev_val) : (prev_val - cur_val);

                        if (diff < delta) { // old and new val agree
                            pHist[i * width + j] |= mask;
                            uint32_t filtered =
                                alpha_fix * cur_val + ((1 << 16) - alpha_fix) * prev_val;
                            uint16_t result = (filtered + (1 << 15)) >> 16;
                            // float filtered2 = alpha * cur_val + (1-alpha) * prev_val;
                            // uint16_t result2 = static_cast<uint16_t>(filtered2 + 0.5f);
                            // if (result != result2) {
                            // 	printf("frame %d: %d %d, %d vs. %d\n", _cur_frame_index, i, j, result2, result);
                            // }
                            pCur[i * width + j] = result;
                            pPrev[i * width + j] = result;
                        } else {
                            pPrev[i * width + j] = cur_val;
                            pHist[i * width + j] = mask;
                        }
                    }
                } else { // insert current value
                    if (prev_val) {
                        unsigned char hist = pHist[i * width + j];
                        unsigned char classification = _persistence_map[hist];
                        if (classification & mask) {
                            pCur[i * width + j] = prev_val;
                        }
                    }
                    pHist[i * width + j] &= ~mask;
                }
            }
        }
        _cur_frame_index = (_cur_frame_index + 1) % 8; // at end of cycle
    }

    void TemporalFast::calc_map(uint8_t mode) {
        _persistence_map.fill(0);

        for (size_t i = 0; i < _persistence_map.size(); i++) {
            uint8_t last_7 = !!(i & 1); // old
            uint8_t last_6 = !!(i & 2);
            uint8_t last_5 = !!(i & 4);
            uint8_t last_4 = !!(i & 8);
            uint8_t last_3 = !!(i & 16);
            uint8_t last_2 = !!(i & 32);
            uint8_t last_1 = !!(i & 64);
            uint8_t lastFrame = !!(i & 128); // new

            if (mode == 1) {
                int sum = lastFrame + last_1 + last_2 + last_3 + last_4 + last_5 + last_6 + last_7;
                if (sum >= 8) // valid in eight of the last eight frames
                    _persistence_map[i] = 1;
            } else if (mode == 2) // <--- default choice
            {
                int sum = lastFrame + last_1 + last_2;
                if (sum >= 2) // valid in two of the last three frames
                    _persistence_map[i] = 1;
            } else if (mode == 3) // <--- default choice recommended
            {
                int sum = lastFrame + last_1 + last_2 + last_3;
                if (sum >= 2) // valid in two of the last four frames
                    _persistence_map[i] = 1;
            } else if (mode == 4) {
                int sum = lastFrame + last_1 + last_2 + last_3 + last_4 + last_5 + last_6 + last_7;
                if (sum >= 2) // valid in two of the last eight frames
                    _persistence_map[i] = 1;
            } else if (mode == 5) {
                int sum = lastFrame + last_1;
                if (sum >= 1) // valid in one of the last two frames
                    _persistence_map[i] = 1;
            } else if (mode == 6) {
                int sum = lastFrame + last_1 + last_2 + last_3 + last_4;
                if (sum >= 1) // valid in one of the last five frames
                    _persistence_map[i] = 1;
            } else if (mode == 7) //  <--- most filling
            {
                int sum = lastFrame + last_1 + last_2 + last_3 + last_4 + last_5 + last_6 + last_7;
                if (sum >= 1) // valid in one of the last eight frames
                    _persistence_map[i] = 1;
            } else if (mode == 8) //  <--- all 1's
            {
                _persistence_map[i] = 1;
            } else // all others, including 0, no persistance
            {
            }
        }

        // Convert to credible enough
        std::array<uint8_t, 256> credible_threshold;
        credible_threshold.fill(0);

        for (auto phase = 0; phase < 8; phase++) {
            uint8_t mask = 1 << phase;
            int i;

            for (i = 0; i < 256; i++) {
                uint8_t pos = (uint8_t)((i << (8 - phase)) | (i >> phase));
                if (_persistence_map[pos])
                    credible_threshold[i] |= mask;
            }
        }
        // Store results
        _persistence_map = credible_threshold;
    }

    TemporalFilter::TemporalFilter(int filter_length, bool bAll, int nThread) :
        mnFilterlength(filter_length), mnImgSize(0), mbInit(false), mbAll(bAll), mnFrameid(0),
        mnThreadNum(nThread) {}

    void TemporalFilter::init_median_lists(int size) {
        this->mnImgSize = size;

        std::vector<ushort> vec(mnFilterlength, 0);
        mvHistoryLists = std::vector<std::vector<ushort>>(size, vec);

        for (auto vec : mvHistoryLists)
            for (auto pix : vec)
                // pix = std::rand();
                pix = 0;

        mnFrameid = 0;
        mbInit = true;
    }

    void TemporalFilter::filter_worker(cv::Mat &imgDst, int nbegin, int nend) {
        cv::MatConstIterator_<ushort> bgr_it = this->mFrame.begin<ushort>() + nbegin,
                                    bgr_end =
                                        this->mFrame.begin<ushort>() + nend; // frame.end  <ushort>();

        std::vector<std::vector<ushort>>::iterator his_it = mvHistoryLists.begin() + nbegin;
        cv::MatIterator_<ushort> dst_it = imgDst.begin<ushort>() + nbegin;

        for (; bgr_it != bgr_end; bgr_it++, dst_it++, his_it++) {
            (*his_it)[mnFrameid] = *bgr_it;
            ushort cur = *bgr_it;

            if (cur > 0 && !mbAll) {
                *dst_it = cur;
                continue;
            }
            double sum = 0;
            int nItem = 0;
            for (int m = 0; m < (*his_it).size(); m++) {
                if ((*his_it)[m] > 0) {
                    sum += (*his_it)[m];
                    nItem++;
                }
            }
            if (nItem > 0) {
                sum /= nItem;
                *dst_it = (int)sum;
            } else
                *dst_it = cur;
        }
    }

    cv::Mat TemporalFilter::process_frame(const cv::Mat &frame) {
        // Initialize the filter if we haven't yet

        mFrame = frame;

        if (!mbInit)
            init_median_lists((int)frame.total());

        cv::Mat retDst(mFrame.size(), mFrame.type());

        if (mnThreadNum == 1) {
            filter_worker(retDst, 0, mnImgSize);
        } else {
            std::vector<std::thread *> vThreads;
            int nInter = mnImgSize / mnThreadNum;
            int b0 = 0;
            int b1 = nInter;
            for (int i = 0; i < mnThreadNum; i++) {
                std::thread *th1 =
                    new std::thread(&TemporalFilter::filter_worker, this, std::ref(retDst), b0, b1);
                vThreads.push_back(th1);
                b0 = b1;
                b1 = b1 + nInter;
                if (i == mnThreadNum - 2)
                    b1 = mnImgSize;
            }
            for (int i = 0; i < mnThreadNum; i++) {
                vThreads[i]->join();
            }
            for (int i = 0; i < mnThreadNum; i++) {
                delete vThreads[i];
            }
        }

        mnFrameid = (mnFrameid + 1) % mnFilterlength;

        return retDst;
    }
}
