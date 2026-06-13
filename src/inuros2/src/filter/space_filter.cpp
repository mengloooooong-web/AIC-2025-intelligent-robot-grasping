#include "space_filter.h"


using namespace cv;
using namespace std;

namespace Inuchip
{

    Space_Filter::Space_Filter():_sigma(0)
    {
    }



    cv::Mat Space_Filter::anisotropic_filter(const cv::Mat& imgSrc, int iterationSum, float k, float lambda)
    {
        //iterationSum
        //k
        //lambda
        cv::Mat img = imgSrc.clone();
        cv::Mat retMat(img.size(), img.type(), Scalar(0));

        int nImgWidth = img.cols;
        int nImgHeight = img.rows;


        for (int iter = 0; iter < iterationSum; ++iter)
        {
            if (iter > 0)
                img = retMat.clone();
            for (int i = 0; i < nImgHeight; ++i)
            {
                const ushort* ppafScan = img.ptr<ushort>(i);
                ushort* outppafScan = retMat.ptr<ushort>(i);

                for (int j = 0; j < nImgWidth; ++j)
                {
                    if (i == 0 || i == nImgHeight - 1 || j == 0 || j == nImgWidth - 1 || ppafScan[j] == 0)
                    {
                        outppafScan[j] = ppafScan[j];
                        continue;
                    }
                    const ushort* ppafScan_pre = img.ptr<ushort>(i - 1);
                    const ushort* ppafScan_nex = img.ptr<ushort>(i + 1);
                    if (ppafScan[j - 1] == 0 || ppafScan_pre[j] == 0 || ppafScan_nex[j] == 0 || ppafScan[j + 1] == 0)
                    {
                        outppafScan[j] = ppafScan[j];
                        continue;
                    }
                    float NI = ppafScan[j - 1] - ppafScan[j];
                    float EI = ppafScan_pre[j] - ppafScan[j];
                    float WI = ppafScan_nex[j] - ppafScan[j];
                    float SI = ppafScan[j + 1] - ppafScan[j];

                    float cN = exp(-NI * NI / (k * k));
                    float cE = exp(-EI * EI / (k * k));
                    float cW = exp(-WI * WI / (k * k));
                    float cS = exp(-SI * SI / (k * k));

                    outppafScan[j] = ppafScan[j] + lambda * (cN * NI + cS * SI + cE * EI + cW * WI);
                }
            }
        }

        return retMat;
    }

    cv::Mat Space_Filter::guassian_filter(const cv::Mat& img, int ksize, double sigma)
    {
        cv::Mat retMat;
        cv::GaussianBlur(img, retMat, cv::Size(ksize, ksize), sigma);


        return retMat;
    }

    cv::Mat Space_Filter::process_passthrough(const cv::Mat& frame, int zMin, int zMax, int xMin, int xMax, int yMin, int yMax)
    {
        cv::Mat retImg = frame.clone();
        bool bConX = (xMin >= 0 && xMin < xMax);
        bool bConY = (yMin >= 0 && yMin < yMax);
        bool bConZ = (zMin >= 0 && zMin < zMax);
        for (int r = 0; r < retImg.rows; r++)
        {
            bool bValY = false;
            if (bConY)
                bValY = (r >= yMin && r <= yMax);
            else
                bValY = true;

            for (int c = 0; c < retImg.cols; c++)
            {
                ushort z = retImg.at<ushort>(r, c);
                if (z == 0)
                    continue;
                bool bValX = false;
                if (bConX)
                    bValX = (c >= xMin && c <= xMax);
                else
                    bValX = true;

                bool bValZ = false;
                if (bConZ)
                    bValZ = (z >= zMin && z <= zMax);
                else
                    bValZ = true;

                if (!(bValZ && bValX && bValY))
                    retImg.at<ushort>(r, c) = 0;

            }
        }
        return retImg;
    }

    cv::Mat Space_Filter::process_outlier_remove(const cv::Mat& frame, float fMaxPercent, int min_dist, int maxremove)
    {

        cv::Mat retImg = frame.clone();
        cv::Mat flag(frame.size(), CV_8UC1, Scalar(0));

        int iThresh = min_dist;//mm
        std::vector< cv::Point2i > indices;
        std::vector<cv::Point2i> vHoles(retImg.rows * retImg.cols);
        indices.reserve(100);

        size_t idx = 0;
        for (int r = 1; r < retImg.rows - 1; r++)
        {
            for (int c = 1; c < retImg.cols - 1; c++)
            {
                if (retImg.at<ushort>(r, c) == 0) continue;
                if (flag.at<uchar>(r, c) == 1) continue;
                flag.at<uchar>(r, c) = 1;

                cv::Point2i piRange;
                piRange.x = idx;

                size_t c_idx = idx;//local idx
                vHoles[idx].x = r;
                vHoles[idx++].y = c;

                while (c_idx < idx)
                {
                    Point2i& pCur = vHoles[c_idx];
                    if (pCur.x == 0 || pCur.y == 0 || pCur.x == retImg.rows - 1 || pCur.y == retImg.cols - 1)
                    {
                        c_idx++;
                        continue;
                    }
                    int zCur = retImg.at<ushort>(pCur.x, pCur.y);

                    for (int rc = -1; rc <= 1; rc++)
                    {
                        int rx = rc + pCur.x;
                        for (int cc = -1; cc <= 1; cc++)
                        {
                            int ry = cc + pCur.y;
                            if (flag.at<uchar>(rx, ry) == 1 || retImg.at<ushort>(rx, ry) == 0)
                                continue;

                            if (abs(retImg.at<ushort>(rx, ry) - zCur) < iThresh)
                            {
                                vHoles[idx].x = rx;
                                vHoles[idx++].y = ry;

                                flag.at<uchar>(rx, ry) = 1;
                            }
                        }
                    }
                    c_idx++;
                }
                piRange.y = idx;

                if (piRange.y - piRange.x < maxremove)
                {
                    for (vector<Point2i>::iterator it = vHoles.begin() + piRange.x; it != vHoles.begin() + piRange.y; it++)
                    {
                        retImg.at<ushort>((*it).x, (*it).y) = 0;
                    }
                }
                else
                    indices.push_back(piRange);

            }
        }
        if (indices.size() == 0)
            return retImg;
        int iMax = indices[0].y - indices[0].x;
        for (size_t i = 1; i < indices.size(); i++)
        {
            int tsize = indices[i].y - indices[i].x;
            if (iMax < tsize)
                iMax = tsize;
        }

        int iThresh2 = int(iMax * fMaxPercent);
        for (size_t i = 0; i < indices.size(); i++)
        {
            if (indices[i].y - indices[i].x < iThresh2)
            {
                for (vector<Point2i>::iterator it = vHoles.begin() + indices[i].x; it != vHoles.begin() + indices[i].y; it++)
                {
                    retImg.at<ushort>((*it).x, (*it).y) = 0;
                }

            }
        }

        return retImg;
    }


    cv::Mat Space_Filter::process_hole_fill(const cv::Mat& frame, int max_radius)
    {

        cv::Mat retImg = frame.clone();

        //cv::Mat flag(frame.size(), CV_8UC1, char(0));
        std::vector<uchar> vec(frame.cols, 0);
        std::vector< std::vector<uchar> > flag = std::vector< std::vector<uchar> >(frame.rows, vec);

        std::vector<cv::Point2i> vHoles(retImg.rows * retImg.cols);

        for (int r = 1; r < retImg.rows - 1; r++)
        {
            for (int c = 1; c < retImg.cols - 1; c++)
            {
                if (retImg.at<ushort>(r, c) > 0) continue;
                if (flag[r][c] == 1) continue;
                float fMean = 0.0f;
                int nMean = 0;
                flag[r][c] = 1;
                size_t idx = 0;
                size_t c_idx = idx;//local idx

                vHoles[idx].x = r;
                vHoles[idx++].y = c;

                int rmin(r), rmax(r);
                int cmin(c), cmax(c);
                while (c_idx < idx)
                {
                    Point2i& pCur = vHoles[c_idx];
                    if (pCur.x <= 0 || pCur.y <= 0 || pCur.x >= retImg.rows - 1 || pCur.y >= retImg.cols - 1)
                    {
                        c_idx++;
                        continue;
                    }
                    for (int rc = -1; rc <= 1; rc++)
                    {
                        int rx = rc + pCur.x;
                        for (int cc = -1; cc <= 1; cc++)
                        {
                            int ry = cc + pCur.y;

                            if (retImg.at<ushort>(rx, ry) == 0 && flag[rx][ry] == 0)
                            {
                                vHoles[idx].x = rx;
                                vHoles[idx++].y = ry;
                                flag[rx][ry] = 1;
                                if (rx > rmax) rmax = rx;
                                if (rx < rmin) rmin = rx;
                                if (ry > cmax) cmax = ry;
                                if (ry < cmin) cmin = ry;

                            }
                            else if (frame.at<ushort>(rx, ry) > 0)
                            {
                                nMean++;
                                fMean = fMean * (nMean - 1) + frame.at<ushort>(rx, ry);
                                fMean /= nMean;
                            }
                        }
                    }
                    c_idx++;
                }

                //std::cout << "or hole: " << vHoles[0].x << "," << vHoles[0].y << ", wh: " << rcHole.width << "," << rcHole.height << std::endl;
                if (rmax - rmin > max_radius || cmax - cmin > max_radius)
                    continue;

                for (vector<Point2i>::iterator it = vHoles.begin(); it != vHoles.begin() + idx; it++)
                {
                    retImg.at<ushort>((*it).x, (*it).y) = (int)fMean;
                }

            }
        }

        return retImg;
    }

    /*
    * alpha 0.5 [0.25-1] The weight of the current pixel for smoothing is bounded within [25..100]%
    * delta 20 [1-50]    The depth gradient below which the smoothing will occur as number of depth levels
    * iter  2 [1-5]
    */
    cv::Mat Space_Filter::process_edge_preserve(const cv::Mat&f, float alpha, int delta,int iter, int hole_radius)
    {
        cv::Mat ret;
        f.copyTo(ret);

        for (int i = 0; i < iter; i++)
        {
            filter_horizontal(&ret, alpha, delta, hole_radius);
            filter_vertical(&ret, alpha, delta);
        }

        return ret;

    }

    void  Space_Filter::filter_horizontal(cv::Mat* image_data, float alpha, uint16_t deltaZ, int hole_radius)
    {

        const uint16_t valid_threshold = 1;

        size_t cur_fill = 0;
        //uint16_t val0, val1;
        uint16_t val0, val1;
        uint16_t width = image_data->cols;
        uint16_t height = image_data->rows;
        uint16_t* data = image_data->ptr<ushort>(0);
        for (int i = 0; i < height; i++)
        {
            val0 = data[i*width];
            cur_fill = 0;
            for (int j = 1; j < width - 1; j++)
            {
                val1 = data[i * width+j];
                if (fabs(val0) >= valid_threshold)
                {
                    if (fabs(val1) >= valid_threshold)
                    {
                        cur_fill = 0;
                        uint16_t diff = static_cast<uint16_t>(fabs(val1 - val0));// val1 > val0 ? (val1 - val0) : (val0 - val1);
                        if (diff >= valid_threshold && diff <= deltaZ)
                        {
                            float filtered = (val1) * alpha + (val0) * (1.0f - alpha);
                            val1 = (uint16_t)(filtered + 0.5);
                            data[i * width +j] = val1;
                        }
                    }
                    else
                    {
                        if (hole_radius)
                        {
                            if (++cur_fill < hole_radius)
                                data[i * width+j] = val1 = val0;
                        }
                    }
                }
                val0 = val1;
            }

            val0 = data[(i + 1)* width - 1];
            for (int j = width - 2; j >=0; j--)
            {
                val1 = data[i * width + j];
                if (fabs(val0) >= valid_threshold)
                {
                    if (fabs(val1) >= valid_threshold)
                    {
                        cur_fill = 0;
                        uint16_t diff = val1 > val0 ? (val1 - val0) : (val0 - val1);
                        if (diff >= valid_threshold && diff <= deltaZ)
                        {
                            float filtered = (val1) * alpha + (val0) * (1.0f - alpha);
                            val1= (uint16_t)(filtered + 0.5);
                            data[i * width + j] = val1;
                        }
                    }
                    else
                    {
                        if (hole_radius)
                        {
                            if (++cur_fill < hole_radius)
                                data[i * width + j]  = val1 = val0;
                        }
                    }
                }
                val0 = val1;
            }
        }
    }

    void Space_Filter::filter_vertical(cv::Mat* image_data, float alpha, uint16_t deltaZ)
    {
        const uint16_t valid_threshold = 1;

        uint16_t val0, val1;
        uint16_t width = image_data->cols;
        uint16_t height = image_data->rows;
        uint16_t* data = image_data->ptr<ushort>(0);
        for (int i = 1; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                val0 = data[j + (i - 1) * width];
                val1 = data[j + i * width];
                //if ((fabs(val0) >= valid_threshold) && (fabs(val1) >= valid_threshold))
                {
                    uint16_t diff = (val0) > (val1) ? ((val0) - (val1)): ((val1) - (val0));
                    if (diff < deltaZ)
                    {
                        float filtered = (val1)*alpha + (val0) * (1.f - alpha);
                        val1 = (uint16_t)(filtered + 0.5);
                        data[j + i * width] = val1;
                    }
                }
            }
        }
        for (int i = height - 2; i >= 0; i--)
        {
            for (int j = 0; j < width; j++)
            {
                val0 = data[(i + 1) * width + j];
                val1 = data[i * width + j];
                if ((fabs(val0) >= valid_threshold) && (fabs(val1) >= valid_threshold))
                {
                    uint16_t diff = (val0) > (val1) ? ((val0) - (val1)) : ((val1) - (val0));
                    if (diff < deltaZ)
                    {
                        float filtered = (val1)*alpha + (val0) * (1.f - alpha);
                        val1 = (uint16_t)(filtered + 0.5);
                        data[j + i * width] = val1;
                    }
                }
            }
        }
    }
    // Build LUT for bilateral kernel weight
    void Space_Filter::prepareKernelLUT(double sigma, int nChannels_guide)
    {
        if (nChannels_guide == 1)
        {
            _KernelLUT.clear();
            _KernelLUT.resize(MAXKERNELLUTLEN);
            for (int m = 0; m < MAXKERNELLUTLEN; m++)
                _KernelLUT[m] = exp(-(double)m / (sigma)); // Kernel LUT

        }
        else
        {
            _KernelLUT.clear();
            _KernelLUT.resize(195075); //255*255*3
            for (int m = 0; m < _KernelLUT.size(); m++)
                _KernelLUT[m] = exp(-sqrt((double)m) / (sigma)); // Kernel LUT

        }
    }


    cv::Mat Space_Filter::fast_global_smooth(const cv::Mat& image, const cv::Mat& guide, double sigma, double lambda, int solver_iteration, double solver_attenuation)
    {
        cv::Mat tmpimg = image.clone();
        int nChannels_guide = guide.channels();
        int nChannels = image.channels();
        int color_diff;

        int width = image.cols;
        int height = image.rows;


        double* a_vec = (double*)malloc(sizeof(double) * width);
        double* b_vec = (double*)malloc(sizeof(double) * width);
        double* c_vec = (double*)malloc(sizeof(double) * width);
        double* x_vec = (double*)malloc(sizeof(double) * width);

        double* a2_vec = (double*)malloc(sizeof(double) * height);
        double* b2_vec = (double*)malloc(sizeof(double) * height);
        double* c2_vec = (double*)malloc(sizeof(double) * height);
        double* x2_vec = (double*)malloc(sizeof(double) * height);


        if (sigma != _sigma || nChannels_guide != _nChannels_guide) {
            cout << "change LUT" << endl;
            prepareKernelLUT(sigma, nChannels_guide);
            _nChannels_guide = nChannels_guide;
            _sigma = sigma;
        }

        //Variation of lambda (NEW)
        double lambda_in = 1.5 * lambda * pow(4.0, solver_iteration - 1) / (pow(4.0, solver_iteration) - 1.0);
        uint8_t* pRGBchar = (uint8_t*)guide.ptr<uint8_t>(0);
        uint16_t* pRGB = (uint16_t*)guide.ptr<uint16_t>(0);
        uint16_t* pDepth = tmpimg.ptr<uint16_t>(0);
        uint16_t* pOrig = (uint16_t*)image.ptr<uint16_t>(0);

        for (int iter = 0; iter < solver_iteration; iter++)
        {
            //for each row
            for (int i = 0; i < height; i++)
            {
                memset(a_vec, 0, sizeof(double) * width);
                memset(b_vec, 0, sizeof(double) * width);
                memset(c_vec, 0, sizeof(double) * width);
                memset(x_vec, 0, sizeof(double) * width);
                for (int j = 1; j < width; j++)
                {
                    color_diff = 0;
                    // compute bilateral weight for all channels
                    if (nChannels_guide == 1)
                    { // depth do not need to calculate square, but need to ensure value is positive and inside the range(0~MAXKERNELLUTLEN)
                        color_diff = pRGB[i * width + j] - pRGB[i * width + j - 1];
                        color_diff = color_diff > 0 ? color_diff : -color_diff;
                        if (color_diff > MAXKERNELLUTLEN)
                            color_diff = MAXKERNELLUTLEN - 1;
                    }
                    else
                    {
                        for (int c = 0; c < nChannels_guide; c++)
                        {
                            int diff = pRGBchar[(i * width + j) * nChannels_guide + c] - pRGBchar[(i * width + j - 1) * nChannels_guide + c];
                            color_diff += diff * diff;
                        }
                    }

                    a_vec[j] = -lambda_in * _KernelLUT[color_diff];
                }
                memcpy(c_vec, &a_vec[1], (width - 1) * sizeof(double));

                for (int j = 0; j < width; j++)
                {
                    b_vec[j] = 1.f - a_vec[j] - c_vec[j];
                }

                for (int j = 0; j < width; j++)
                {
                    x_vec[j] = pDepth[i * width + j];
                }
                solve_three_point_laplacian_matrix(x_vec, width, a_vec, b_vec, c_vec);
                for (int j = 0; j < width; j++)
                {
                    pDepth[i * width + j] = x_vec[j];
                }
            }


            //for each column
            for (int j = 0; j < width; j++)
            {
                memset(a2_vec, 0, sizeof(double) * height);
                memset(b2_vec, 0, sizeof(double) * height);
                memset(c2_vec, 0, sizeof(double) * height);
                memset(x2_vec, 0, sizeof(double) * height);
                for (int i = 1; i < height; i++)
                {
                    color_diff = 0;
                    // compute bilateral weight for all channels
                    if (nChannels_guide == 1)
                    { // depth do not need to calculate square, but need to ensure value is positive and inside the range(0~MAXKERNELLUTLEN)
                        color_diff = pRGB[i * width + j] - pRGB[(i - 1) * width + j];
                        color_diff = color_diff > 0 ? color_diff : -color_diff;
                        if (color_diff > MAXKERNELLUTLEN)
                            color_diff = MAXKERNELLUTLEN - 1;
                    }
                    else
                    {
                        for (int c = 0; c < nChannels_guide; c++)
                        {
                            int diff = pRGBchar[(i * width + j) * nChannels_guide + c] - pRGBchar[((i - 1) * width + j) * nChannels_guide + c];
                            color_diff += diff * diff;
                        }
                    }
                    a2_vec[i] = -lambda_in * _KernelLUT[color_diff];
                }

                memcpy(c2_vec, &a2_vec[1], sizeof(double) * (height - 1));


                for (int i = 0; i < height; i++) {
                    b2_vec[i] = 1.f - a2_vec[i] - c2_vec[i];
                }

                for (int i = 0; i < height; i++)
                    x2_vec[i] = pDepth[i * width + j];
                solve_three_point_laplacian_matrix(x2_vec, height, a2_vec, b2_vec, c2_vec);

                for (int i = 0; i < height; i++)
                {
                    pDepth[i * width + j] = x2_vec[i];
                }
            }

            lambda_in /= solver_attenuation;
        }


        // filter abnormal points
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                if (abs(pOrig[i * width + j] - pDepth[i * width + j]) > 20)
                {
                    pDepth[i * width + j] = pOrig[i * width + j];
                }
            }
        }
        free(a_vec);
        free(b_vec);
        free(c_vec);
        free(x_vec);

        free(a2_vec);
        free(b2_vec);
        free(c2_vec);
        free(x2_vec);

        return tmpimg;
    }

    void  Space_Filter::solve_three_point_laplacian_matrix(double x[], const size_t N, const double a[], const double b[], double c[])
    {
        // solve three point laplacian matrix by Gaussian elimination method
        int n;

        c[0] = c[0] / b[0];
        x[0] = x[0] / b[0];

        // loop from 1 to N - 1 inclusive
        for (n = 1; n < N; n++) {
            double m = 1.0f / (b[n] - a[n] * c[n - 1]);
            c[n] = c[n] * m;
            x[n] = (x[n] - a[n] * x[n - 1]) * m;
        }

        // loop from N - 2 to 0 inclusive
        for (n = N - 2; n >= 0; n--)
        {
            x[n] = x[n] - c[n] * x[n + 1];
        }
    }

}
