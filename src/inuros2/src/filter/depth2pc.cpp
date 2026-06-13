#include "opencv2/opencv.hpp"
#include "depth2pc.h"
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/types_c.h>

#ifdef ENABLE_OMP
#include "omp.h"
#endif

using namespace cv;
using namespace std;


namespace Inuchip
{

    inline int cvRodrigues2_(const CvMat* src, CvMat* dst, CvMat* jacobian)
    {
        int depth, elem_size;
        int i, k;
        double J[27];
        CvMat matJ = cvMat(3, 9, CV_64F, J);

        if (!CV_IS_MAT(src))
            CV_Error(!src ? CV_StsNullPtr : CV_StsBadArg, "Input argument is not a valid matrix");

        if (!CV_IS_MAT(dst))
            CV_Error(!dst ? CV_StsNullPtr : CV_StsBadArg,
                "The first output argument is not a valid matrix");

        depth = CV_MAT_DEPTH(src->type);
        elem_size = CV_ELEM_SIZE(depth);

        if (depth != CV_32F && depth != CV_64F)
            CV_Error(CV_StsUnsupportedFormat, "The matrices must have 32f or 64f data type");

        if (!CV_ARE_DEPTHS_EQ(src, dst))
            CV_Error(CV_StsUnmatchedFormats, "All the matrices must have the same data type");

        if (jacobian)
        {
            if (!CV_IS_MAT(jacobian))
                CV_Error(CV_StsBadArg, "Jacobian is not a valid matrix");

            if (!CV_ARE_DEPTHS_EQ(src, jacobian) || CV_MAT_CN(jacobian->type) != 1)
                CV_Error(CV_StsUnmatchedFormats, "Jacobian must have 32fC1 or 64fC1 datatype");

            if ((jacobian->rows != 9 || jacobian->cols != 3) &&
                (jacobian->rows != 3 || jacobian->cols != 9))
                CV_Error(CV_StsBadSize, "Jacobian must be 3x9 or 9x3");
        }

        if (src->cols == 1 || src->rows == 1)
        {
            double rx, ry, rz, theta;
            int step = src->rows > 1 ? src->step / elem_size : 1;

            if (src->rows + src->cols * CV_MAT_CN(src->type) - 1 != 3)
                CV_Error(CV_StsBadSize, "Input matrix must be 1x3, 3x1 or 3x3");

            if (dst->rows != 3 || dst->cols != 3 || CV_MAT_CN(dst->type) != 1)
                CV_Error(CV_StsBadSize, "Output matrix must be 3x3, single-channel floating point matrix");

            if (depth == CV_32F)
            {
                rx = src->data.fl[0];
                ry = src->data.fl[step];
                rz = src->data.fl[step * 2];
            }
            else
            {
                rx = src->data.db[0];
                ry = src->data.db[step];
                rz = src->data.db[step * 2];
            }
            theta = sqrt(rx * rx + ry * ry + rz * rz);

            if (theta < DBL_EPSILON)
            {
                cvSetIdentity(dst);

                if (jacobian)
                {
                    memset(J, 0, sizeof(J));
                    J[5] = J[15] = J[19] = -1;
                    J[7] = J[11] = J[21] = 1;
                }
            }
            else
            {
                const double I[] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

                double c = cos(theta);
                double s = sin(theta);
                double c1 = 1. - c;
                double itheta = theta ? 1. / theta : 0.;

                rx *= itheta; ry *= itheta; rz *= itheta;

                double rrt[] = { rx * rx, rx * ry, rx * rz, rx * ry, ry * ry, ry * rz, rx * rz, ry * rz, rz * rz };
                double _r_x_[] = { 0, -rz, ry, rz, 0, -rx, -ry, rx, 0 };
                double R[9];
                CvMat matR = cvMat(3, 3, CV_64F, R);

                // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
                // where [r_x] is [0 -rz ry; rz 0 -rx; -ry rx 0]
                for (k = 0; k < 9; k++)
                    R[k] = c * I[k] + c1 * rrt[k] + s * _r_x_[k];

                cvConvert(&matR, dst);

                if (jacobian)
                {
                    double drrt[] = { rx + rx, ry, rz, ry, 0, 0, rz, 0, 0,
                        0, rx, 0, rx, ry + ry, rz, 0, rz, 0,
                        0, 0, rx, 0, 0, ry, rx, ry, rz + rz };
                    double d_r_x_[] = { 0, 0, 0, 0, 0, -1, 0, 1, 0,
                        0, 0, 1, 0, 0, 0, -1, 0, 0,
                        0, -1, 0, 1, 0, 0, 0, 0, 0 };
                    for (i = 0; i < 3; i++)
                    {
                        double ri = i == 0 ? rx : i == 1 ? ry : rz;
                        double a0 = -s * ri, a1 = (s - 2 * c1 * itheta) * ri, a2 = c1 * itheta;
                        double a3 = (c - s * itheta) * ri, a4 = s * itheta;
                        for (k = 0; k < 9; k++)
                            J[i * 9 + k] = a0 * I[k] + a1 * rrt[k] + a2 * drrt[i * 9 + k] +
                            a3 * _r_x_[k] + a4 * d_r_x_[i * 9 + k];
                    }
                }
            }
        }
        else if (src->cols == 3 && src->rows == 3)
        {
            double R[9], U[9], V[9], W[3], rx, ry, rz;
            CvMat matR = cvMat(3, 3, CV_64F, R);
            CvMat matU = cvMat(3, 3, CV_64F, U);
            CvMat matV = cvMat(3, 3, CV_64F, V);
            CvMat matW = cvMat(3, 1, CV_64F, W);
            double theta, s, c;
            int step = dst->rows > 1 ? dst->step / elem_size : 1;

            if ((dst->rows != 1 || dst->cols * CV_MAT_CN(dst->type) != 3) &&
                (dst->rows != 3 || dst->cols != 1 || CV_MAT_CN(dst->type) != 1))
                CV_Error(CV_StsBadSize, "Output matrix must be 1x3 or 3x1");

            cvConvert(src, &matR);
            if (!cvCheckArr(&matR, CV_CHECK_RANGE + CV_CHECK_QUIET, -100, 100))
            {
                cvZero(dst);
                if (jacobian)
                    cvZero(jacobian);
                return 0;
            }

            cvSVD(&matR, &matW, &matU, &matV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);
            cvGEMM(&matU, &matV, 1, 0, 0, &matR, CV_GEMM_A_T);

            rx = R[7] - R[5];
            ry = R[2] - R[6];
            rz = R[3] - R[1];

            s = sqrt((rx * rx + ry * ry + rz * rz) * 0.25);
            c = (R[0] + R[4] + R[8] - 1) * 0.5;
            c = c > 1. ? 1. : c < -1. ? -1. : c;
            theta = acos(c);

            if (s < 1e-5)
            {
                double t;

                if (c > 0)
                    rx = ry = rz = 0;
                else
                {
                    t = (R[0] + 1) * 0.5;
                    rx = sqrt(MAX(t, 0.));
                    t = (R[4] + 1) * 0.5;
                    ry = sqrt(MAX(t, 0.)) * (R[1] < 0 ? -1. : 1.);
                    t = (R[8] + 1) * 0.5;
                    rz = sqrt(MAX(t, 0.)) * (R[2] < 0 ? -1. : 1.);
                    if (fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R[5] > 0) != (ry * rz > 0))
                        rz = -rz;
                    theta /= sqrt(rx * rx + ry * ry + rz * rz);
                    rx *= theta;
                    ry *= theta;
                    rz *= theta;
                }

                if (jacobian)
                {
                    memset(J, 0, sizeof(J));
                    if (c > 0)
                    {
                        J[5] = J[15] = J[19] = -0.5;
                        J[7] = J[11] = J[21] = 0.5;
                    }
                }
            }
            else
            {
                double vth = 1 / (2 * s);

                if (jacobian)
                {
                    double t, dtheta_dtr = -1. / s;
                    // var1 = [vth;theta]
                    // var = [om1;var1] = [om1;vth;theta]
                    double dvth_dtheta = -vth * c / s;
                    double d1 = 0.5 * dvth_dtheta * dtheta_dtr;
                    double d2 = 0.5 * dtheta_dtr;
                    // dvar1/dR = dvar1/dtheta*dtheta/dR = [dvth/dtheta; 1] * dtheta/dtr * dtr/dR
                    double dvardR[5 * 9] =
                    {
                        0, 0, 0, 0, 0, 1, 0, -1, 0,
                        0, 0, -1, 0, 0, 0, 1, 0, 0,
                        0, 1, 0, -1, 0, 0, 0, 0, 0,
                        d1, 0, 0, 0, d1, 0, 0, 0, d1,
                        d2, 0, 0, 0, d2, 0, 0, 0, d2
                    };
                    // var2 = [om;theta]
                    double dvar2dvar[] =
                    {
                        vth, 0, 0, rx, 0,
                        0, vth, 0, ry, 0,
                        0, 0, vth, rz, 0,
                        0, 0, 0, 0, 1
                    };
                    double domegadvar2[] =
                    {
                        theta, 0, 0, rx * vth,
                        0, theta, 0, ry * vth,
                        0, 0, theta, rz * vth
                    };

                    CvMat _dvardR = cvMat(5, 9, CV_64FC1, dvardR);
                    CvMat _dvar2dvar = cvMat(4, 5, CV_64FC1, dvar2dvar);
                    CvMat _domegadvar2 = cvMat(3, 4, CV_64FC1, domegadvar2);
                    double t0[3 * 5];
                    CvMat _t0 = cvMat(3, 5, CV_64FC1, t0);

                    cvMatMul(&_domegadvar2, &_dvar2dvar, &_t0);
                    cvMatMul(&_t0, &_dvardR, &matJ);

                    // transpose every row of matJ (treat the rows as 3x3 matrices)
                    CV_SWAP(J[1], J[3], t); CV_SWAP(J[2], J[6], t); CV_SWAP(J[5], J[7], t);
                    CV_SWAP(J[10], J[12], t); CV_SWAP(J[11], J[15], t); CV_SWAP(J[14], J[16], t);
                    CV_SWAP(J[19], J[21], t); CV_SWAP(J[20], J[24], t); CV_SWAP(J[23], J[25], t);
                }

                vth *= theta;
                rx *= vth; ry *= vth; rz *= vth;
            }

            if (depth == CV_32F)
            {
                dst->data.fl[0] = (float)rx;
                dst->data.fl[step] = (float)ry;
                dst->data.fl[step * 2] = (float)rz;
            }
            else
            {
                dst->data.db[0] = rx;
                dst->data.db[step] = ry;
                dst->data.db[step * 2] = rz;
            }
        }

        if (jacobian)
        {
            if (depth == CV_32F)
            {
                if (jacobian->rows == matJ.rows)
                    cvConvert(&matJ, jacobian);
                else
                {
                    float Jf[3 * 9];
                    CvMat _Jf = cvMat(matJ.rows, matJ.cols, CV_32FC1, Jf);
                    cvConvert(&matJ, &_Jf);
                    cvTranspose(&_Jf, jacobian);
                }
            }
            else if (jacobian->rows == matJ.rows)
                cvCopy(&matJ, jacobian);
            else
                cvTranspose(&matJ, jacobian);
        }

        return 1;
    }

    inline void Rodrigues_(cv::InputArray _src, cv::OutputArray _dst, cv::OutputArray _jacobian = cv::noArray())
    {
        cv::Mat src = _src.getMat();
        bool v2m = src.cols == 1 || src.rows == 1;
        _dst.create(3, v2m ? 3 : 1, src.depth());
        cv::Mat dst = _dst.getMat();
#if defined(CV_MAJOR_VERSION) && (CV_MAJOR_VERSION > 3)
        CvMat _csrc = cvMat(src), _cdst = cvMat(dst), _cjacobian;
#else
        CvMat _csrc = src, _cdst = dst, _cjacobian;
#endif
        if (_jacobian.needed())
        {
            _jacobian.create(v2m ? cv::Size(9, 3) : cv::Size(3, 9), src.depth());
#if defined(CV_MAJOR_VERSION) && (CV_MAJOR_VERSION > 3)
            _cjacobian = cvMat(_jacobian.getMat());
#else
            _cjacobian = _jacobian.getMat();
#endif
        }
        bool ok = cvRodrigues2_(&_csrc, &_cdst, _jacobian.needed() ? &_cjacobian : 0) > 0;
        if (!ok) {
            dst = cv::Scalar(0);
        }
    }

    /// Convert orientation angles stored in rodrigues conventions to rotation matrix
    /// for details: http://mesh.brown.edu/en193s08-2003/notes/en193s08-rots.pdf
    cv::Mat calc_rotation_from_angles(const cv::Point3f& rot)
    {

        cv::Mat rot_mat = Mat::zeros(3, 3, CV_32F);

        double theta = sqrt(rot.x * rot.x + rot.y * rot.y + rot.z * rot.z);
        double r1 = rot.x, r2 = rot.y, r3 = rot.z;
        const double SQRT_DBL_EPSILON = sqrt(std::numeric_limits<double>::epsilon());

        if (theta <= SQRT_DBL_EPSILON) // identityMatrix
        {
            rot_mat.at<float>(0, 0) = rot_mat.at<float>(1, 1) = rot_mat.at<float>(2, 2) = 1.0;
            rot_mat.at<float>(0, 1) = rot_mat.at<float>(0, 2) = rot_mat.at<float>(1, 0) = rot_mat.at<float>(1, 2) = rot_mat.at<float>(2, 0) = rot_mat.at<float>(2, 1) = 0.0;
        }
        else
        {
            r1 /= theta;
            r2 /= theta;
            r3 /= theta;

            double c = cos(theta);
            double s = sin(theta);
            double g = 1 - c;

            rot_mat.at<float>(0, 0) = float(c + g * r1 * r1);
            rot_mat.at<float>(0, 1) = float(g * r1 * r2 - s * r3);
            rot_mat.at<float>(0, 2) = float(g * r1 * r3 + s * r2);
            rot_mat.at<float>(1, 0) = float(g * r2 * r1 + s * r3);
            rot_mat.at<float>(1, 1) = float(c + g * r2 * r2);
            rot_mat.at<float>(1, 2) = float(g * r2 * r3 - s * r1);
            rot_mat.at<float>(2, 0) = float(g * r3 * r1 - s * r2);
            rot_mat.at<float>(2, 1) = float(g * r3 * r2 + s * r1);
            rot_mat.at<float>(2, 2) = float(c + g * r3 * r3);
        }

        return rot_mat;
    }

    void COptData::LoadYaml(const std::string& fname)
    {
        cv::FileStorage fs(fname, FileStorage::READ);
        if (!fs.isOpened())
        {
            std::cout << "Can't find " << fname << std::endl;
            return;
        }

        fs["Depth_fx"] >> focalDepth[0];
        fs["Depth_fy"] >> focalDepth[1];

        fs["Depth_cx"] >> centerDepth[0];
        fs["Depth_cy"] >> centerDepth[1];

        fs["Depth_base"] >> baselineDepth;

        fs["Web_fx"] >> focalWeb[0];
        fs["Web_fy"] >> focalWeb[1];

        fs["Web_cx"] >> centerWeb[0];
        fs["Web_cy"] >> centerWeb[1];

        for (int i = 0; i < 5; i++)
        {
            fs["Web_K" + std::to_string(i)] >> KdWeb[i];
        }

        for (int i = 0; i < 2; i++)
        {
            fs["Web_uv" + std::to_string(i)] >> uvWeb[i];
        }

        for (int i = 0; i < 3; i++)
        {
            fs["Web_tran" + std::to_string(i)] >> WebcamTranslate[i];
        }

        fs["Web_width"] >> sizeWeb.width;
        fs["Web_height"] >> sizeWeb.height;

        fs["rotAngle_x"] >> rotAngle.x;
        fs["rotAngle_y"] >> rotAngle.y;
        fs["rotAngle_z"] >> rotAngle.z;
        fs.release();

        Init();

    }
    void COptData::DumpYaml(const std::string& fname)
    {
        cv::FileStorage fs(fname, FileStorage::WRITE);

        fs << "Depth_fx" << focalDepth[0];
        fs << "Depth_fy" << focalDepth[1];

        fs << "Depth_cx" << centerDepth[0];
        fs << "Depth_cy" << centerDepth[1];

        fs << "Depth_base" << baselineDepth;

        fs << "Web_fx" << focalWeb[0];
        fs << "Web_fy" << focalWeb[1];

        fs << "Web_cx" << centerWeb[0];
        fs << "Web_cy" << centerWeb[1];

        for (int i = 0; i < 5; i++)
        {
            fs << "Web_K" + std::to_string(i) << KdWeb[i];
        }

        for (int i = 0; i < 2; i++)
        {
            fs << "Web_uv" + std::to_string(i) << uvWeb[i];
        }

        for (int i = 0; i < 3; i++)
        {
            fs << "Web_tran" + std::to_string(i) << WebcamTranslate[i];
        }

        fs << "Web_width" << sizeWeb.width;
        fs << "Web_height" << sizeWeb.height;
        fs << "rotAngle_x" << rotAngle.x;
        fs << "rotAngle_y" << rotAngle.y;
        fs << "rotAngle_z" << rotAngle.z;
        fs.release();
    }

    COptData::COptData()
    {
        uvWeb[0] = uvWeb[1] = 0.0f;
    }

    void COptData::Init()
    {
        inversefocalDepth[0] = 1 / focalDepth[0];
        inversefocalDepth[1] = 1 / focalDepth[1];

        Mat rotat_vectr = Mat::zeros(3, 1, CV_32F);
        rotRightWeb = Mat::zeros(3, 3, CV_32F);
        rotat_vectr.at<float>(0, 0) = float(rotAngle.x);
        rotat_vectr.at<float>(1, 0) = float(rotAngle.y);
        rotat_vectr.at<float>(2, 0) = float(rotAngle.z);
        Rodrigues_(rotat_vectr, rotRightWeb);
        calcR();
        calcR_fix();
    }

    void COptData::calcR()
    {
        // calculate paramaters to save compuatation, this function should be called in init function
        float* rdata = (float*)rotRightWeb.data;
        r[0] = rdata[0] * focalWeb[0] * inversefocalDepth[0];
        r[1] = rdata[1] * focalWeb[0] * inversefocalDepth[1];
        r[2] = (rdata[2] - rdata[0] * centerDepth[0] * inversefocalDepth[0] - rdata[1] * centerDepth[1] * inversefocalDepth[1]) * focalWeb[0];
        r[3] = WebcamTranslate[0];
        r[4] = rdata[3] * focalWeb[1] * float(inversefocalDepth[0]);
        r[5] = rdata[4] * focalWeb[1] * float(inversefocalDepth[1]);
        r[6] = (rdata[5] - rdata[3] * centerDepth[0] * inversefocalDepth[0] - rdata[4] * centerDepth[1] * inversefocalDepth[1]) * focalWeb[1];
        r[7] = WebcamTranslate[1];
        r[8] = rdata[6] * inversefocalDepth[0];
        r[9] = rdata[7] * inversefocalDepth[1];
        r[10] = (rdata[8] - rdata[6] * centerDepth[0] * inversefocalDepth[0] - rdata[7] * centerDepth[1] * inversefocalDepth[1]);
        r[11] = WebcamTranslate[2];
        r[12] = focalWeb[0];
        r[13] = focalWeb[1];
        for (int i = 0; i < 14; i++)
        {
            printf("raw %d-->%f\n", i, r[i]);
        }
    }

    void COptData::calcR_fix()
    {
        int zfix[4];
        int zfactor[3];
        int ufix[5];
        int ufactor[3];
        int vfix[5];
        int vfactor[3];
        int maxPosInt = 1073741823;
        int maxNegInt = -1073741823;
        int maxZshift = 14; // max z is 2^14
        int maxidx = 2000;
        float rtmp[13];
        memcpy(rtmp, r, 13 * sizeof(float));
        rtmp[0] *= maxidx;
        rtmp[1] *= maxidx;
        rtmp[4] *= maxidx;
        rtmp[5] *= maxidx;
        rtmp[8] *= maxidx;
        rtmp[9] *= maxidx;

        // params for z

        float maxr = 0, minr = 0;
        for (int i = 8; i <= 10; i++)
        {
            maxr = maxr > rtmp[i] ? maxr : rtmp[i];
            minr = minr < rtmp[i] ? minr : rtmp[i];
        }
        float f1 = maxPosInt / maxr, f2 = maxNegInt / minr;

        float factor = floorf(f1 < f2 ? f1 : f2);
        if (f1 < 0)
            factor = f2;
        if (f2 < 0)
            factor = f1;

        int fixfactor = (int)factor;
        int ct = 0;
        while (fixfactor)
        {
            fixfactor >>= 1;
            if (fixfactor == 0)
            {
                break;
            }
            ct++;
        }

        int cnt = 0;
        for (int i = 8; i <= 10; i++)
        {
            zfix[cnt++] = int(r[i] * (1 << ct));
        }
        zfactor[0] = ct;
        zfactor[1] = maxZshift;
        zfactor[2] = ct - maxZshift;
        zfix[cnt] = int(r[11] *(1 << zfactor[2]));


        // params for u
        maxr = 0, minr = 0;
        for (int i = 0; i <= 2; i++)
        {
            maxr = maxr > rtmp[i] ? maxr : rtmp[i];
            minr = minr < rtmp[i] ? minr : rtmp[i];
        }
        f1 = maxPosInt / maxr, f2 = maxNegInt / minr;

        factor = floorf(f1 < f2 ? f1 : f2);
        if (f1 < 0)
            factor = f2;
        if (f2 < 0)
            factor = f1;

        fixfactor = (int)factor;
        ct = 0;
        while (fixfactor)
        {
            fixfactor >>= 1;
            if (fixfactor == 0)
            {
                break;
            }
            ct++;
        }

        cnt = 0;
        for (int i = 0; i <= 2; i++)
        {
            ufix[cnt++] = int(r[i] * (1 << ct));
        }
        ufactor[0] = ct;

        maxr = centerWeb[0] > (r[3] * r[12]) ? centerWeb[0] : (r[3] * r[12]);
        minr = centerWeb[0] < (r[3] * r[12]) ? centerWeb[0] : (r[3] * r[12]);
        f1 = maxPosInt / maxr, f2 = maxNegInt / minr;

        factor = floorf(f1 < f2 ? f1 : f2);
        if (f1 < 0)
            factor = f2;
        if (f2 < 0)
            factor = f1;

        fixfactor = (int)factor;
        ct = 0;
        while (fixfactor)
        {
            fixfactor >>= 1;
            if (fixfactor == 0)
            {
                break;
            }
            ct++;
        }

        ufactor[2] = ct< ufactor[0]?ct: ufactor[0];
        ufactor[1] = ufactor[0]- ufactor[2];
        ufix[cnt++] = int(r[12]*r[3] * (1 << ufactor[2]));
        ufix[cnt]	= int(centerWeb[0] * (1 << ufactor[2]));

        // params for u
        maxr = 0, minr = 0;
        for (int i = 4; i <= 6; i++)
        {
            maxr = maxr > rtmp[i] ? maxr : rtmp[i];
            minr = minr < rtmp[i] ? minr : rtmp[i];
        }
        f1 = maxPosInt / maxr, f2 = maxNegInt / minr;

        factor = floorf(f1 < f2 ? f1 : f2);
        if (f1 < 0)
            factor = f2;
        if (f2 < 0)
            factor = f1;

        fixfactor = (int)factor;
        ct = 0;
        while (fixfactor)
        {
            fixfactor >>= 1;
            if (fixfactor == 0)
            {
                break;
            }
            ct++;
        }

        cnt = 0;
        for (int i = 4; i <= 6; i++)
        {
            vfix[cnt++] = int(r[i] * (1 << ct));
        }
        vfactor[0] = ct;
        maxr = centerWeb[1] > (r[7] * r[13]) ? centerWeb[1] : (r[7] * r[13]);
        minr = centerWeb[1] < (r[7] * r[13]) ? centerWeb[1] : (r[7] * r[13]);
        f1 = maxPosInt / maxr, f2 = maxNegInt / minr;

        factor = floorf(f1 < f2 ? f1 : f2);
        if (f1 < 0)
            factor = f2;
        if (f2 < 0)
            factor = f1;

        fixfactor = (int)factor;
        ct = 0;
        while (fixfactor)
        {
            fixfactor >>= 1;
            if (fixfactor == 0)
            {
                break;
            }
            ct++;
        }

        vfactor[2] = ct < vfactor[0] ? ct : vfactor[0];
        vfactor[1] = vfactor[0] - vfactor[2];
        vfix[cnt++] = int(r[13] * r[7] * (1 << vfactor[2]));
        vfix[cnt] = int(centerWeb[1] * (1 << vfactor[2]));

        for (int i = 0; i < 4; i++)
        {
            printf("zfix[%d]-->%d\n", i, zfix[i]);
        }
        for (int i = 0; i < 3; i++)
        {
            printf("zfacotr[%d]-->%d\n", i, zfactor[i]);
        }
        for (int i = 0; i < 5; i++)
        {
            printf("ufix[%d]-->%d\n", i, ufix[i]);
        }
        for (int i = 0; i < 3; i++)
        {
            printf("ufactor[%d]-->%d\n", i, ufactor[i]);
        }
        for (int i = 0; i < 5; i++)
        {
            printf("vfix[%d]-->%d\n", i, vfix[i]);
        }
        for (int i = 0; i < 3; i++)
        {
            printf("vfactor[%d]-->%d\n", i, vfactor[i]);
        }
        memcpy(rfix, zfix, 4 * sizeof(int));
        memcpy(&rfix[4], zfactor, 3 * sizeof(int));
        memcpy(&rfix[7], ufix, 5 * sizeof(int));
        memcpy(&rfix[12], ufactor, 3 * sizeof(int));
        memcpy(&rfix[15], vfix, 5 * sizeof(int));
        memcpy(&rfix[20], vfactor, 3 * sizeof(int));
    }

    void Depth2Ply(const cv::Mat& imgDep, const cv::Mat& imgRgb, const COptData& mOD, std::string& sBuf, float depthscale)
    {
        float x0 = mOD.centerWeb[0];
        float y0 = mOD.centerWeb[1];
        float fx = mOD.focalWeb[0];
        float fy = mOD.focalWeb[1];


        std::vector<cv::Point3f> vAll;
        std::vector<cv::Vec3b> vRGB;
        for (int y = 0; y < imgDep.rows; y++)
        {
            for (int x = 0; x < imgDep.cols; x++)
            {
                float z = imgDep.at<ushort>(y, x) / depthscale;
                if (z < 10) continue;

                float xw = z * (x - x0) / fx;
                float yw = z * (y - y0) / fy;
                vAll.emplace_back(cv::Point3f(xw, yw, z));

                cv::Vec3b irgb = imgRgb.at<Vec3b>(y, x);
                vRGB.emplace_back(irgb);
            }

        }
        if (vAll.size() == 0)
        {
            std::cout << "Empty depth\n";
            return;
        }

        std::ofstream  recordFile(sBuf);

        recordFile << "ply" << endl;
        recordFile << "format ascii 1.0" << endl;
        recordFile << "element vertex " << vAll.size() << endl;
        recordFile << "property float x" << endl;
        recordFile << "property float y" << endl;
        recordFile << "property float z" << endl;
        recordFile << "property uchar red" << endl;
        recordFile << "property uchar green" << endl;
        recordFile << "property uchar blue" << endl;
        recordFile << "end_header" << endl;

        for (size_t i = 0; i < vAll.size(); i++)
        {
            recordFile << vAll[i].x << " " << vAll[i].y << " " << vAll[i].z << " ";
            recordFile << int(vRGB[i][2]) << " " << int(vRGB[i][1]) << " " << int(vRGB[i][0]) << std::endl;
        }

        recordFile.close();
        //std::cout << sBuf << " saved!\n";
    }
    cv::Point3f Depth2Web_Pt_Dist(float Ix, float Iy, float Z, const COptData& OD, float& x3D, float& y3D)
    {
        x3D = ((float)Ix - OD.centerDepth[0]) * Z * OD.inversefocalDepth[0];
        y3D = ((float)Iy - OD.centerDepth[1]) * Z * OD.inversefocalDepth[1];

        float* rdata = (float*)OD.rotRightWeb.data;
        float x_RGB = rdata[0] * x3D + rdata[1] * y3D + rdata[2] * Z + OD.WebcamTranslate[0];
        float y_RGB = rdata[3] * x3D + rdata[4] * y3D + rdata[5] * Z + OD.WebcamTranslate[1];
        float z_RGB = rdata[6] * x3D + rdata[7] * y3D + rdata[8] * Z + OD.WebcamTranslate[2];

        float xnorm = x_RGB / z_RGB;
        float ynorm = y_RGB / z_RGB;
        float xnorm2 = xnorm * xnorm;
        float ynorm2 = ynorm * ynorm;
        float distance2 = xnorm2 + ynorm2;
        float distance4 = distance2 * distance2;

        float radial_distort = 1.0f + OD.KdWeb[0] * distance2 + distance4 * (OD.KdWeb[1] + OD.KdWeb[4] * distance2);

        float xDistorted = xnorm * radial_distort;
        float yDistorted = ynorm * radial_distort;

        // Tangential distortion ...
        float xynorm = xnorm * ynorm;
        float xtangential = 2 * OD.KdWeb[2] * xynorm + OD.KdWeb[3] * (distance2 + 2 * xnorm2);
        float ytangential = 2 * OD.KdWeb[3] * xynorm + OD.KdWeb[2] * (distance2 + 2 * ynorm2);

        // Final re-distortion:
        xDistorted += xtangential;  yDistorted += ytangential;

        float u_distort = OD.centerWeb[0] + OD.focalWeb[0] * xDistorted + OD.uvWeb[0];
        float v_distort = OD.centerWeb[1] + OD.focalWeb[1] * yDistorted + OD.uvWeb[1];

        return cv::Point3f(u_distort, v_distort, z_RGB);
    }

    cv::Point3f Depth2Web_Pt(float Ix, float Iy, float Z, const COptData& OD, float& x3D, float& y3D)
    {
        float z_RGB = (OD.r[8] * Ix + OD.r[9] * Iy + OD.r[10]) * Z + OD.r[11];
        double abs_z_RGB = fabs(z_RGB);
        if (abs_z_RGB < 1.e-10) {
            z_RGB = (float)(abs_z_RGB * 1.e-3);
            return cv::Point3f(-1, -1, z_RGB);
        }

        float u_distort = ((OD.r[0] * Ix + OD.r[1] * Iy + OD.r[2]) * Z + OD.r[3] * OD.r[12]) / z_RGB + OD.centerWeb[0];
        float v_distort = ((OD.r[4] * Ix + OD.r[5] * Iy + OD.r[6]) * Z + OD.r[7] * OD.r[13]) / z_RGB + OD.centerWeb[1];

        return cv::Point3f(u_distort, v_distort, z_RGB);
    }

    cv::Point3d Depth2Web_Pt_fix(int Ix, int Iy, int Z, const COptData& OD)
    {
        int  z_RGB = (((OD.rfix[0] * Ix + OD.rfix[1] * Iy + OD.rfix[2]) >> OD.rfix[5]) * Z) + OD.rfix[3];
        z_RGB = ((z_RGB + (1<<(OD.rfix[6]-1)))>>OD.rfix[6]);
        int abs_z_RGB = abs(z_RGB);
        if (abs_z_RGB < 1) {
            return cv::Point3d(-1, -1, 0);
        }

        int u = ((((OD.rfix[7] * Ix + OD.rfix[8] * Iy + OD.rfix[9]) / z_RGB )>>OD.rfix[13]) * Z + OD.rfix[10] / z_RGB) + OD.rfix[11];
        int v = ((((OD.rfix[15] * Ix + OD.rfix[16] * Iy + OD.rfix[17]) / z_RGB) >> OD.rfix[21]) * Z + OD.rfix[18] / z_RGB) + OD.rfix[19];
        u = ((u + (1 << (OD.rfix[14] - 1))) >> OD.rfix[14]);
        v = ((v + (1 << (OD.rfix[22] - 1))) >> OD.rfix[22]);
        return cv::Point3d((int)u, (int)v, (int)z_RGB);
    }


    cv::Point3f Depth2Web_Pt_prev(float Ix, float Iy, float Z, const COptData& OD, float& x3D, float& y3D)
    {
        // previous version, simple logic but slow
        x3D = (Ix - float(OD.centerDepth[0])) * Z * float(OD.inversefocalDepth[0]);
        y3D = (Iy - float(OD.centerDepth[1])) * Z * float(OD.inversefocalDepth[1]);


        float* rdata = (float*)OD.rotRightWeb.data;
        float x_RGB = rdata[0] * x3D + rdata[1] * y3D + rdata[2] * Z + float(OD.WebcamTranslate[0]);
        float y_RGB = rdata[3] * x3D + rdata[4] * y3D + rdata[5] * Z + float(OD.WebcamTranslate[1]);
        float z_RGB = rdata[6] * x3D + rdata[7] * y3D + rdata[8] * Z + float(OD.WebcamTranslate[2]);

        double abs_z_RGB = fabs(z_RGB);

        /***************** end rotation translation ******************************************/

        if (abs_z_RGB < 1.e-10) {
            z_RGB = (float)(abs_z_RGB * 1.e-3);
        }

        float xnorm = x_RGB / z_RGB;
        float ynorm = y_RGB / z_RGB;

        float u_distort = xnorm * OD.focalWeb[0] + OD.centerWeb[0];
        float v_distort = ynorm * OD.focalWeb[1] + OD.centerWeb[1];

        return cv::Point3f(u_distort, v_distort, z_RGB);
    }

    std::shared_ptr<cv::Mat> Depth2Colorfix(const cv::Mat& imgDepth, COptData& mOD, cv::Mat& imgVal, cv::Mat& imgIdx, bool closeHole, int zMin, int zMax, float depthscale_in, float depthscale_out)
    {

        std::shared_ptr<cv::Mat> depAlign = std::make_shared<cv::Mat>(mOD.sizeWeb, imgDepth.type(), cv::Scalar(0));
        cv::Mat mask(mOD.sizeWeb, CV_8UC1, cv::Scalar(0));
        int widthIn = imgDepth.cols, heightIn = imgDepth.rows, widthOut = depAlign->cols, heightOut = depAlign->rows;

        ushort* ptrIn = (ushort*)imgDepth.ptr<ushort>();
        ushort* ptrOut = (ushort*)depAlign->ptr<ushort>();
        uchar* ptrmask = (uchar*)mask.ptr<uchar>();

        ushort* ptrVal = (ushort*)imgVal.ptr<ushort>();
        ushort* ptrIdx = (ushort*)imgIdx.ptr<ushort>();
#ifdef ENABLE_OMP
        omp_set_num_threads(4);
#pragma omp parallel for
#endif
        int max_dr = 0;
        for (int r = 0; r < heightIn; r++)
        {
            int max_r = 0;
            int min_r = heightOut;
            for (int c = 0; c < widthIn; c++)
            {
                int id = r * widthIn + c;
                float z = ptrIn[id] / depthscale_in;
                if (z < zMin || z > zMax)
                {
                    continue;
                }
                float x3d, y3d;
                cv::Point3d uv = Depth2Web_Pt_fix(c, r, z, mOD);
                int dx = int(uv.x);
                int dy = int(uv.y);
                if (dx >= 0 && dx < widthOut && dy >= 0 && dy < heightOut)
                {
                    min_r = min(min_r, dy);
                    max_r = max(max_r, dy);
                    int id2 = dy * widthOut + dx;
                    unsigned short z_curr_ = (unsigned short)(uv.z* depthscale_out);
                    unsigned short z2 = ptrOut[id2];
                    ptrVal[id] = z_curr_;
                    ptrIdx[id] = id2;
                    if (z2 == 0 || z_curr_ < z2)
                    {
                        ptrOut[id2] = z_curr_;
                        ptrmask[id2] = 1;
                    }
                }
            }
            max_dr = max(max_dr, max_r - min_r);
        }
        printf("max_dr = %d ", max_dr);

        if (closeHole)
        {
            int W_h = 1;
            int H_h = 1;
#ifdef ENABLE_OMP
#pragma omp parallel for schedule(dynamic)
#endif
            for (int row = 0; row < heightOut; row++)
            {
                for (int col = 0; col < widthOut; col++)
                {
                    if (mask.at<unsigned char>(row, col) == 0)
                    {
                        unsigned short minDist = 0xffffU;
                        for (int m = -H_h; m <= H_h; m++)
                        {
                            if ((row + m) < 0 || (row + m) >= heightOut)
                                continue;
                            for (int n = -W_h; n <= W_h; n++)
                            {
                                if ((col + n) < 0 || (col + n) >= widthOut)
                                    continue;
                                int id = (row + m) * widthOut + col + n;
                                int id2 = row * widthOut + col;

                                if (ptrmask[id] != 0 && (ptrOut[id] < minDist))
                                {
                                    minDist = ptrOut[id];
                                    ptrOut[id2] = minDist;
                                }
                            }
                        }
                    }
                }
            }
        }

        return depAlign;
    }
    std::shared_ptr<cv::Mat> Depth2Color(const cv::Mat& imgDepth, COptData& mOD, bool closeHole, int zMin, int zMax, float depthscale_in, float depthscale_out)
    {

        std::shared_ptr<cv::Mat> depAlign = std::make_shared<cv::Mat>(mOD.sizeWeb, imgDepth.type(), cv::Scalar(0));
        cv::Mat mask(mOD.sizeWeb, CV_8UC1, cv::Scalar(0));
        int widthIn = imgDepth.cols, heightIn = imgDepth.rows, widthOut = depAlign->cols, heightOut = depAlign->rows;

        ushort* ptrIn = (ushort*)imgDepth.ptr<ushort>();
        ushort* ptrOut = (ushort*)depAlign->ptr<ushort>();
        uchar* ptrmask = (uchar*)mask.ptr<uchar>();
#ifdef ENABLE_OMP
        omp_set_num_threads(4);
#pragma omp parallel for
#endif
        for (int r = 0; r < heightIn; r++)
        {
            for (int c = 0; c < widthIn; c++)
            {
                int id = r * widthIn + c;
                float z = ptrIn[id] / depthscale_in;
                if (z < zMin || z > zMax)
                {
                    continue;
                }
                float x3d, y3d;
                cv::Point3f uv = Depth2Web_Pt(c, r, z, mOD, x3d, y3d);
                int dx = int(uv.x);
                int dy = int(uv.y);
                if (dx >= 0 && dx < widthOut && dy >= 0 && dy < heightOut)
                {
                    int id2 = dy * widthOut + dx;
                    unsigned short z_curr_ = (unsigned short)(uv.z * depthscale_out);
                    unsigned short z2 = ptrOut[id2];
                    if (z2 == 0 || z_curr_ < z2)
                    {
                        ptrOut[id2] = z_curr_;
                        ptrmask[id2] = 1;
                    }
                }
            }
        }

        if (closeHole)
        {
            int W_h = 1;
            int H_h = 1;
#ifdef ENABLE_OMP
#pragma omp parallel for schedule(dynamic)
#endif
            for (int row = 0; row < heightOut; row++)
            {
                for (int col = 0; col < widthOut; col++)
                {
                    if (mask.at<unsigned char>(row, col) == 0)
                    {
                        unsigned short minDist = 0xffffU;
                        for (int m = -H_h; m <= H_h; m++)
                        {
                            if ((row + m) < 0 || (row + m) >= heightOut)
                                continue;
                            for (int n = -W_h; n <= W_h; n++)
                            {
                                if ((col + n) < 0 || (col + n) >= widthOut)
                                    continue;
                                int id = (row + m) * widthOut + col + n;
                                int id2 = row * widthOut + col;

                                if (ptrmask[id] != 0 && (ptrOut[id] < minDist))
                                {
                                    minDist = ptrOut[id];
                                    ptrOut[id2] = minDist;
                                }
                            }
                        }
                    }
                }
            }
        }
        return depAlign;
    }



    std::shared_ptr<cv::Mat> Color2Depth(const cv::Mat& imgDepth, const cv::Mat& imgWeb, COptData& mOD, int zMin, int zMax)
    {

        std::shared_ptr<cv::Mat> colAlign = std::make_shared<cv::Mat>(imgDepth.size(), imgWeb.type(), cv::Scalar(0));

        int width = colAlign->cols, height = colAlign->rows, rgbw = imgWeb.cols, rgbh = imgWeb.rows;
        ushort* ptrDep = (ushort*)imgDepth.ptr<ushort>(0);
        uchar* ptrRgb = (uchar*)imgWeb.ptr<uchar>(0);
        uchar* ptrRet = (uchar*)colAlign->ptr<uchar>(0);
        int cNum = imgWeb.channels();
        for (int r = 0; r < height; r++)
        {
            for (int c = 0; c < width; c++)
            {
                int idx = r * width + c;
                int z = ptrDep[idx];
                if (z < zMin || z > zMax)
                {
                    continue;
                }
                float x3d, y3d;
                cv::Point3f uv = Depth2Web_Pt(c, r, z, mOD, x3d, y3d);
                int dx = int(uv.x);
                int dy = int(uv.y);

                if ((dx >= 0 && dx < rgbw) && (dy >= 0 && dy < rgbh))
                {
                    int idx2 = r * width * cNum + c * cNum;
                    int idx3 = dy * rgbw * cNum + dx * cNum;
                    memcpy(&ptrRet[idx2], &ptrRgb[idx3], cNum * sizeof(uchar));
                }

            }
        }

        return colAlign;
    }

    cv::Point2f Depth2Web_rs(float x, float y, float z, float* pOD) {
        float x3D = (x - pOD[0]) * z * pOD[2];
        float y3D = (y - pOD[1]) * z * pOD[3];

        float x_RGB = pOD[8] * x3D + pOD[9] * y3D + pOD[10] * z + pOD[17];
        float y_RGB = pOD[11] * x3D + pOD[12] * y3D + pOD[13] * z + pOD[18];
        float z_RGB = pOD[14] * x3D + pOD[15] * y3D + pOD[16] * z + pOD[19];
        double abs_z_RGB = fabs(z_RGB);
        if (abs_z_RGB < 1.e-10) {
            z_RGB = (float)(abs_z_RGB * 1.e-3);
        }

        float xnorm = x_RGB / z_RGB;
        float ynorm = y_RGB / z_RGB;

        float u_distort = xnorm * pOD[6] + pOD[4];
        float v_distort = ynorm * pOD[7] + pOD[5];

        return cv::Point2f(u_distort, v_distort);
    }

    std::shared_ptr<cv::Mat> Depth2Color_rs(const cv::Mat& imgDepth, COptData& mOD, int zMin, int zMax) {
        std::shared_ptr<cv::Mat> depAlign =
            std::make_shared<cv::Mat>(mOD.sizeWeb, imgDepth.type(), cv::Scalar(0));

        int iw = imgDepth.cols;
        int ih = imgDepth.rows;
        int ow = depAlign->cols;
        int oh = depAlign->rows;

        ushort* p_in = (ushort*)imgDepth.ptr<ushort>();
        ushort* p_out = (ushort*)depAlign->ptr<ushort>();
        uint32_t* arr = (uint32_t*)p_out;
        memset(p_out, 0xff, ow * oh * sizeof(ushort));

        // int* map0 = new int[iw * ih];
        // int* map1 = new int[iw * ih];

        float pOD[20];
        pOD[0] = mOD.centerDepth[0];
        pOD[1] = mOD.centerDepth[1];
        pOD[2] = mOD.inversefocalDepth[0];
        pOD[3] = mOD.inversefocalDepth[1];
        pOD[4] = mOD.centerWeb[0];
        pOD[5] = mOD.centerWeb[1];
        pOD[6] = mOD.focalWeb[0];
        pOD[7] = mOD.focalWeb[1];
        float* rdata = (float*)mOD.rotRightWeb.data;
        for (int i = 0; i < 9; i++) {
            pOD[8 + i] = rdata[i];
        }
        for (int i = 0; i < 3; i++) {
            pOD[17 + i] = mOD.WebcamTranslate[i];
        }

        for (int i = 0; i < ih; ++i) {
            for (int j = 0; j < iw; ++j) {
                int id = i * iw + j;
                int z = p_in[id];
                if (z < zMin || z > zMax) {
                    // map0[id] = -1;
                    // map1[id] = -1;
                    continue;
                }
                cv::Point2f p0 = Depth2Web_rs(j - 0.5f, i - 0.5f, z, pOD);
                cv::Point2f p1 = Depth2Web_rs(j + 0.5f, i + 0.5f, z, pOD);
                // map0[id] = p0.y * ow + p0.x;
                // map1[id] = p1.y * ow + p1.x;
                int y0 = (int)p0.y;
                int y1 = (int)p1.y;
                int x0 = (int)p0.x;
                int x1 = (int)p1.x;
                uint32_t newval = p_in[id];
                for (int y = y0; y < y1; ++y) {
                    for (int x = x0; x < x1; ++x) {
                        if (y < 0 || y >= oh || x < 0 || x >= ow)
                            continue;
                        auto other_pixel_index = y * ow + x;
                        newval = newval << 16 | newval;
                        arr[other_pixel_index / 2] = min(arr[other_pixel_index / 2], newval);
                    }
                }
            }
        }

        for (int i = 0; i < ow * oh; i++) {
            if (p_out[i] == 0xffff)
                p_out[i] = 0;
        }

        // delete[] map0;
        // delete[] map1;
        return depAlign;
    }
}


