#pragma once
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Rect centerROI(const Mat& m, int w, int h)
{
    int x = (m.cols - w) / 2;
    int y = (m.rows - h) / 2;
    return Rect(x, y, w, h);
}

bool estimateShift(const Mat& img1,
                   const Mat& img2,
                   int roiW,
                   int roiH,
                   Point2d& shift,
                   Mat& affine_out)
{
    Rect r = centerROI(img1, roiW, roiH);

    Mat roi1 = img1(r);
    Mat roi2 = img2(r);

    Mat g1, g2;
    cvtColor(roi1, g1, COLOR_BGR2GRAY);
    cvtColor(roi2, g2, COLOR_BGR2GRAY);

    // ---------- feature extraction ----------
    std::vector<cv::Point2f> pts1;

    const int GRID=8;
    int gw=g1.cols/GRID;
    int gh=gw*g1.rows/g1.cols;

    for(int y=0;y<GRID;y++)
        for(int x=0;x<GRID;x++)
        {
            cv::Rect r(x*gw,y*gh,gw,gh);
            std::vector<cv::Point2f> tmp;

            cv::goodFeaturesToTrack(
                g1(r), tmp,
                10,
                0.001,
                5);

            for(auto&p:tmp)
                pts1.push_back(p+cv::Point2f(r.x,r.y));
        }

    // FAST fallback
    if(pts1.size()<80)
    {
        std::vector<cv::KeyPoint> kp;
        cv::FAST(g1,kp,20);
        for(auto&k:kp) pts1.push_back(k.pt);
    }

    if(pts1.size()<30) return false;

    // ---------- LK forward ----------
    std::vector<cv::Point2f> pts2;
    std::vector<uchar> st;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(
        g1,g2,
        pts1,pts2,
        st,err,
        cv::Size(21,21),3);

    // ---------- backward ----------
    std::vector<cv::Point2f> pts1b;
    std::vector<uchar> stb;
    std::vector<float> errb;

    cv::calcOpticalFlowPyrLK(
        g2,g1,
        pts2,pts1b,
        stb,errb,
        cv::Size(21,21),3);

    // ---------- FB filter ----------
    std::vector<cv::Point2f> p1,p2;

    for(int i=0;i<pts1.size();i++)
        if(st[i]&&stb[i])
            if(cv::norm(pts1[i]-pts1b[i])<1.0)
            {
                p1.push_back(pts1[i]);
                p2.push_back(pts2[i]);
            }

    if(p1.size()<20) return false;

    // ---------- affine ----------
    cv::Mat inl;
    cv::Mat A=cv::estimateAffinePartial2D(p1,p2,inl,cv::RANSAC);

    if(A.empty()) return false;

    affine_out=A.clone();

    cv::Point2f c(g1.cols*0.5f,g1.rows*0.5f);

    double nx=A.at<double>(0,0)*c.x+A.at<double>(0,1)*c.y+A.at<double>(0,2);
    double ny=A.at<double>(1,0)*c.x+A.at<double>(1,1)*c.y+A.at<double>(1,2);

    shift.x=nx-c.x;
    shift.y=ny-c.y;

    return true;
}