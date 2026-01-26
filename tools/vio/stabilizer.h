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

    // ORB
    Ptr<ORB> orb = ORB::create(2000);

    vector<KeyPoint> k1, k2;
    Mat d1, d2;

    orb->detectAndCompute(g1, noArray(), k1, d1);
    orb->detectAndCompute(g2, noArray(), k2, d2);

    if (k1.size() < 20 || k2.size() < 20)
        return false;

    BFMatcher matcher(NORM_HAMMING, true);
    vector<DMatch> matches;
    matcher.match(d1, d2, matches);

    if (matches.size() < 10)
        return false;

    vector<Point2f> p1, p2;
    for (auto& m : matches)
    {
        p1.push_back(k1[m.queryIdx].pt);
        p2.push_back(k2[m.trainIdx].pt);
    }

    // Affine (rotation + scale + translation)
    Mat inliers;
    Mat A = estimateAffinePartial2D(p1, p2, inliers, RANSAC);

    if (A.empty())
        return false;

    affine_out = A.clone();

    // ROI中心点
    Point2f c(roiW * 0.5f, roiH * 0.5f);

    double nx =
        A.at<double>(0,0)*c.x +
        A.at<double>(0,1)*c.y +
        A.at<double>(0,2);

    double ny =
        A.at<double>(1,0)*c.x +
        A.at<double>(1,1)*c.y +
        A.at<double>(1,2);

    shift.x = nx - c.x;
    shift.y = ny - c.y;

    return true;
}