#ifndef VISIONFEATUREEXTRACTOR_H
#define VISIONFEATUREEXTRACTOR_H

#include <iostream>
#include <vector>
#include "Serial.h"
#include "depthai/depthai.hpp"

using namespace cv;
using namespace std;

class VisionFeatureExtractor
{
public:
    cv::Mat preprocessImage(const cv::Mat& rgb_img);
    vector<vector<Point>> FirstSelectContours(const Mat frame);
    vector<Point> FindCaseLocation(const vector<vector<Point>>& Contours, const Mat &mask, const Point MiddlePoint);
    vector<Point> FindCaseLocation(const vector<vector<Point>>& contours, const Mat &mask);
    vector<vector<Point>> SecondSelectContours(const vector<vector<Point>> FirstContours);
    template <typename T>
    void Compare(T a,T b);
    Point calculateMidpoint(const vector<vector<Point>>& contours);
    double distance(Point p1, Point p2);
    double calculateAngle(Point p1, Point p2, Point p3);
    float calculateDepth(float focal_length, float baseline, uint16_t disparity);
    float getFocalLengthInPixels(float image_width_in_pixels, float HFOV);
    std::vector<float> calculate_distances(const std::vector<cv::Point2f>& points, dai::Device& device);
    std::vector<float> getDistances(const std::vector<cv::Point>& points, const cv::Mat& depthMap);
    dai::Point3f crossProduct(const dai::Point3f& a, const dai::Point3f& b);
    double dotProduct(const dai::Point3f& a, const dai::Point3f& b);
    double magnitude(const dai::Point3f& a);
    std::vector<double> calculatePlaneAxisAngles(const dai::Point3f& p1, const dai::Point3f& p2, const dai::Point3f& p3);
    void SerialControl();
    dai::Point3f calculateDirectionVector(const dai::Point3f& p1, const dai::Point3f& p2);
    dai::Point3f MissingPoint(const vector<vector<Point>>& contours);
    vector<Point> FindOuterCaseLocation(const vector<vector<Point>>& contours, const Mat &mask);
    Point findFourthCorner(const vector<vector<Point>>& triangles, Mat mask);
    Point calculateIntersection(const vector<Point>& corners);
    dai::Point3f calculatePlaneNormal(const dai::Point3f& p1, const dai::Point3f& p2, const dai::Point3f& p3);
    bool lineIntersect(double A1, double B1, double C1, double A2, double B2, double C2, Point2f& intersection);
    vector<Point2f> findIntersection(const std::vector<Point2f>& triangle1, const std::vector<Point2f>& triangle2, Mat frame);
    void lineGeneralForm(Point p1, Point p2, double& A, double& B, double& C);
    void drawLineOnImage(Mat& img, double A, double B, double C);
    void calculateLinePoints(double A, double B, double C, int imgWidth, int imgHeight, Point& pt1, Point& pt2);
    dai::Point3f unitization(double x, double y, double z);
    std::vector<cv::Point> sortCornersCounterClockwise(const std::vector<cv::Point>& corners, const cv::Point2f& center);
};

#endif