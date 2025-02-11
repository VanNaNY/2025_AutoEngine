#include "VisionFeatureExtractor.hpp"

template <typename T>
T Compare(T a,T b)
{
    return a > b;
}

cv::Mat VisionFeatureExtractor::preprocessImage(const cv::Mat& rgb_img)
{
    Mat hsv_img;
    cvtColor(rgb_img, hsv_img, COLOR_RGB2HSV);
    vector<Mat> hsv_channels;
    // 拆分HSV图像的通道
    split(hsv_img, hsv_channels);

    // 定义红色HSV三个通道的范围
    Mat red_mask_1, red_mask_2;
    // 色相通道范围1（接近0度的红色）
    Scalar red_hue_lower_1(100);
    Scalar red_hue_upper_1(150);
    // 色相通道范围2（接近180度的红色）
    Scalar red_hue_lower_2(160);
    Scalar red_hue_upper_2(180);
    // 饱和度通道范围（降低饱和度的最小值，以适应不同光照）
    Scalar red_sat_lower(100);
    Scalar red_sat_upper(255);
    // 明度通道范围（降低明度的最小值，以适应暗部）
    Scalar red_val_lower(150);
    Scalar red_val_upper(255);

    // 分别对红色色相的两个范围，同时结合饱和度、明度通道进行范围限制来提取红色区域 
    inRange(hsv_channels.at(0), red_hue_lower_1, red_hue_upper_1, red_mask_1);
    Mat red_sat_mask_1;
    inRange(hsv_channels.at(1), red_sat_lower, red_sat_upper, red_sat_mask_1);
    bitwise_and(red_mask_1, red_sat_mask_1, red_mask_1);
    Mat red_val_mask_1;
    inRange(hsv_channels.at(2), red_val_lower, red_val_upper, red_val_mask_1);
    bitwise_and(red_mask_1, red_val_mask_1, red_mask_1);

    inRange(hsv_channels.at(0), red_hue_lower_2, red_hue_upper_2, red_mask_2);
    Mat red_sat_mask_2;
    inRange(hsv_channels.at(1), red_sat_lower, red_sat_upper, red_sat_mask_2);
    bitwise_and(red_mask_2, red_sat_mask_2, red_mask_2);
    Mat red_val_mask_2;
    inRange(hsv_channels.at(2), red_val_lower, red_val_upper, red_val_mask_2);
    bitwise_and(red_mask_2, red_val_mask_2, red_mask_2);

    // 合并红色色相两个范围对应的掩码
    Mat red_mask = red_mask_1 | red_mask_2;

    // 定义蓝色在色相、饱和度、明度三个通道的范围
    Mat blue_mask;
    // 色相通道范围（蓝色色相范围示例，可根据实际微调）
    Scalar blue_hue_lower(100);
    Scalar blue_hue_upper(114);
    // 饱和度通道范围（可调整）
    Scalar blue_sat_lower(150);
    Scalar blue_sat_upper(255);
    // 明度通道范围（可调整）
    Scalar blue_val_lower(100);
    Scalar blue_val_upper(255);

    // 同时对三个通道进行范围限制来提取蓝色区域
    inRange(hsv_channels.at(0), blue_hue_lower, blue_hue_upper, blue_mask);
    Mat blue_sat_mask;
    inRange(hsv_channels.at(1), blue_sat_lower, blue_sat_upper, blue_sat_mask);
    bitwise_and(blue_mask, blue_sat_mask, blue_mask);
    Mat blue_val_mask;
    inRange(hsv_channels.at(2), blue_val_lower, blue_val_upper, blue_val_mask);
    bitwise_and(blue_mask, blue_val_mask, blue_mask);

    // 合并红色和蓝色的掩码，通过逻辑或操作，得到同时包含红色和蓝色区域的掩码
    Mat combined_mask = red_mask | blue_mask;

    Mat result;
    // 将原图像中对应掩码为白色（即红色和蓝色区域）的部分复制到result中，实现只显示红色和蓝色部分
    rgb_img.copyTo(result, combined_mask);
    imshow("result",result);

    return combined_mask;
    
}


// 初筛函数，根据给定的条件筛选轮廓
vector<vector<Point>> VisionFeatureExtractor::FirstSelectContours(const Mat frame) 
{
    vector<vector<Point>> allContours;
    vector<Vec4i> hierarchy;
    findContours(frame, allContours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<Point>> screenedContours;
    for (const auto& contour : allContours) {
        // 计算轮廓的外接矩形，以获取宽和高
        Rect rect = boundingRect(contour);
        double width = rect.width;
        double height = rect.height;
        // 计算轮廓面积
        double area = contourArea(contour);
        // 进行多边形逼近，获取拟合边数
        vector<Point> approx;
        approxPolyDP(contour, approx, 2, true);
        int edge = (int)approx.size();

        // 判断宽高比和高宽比小于4.5
        if ((height / width < 4.5) && (width / height < 4.5)) {
            // 判断轮廓面积大于400小于12000
            if (area > 1 && area < 5000){
                // 判断多边形拟合边数大于5小于9
                if (edge > 4 && edge < 9) {
                    screenedContours.push_back(contour);
                }
            }
        }
    }
    return screenedContours;
}

//再筛函数
vector<vector<Point>> VisionFeatureExtractor::SecondSelectContours(vector<vector<Point>> firstContours)
{
    if (firstContours.size() < 4)
    {
        return vector<vector<Point>>();
        //Point3f missing_point = MissingPoint(firstContours);
    }
    
    else if(firstContours.size() > 4)
    {
        sort(firstContours.begin(), firstContours.end(), [](const vector<Point>& contour1, const vector<Point>& contour2) {
            double area1 = contourArea(contour1);
            double area2 = contourArea(contour2);
            return area1 > area2;
        });

        auto start = firstContours.begin();
        auto end = std::next(start, 4); 
        vector<vector<Point>> topFourContours;
        copy(start, end, back_inserter(topFourContours));
        return topFourContours;
    }
    

    double minSize = contourArea(firstContours[0]);
    double maxSize = contourArea(firstContours[0]);
    for (const auto& contour : firstContours) 
    {
        double area = contourArea(contour);
        minSize = min(minSize, area);
        maxSize = max(maxSize, area);
    }
    if (maxSize < 10 * minSize) 
    {
        cout<<"Contours >= 4 but the maxArea has much too large than minArea."<<endl;
        return firstContours;
    }
    return vector<vector<Point>>();
    
}

// 辅助函数：在给定窗口内（基于二值掩码）检测小方块，并返回候选小方块的中心点
static vector<Point> detectSmallSquaresInMask(const Mat &mask, const Rect &win) {
    vector<Point> centroids;
    Mat roi = mask(win); // 掩码图已经是二值图，不需要转换
    vector<vector<Point>> cnts;
    findContours(roi, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (auto &cnt : cnts) {
        double area = contourArea(cnt);
        if (area < 50 || area > 200)
            continue;
        RotatedRect rRect = minAreaRect(cnt);
        double rectArea = rRect.size.width * rRect.size.height;
        if (rectArea > 1.2 * area)
            continue;
        float ratio1 = rRect.size.height / rRect.size.width;
        float ratio2 = rRect.size.width / rRect.size.height;
        if (ratio1 >= 2 || ratio2 >= 2)
            continue;
        Moments M = moments(cnt);
        if (M.m00 != 0) {
            int cx = int(M.m10 / M.m00);
            int cy = int(M.m01 / M.m00);
            centroids.push_back(Point(cx + win.x, cy + win.y));
        }
    }
    return centroids;
}

// 根据角点附近小方块候选，确定哪个角点为0号
static int findIndexOfCaseCorner0(const vector<Point> &corners, const Mat &mask, const vector<vector<Point>> &contours) {
    int bestIndex = -1;
    double bestDist = 1e9;
    bool candidateFound = false;
    int windowSize = 200; // 检测周围小正方形的范围
    for (int i = 0; i < corners.size(); i++) {
        Point pt = corners[i];
        int x = max(pt.x - windowSize / 2, 0);
        int y = max(pt.y - windowSize / 2, 0);
        int w = min(windowSize, mask.cols - x);
        int h = min(windowSize, mask.rows - y);
        Rect win(x, y, w, h);
        vector<Point> centers = detectSmallSquaresInMask(mask, win);
        if (!centers.empty()) {
            int sumx = 0, sumy = 0;
            for (auto &c : centers) {
                sumx += c.x;
                sumy += c.y;
            }
            Point avg(sumx / centers.size(), sumy / centers.size());
            double dist = norm(avg - pt);
            if (dist < bestDist) {
                bestDist = dist;
                bestIndex = i;
                candidateFound = true;
            }
        }
    }
    
    if (!candidateFound) {
        // 如果没有检测到任何小方块，则选择对应轮廓面积最小的角点（远离相机的角点）
        cout<<"not found small squares!"<<endl;
        double minArea = 1e9;
        int index = 0;
        for (int i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area < minArea) {
                minArea = area;
                index = i;
            }
        }
        bestIndex = index;
    }
    return bestIndex;
}

// 重排序，将角点向量旋转使得0号角点位于首位
static vector<Point> reorderCaseCorners(const vector<Point> &corners, const Mat &mask, const vector<vector<Point>> &contours) {
    int idx0 = findIndexOfCaseCorner0(corners, mask, contours);
    vector<Point> reordered;
    for (int i = 0; i < corners.size(); i++) {
        reordered.push_back(corners[(i + idx0) % corners.size()]);
    }
    return reordered;
}

vector<Point> VisionFeatureExtractor::FindCaseLocation(const vector<vector<Point>>& contours, const Mat &mask)
{
    Point midpoint = calculateMidpoint(contours);
    vector<Point> cornerVertices(4);
    
    for (size_t i = 0; i < contours.size(); ++i) {
        // 将轮廓点转换为 Point2f 类型
        vector<Point2f> contourPoints;
        for (const auto& point : contours[i]) {
            contourPoints.push_back(point);
        }
        vector<Point2f> triangle;
        minEnclosingTriangle(contourPoints, triangle);

        // 计算三角形重心作为角点
        Point2f centroid;
        centroid.x = (triangle[0].x + triangle[1].x + triangle[2].x) / 3.0f;
        centroid.y = (triangle[0].y + triangle[1].y + triangle[2].y) / 3.0f;
        
        // 在原始轮廓点中找到最接近重心的点
        Point nearestPoint;
        double minDist = DBL_MAX;
        for(const Point& pt : contours[i]) {
            double dist = norm(Point2f(pt) - centroid);
            if(dist < minDist) {
                minDist = dist;
                nearestPoint = pt;
            }
        }
        
        cornerVertices[i] = nearestPoint;
    }
    
    // 利用辅助函数根据小方块检测结果重新排列角点，使得0号角点为"右上小角点"
    vector<Point> orderedCorners = reorderCaseCorners(cornerVertices, mask, contours);
    return orderedCorners;
}
        

Point VisionFeatureExtractor::calculateMidpoint(const vector<vector<Point>>& contours) {
    double sumX = 0;
    double sumY = 0;
    for (size_t i = 0; i < 4; ++i) {
        // 计算每个轮廓的最小外接圆
        Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);
        sumX += center.x;
        sumY += center.y;
    }
    return Point(sumX / 4, sumY / 4);
}


double VisionFeatureExtractor::distance(Point p1, Point p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double VisionFeatureExtractor::calculateAngle(Point p1, Point p2, Point p3) {
    double a = distance(p2, p3);
    double b = distance(p1, p3);
    double c = distance(p1, p2);
    return acos((b * b + c * c - a * a) / (2 * b * c)) * 180 / CV_PI;
}

float VisionFeatureExtractor::getFocalLengthInPixels(float image_width_in_pixels, float HFOV) {
    return image_width_in_pixels * 0.5 / std::tan(HFOV * 0.5 * CV_PI / 180.0);
}

float VisionFeatureExtractor::calculateDepth(float focal_length, float baseline, uint16_t disparity) {
    if (disparity == 0) {
        return -1.0;  // 避免除以零
    }
    return (focal_length * baseline) / static_cast<float>(disparity);
}


std::vector<float> VisionFeatureExtractor::calculate_distances(const std::vector<cv::Point2f>& points, dai::Device& device) {
    std::vector<float> distances;
    auto spatialCalcConfigInQueue = device.getInputQueue("spatialCalcConfig");
    auto spatialDataQueue = device.getOutputQueue("spatialData");
    dai::SpatialLocationCalculatorConfigData config;
    config.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::AVERAGE;
    config.depthThresholds.lowerThreshold = 300;
    config.depthThresholds.upperThreshold = 2000;

    for (const auto& point : points) {

        config.roi = dai::Rect(point.x, point.y, 1, 1);
        //config.point = dai::Point2f(point.x, point.y);
        dai::SpatialLocationCalculatorConfig cfg;
        cfg.addROI(config);

        spatialCalcConfigInQueue->send(cfg); 
        std::shared_ptr<dai::SpatialLocationCalculatorData> spatialData = spatialDataQueue->get<dai::SpatialLocationCalculatorData>();
        if (spatialData) {
            auto depthData = spatialData->getSpatialLocations()[0];
            distances.push_back(depthData.spatialCoordinates.z);
        } else {
            std::cerr << "Failed to get spatial data for point (" << point.x << ", " << point.y << ")" << std::endl;
            distances.push_back(-1.0);
            // 如果获取空间数据失败，输出错误信息并将 -1.0 作为失败标志添加到 distances 向量中。
        }
    }

    return distances;
}

std::vector<float> VisionFeatureExtractor::getDistances(const std::vector<cv::Point>& points, const cv::Mat& depthMap) {
    std::vector<float> distances;
    for (const cv::Point& point : points) {
        if (point.x >= 0 && point.x < depthMap.cols && point.y >= 0 && point.y < depthMap.rows) {
            float depthValue = depthMap.at<ushort>(point.y, point.x);
            distances.push_back(depthValue);
        } else {
            std::cerr << "Invalid point: (" << point.x << ", " << point.y << ")" << std::endl;
            distances.push_back(0);  // 对于无效点，距离设为 0 或其他合适的默认值
        }
    }
    return distances;
}

// 计算向量叉乘
dai::Point3f VisionFeatureExtractor::crossProduct(const dai::Point3f& a, const dai::Point3f& b) 
{
    return dai::Point3f
    (
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

// 计算向量点乘
double VisionFeatureExtractor::dotProduct(const dai::Point3f& a, const dai::Point3f& b) 
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 计算向量的模
double VisionFeatureExtractor::magnitude(const dai::Point3f& a) 
{
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

// 计算三个点确定的平面与坐标轴的夹角
std::vector<double> VisionFeatureExtractor::calculatePlaneAxisAngles(const dai::Point3f& p1, const dai::Point3f& p2, const dai::Point3f& p3) 
{
    // 计算向量 AB 和 AC
    dai::Point3f AB(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    dai::Point3f AC(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);

    // 计算平面的法向量
    dai::Point3f normal = crossProduct(AB, AC);
    double normalMagnitude = magnitude(normal);

    // 定义坐标轴单位向量
    dai::Point3f unitX(1, 0, 0);
    dai::Point3f unitY(0, 1, 0);
    dai::Point3f unitZ(0, 0, 1);

    // 计算法向量与坐标轴单位向量的夹角余弦值
    double cosThetaX = dotProduct(normal, unitX) / normalMagnitude;
    double cosThetaY = dotProduct(normal, unitY) / normalMagnitude;
    double cosThetaZ = dotProduct(normal, unitZ) / normalMagnitude;

    // 计算夹角（弧度制）
    double thetaX = std::acos(cosThetaX);
    double thetaY = std::acos(cosThetaY);
    double thetaZ = std::acos(cosThetaZ);

    // 转换为角度制并计算平面与坐标轴的夹角
    double alpha = 90 - (thetaX * 180 / M_PI);
    double beta = 90 - (thetaY * 180 / M_PI);
    double gamma = 90 - (thetaZ * 180 / M_PI);

    return {alpha, beta, gamma};
}

//计算两个点的方向向量
dai::Point3f VisionFeatureExtractor::calculateDirectionVector(const dai::Point3f& p1, const dai::Point3f& p2) 
{
    return dai::Point3f(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
}

/*
dai::Point3f VisionFeatureExtractor::MissingPoint(const vector<vector<Point>>& contours)
{
    if(contours.size() == 3)
    {
        
    }
    else if(contours.size()==2)
    {
        return vector<vector<Point>>();
    }
    else if(contours.size()==1)
    {
        return vector<vector<Point>>();
    }
}
*/