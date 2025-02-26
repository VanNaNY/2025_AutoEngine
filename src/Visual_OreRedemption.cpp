#include "VisionFeatureExtractor.cpp"
// Includes common necessary includes for development using depthai library
#define DEBUG


int main() 
{
    using namespace TRoMaC;
    VisionFeatureExtractor Vfe;
    dai::CameraControl ctrl;
    dai::RawToFConfig rawToFConfig;
    std::vector<double> angles;
    ReceiveData receive_data;
    VisionData send_data;
    dai::Pipeline pipeline;
    std::shared_ptr<Uart> serial = std::make_shared<Uart>(); // 串口共享指针
    chrono::high_resolution_clock::time_point serial_start, serial_end;
    const float FPS = 30.0;
    const dai::CameraBoardSocket TOF_SOCKET = dai::CameraBoardSocket::CAM_A;
    const dai::CameraBoardSocket RGB_SOCKET = dai::CameraBoardSocket::CAM_C;
    std::vector<dai::SpatialLocationCalculatorData> depth_datas;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto tof = pipeline.create<dai::node::ToF>();
    auto xinTofConfig = pipeline.create<dai::node::XLinkIn>();
    auto cam_tof = pipeline.create<dai::node::ColorCamera>();
    auto spatialLocationCalculator = pipeline.create<dai::node::SpatialLocationCalculator>();
    auto xoutSpatialData = pipeline.create<dai::node::XLinkOut>();
    auto xinSpatialCalcConfig = pipeline.create<dai::node::XLinkIn>();
    auto controlIn = pipeline.create<dai::node::XLinkIn>();

    tof->initialConfig.set(rawToFConfig);
    controlIn->setStreamName("control");
    xoutVideo->setStreamName("video");
    xoutDepth->setStreamName("depth");
    xinTofConfig->setStreamName("tofConfig");
    xoutSpatialData->setStreamName("spatialData");
    xinSpatialCalcConfig->setStreamName("spatialCalcConfig");
    
    /*
    if (serial -> Open(1, 115200))
    {
        std::cout << "Open the serial successfully" << std::endl;
    }
    else
    {
        std::cout << "can't open the serial" << std::endl;
        // 打不开串口重启程序
        const char *program = "../bin/Visual_OreRedemption";
        char *const argv[] = {const_cast<char *>(program), nullptr};
        execv(program, argv);
        perror("execv failed");
        exit(EXIT_FAILURE);
    }
    */
    
    dai::SpatialLocationCalculatorConfigData config;
    spatialLocationCalculator->initialConfig.addROI(config);
    config.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MIN;
    
    // Properties
    camRgb->setBoardSocket(RGB_SOCKET);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_800_P);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->initialControl.setManualExposure(0, 0);
    camRgb->setFps(FPS);
    cam_tof->setFps(FPS);
    cam_tof->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    
    spatialLocationCalculator->inputConfig.setWaitForMessage(false);
    spatialLocationCalculator->inputDepth.setBlocking(false);
    spatialLocationCalculator->inputDepth.setQueueSize(8);

    // Linking
    controlIn->out.link(camRgb->inputControl);
    cam_tof->raw.link(tof->input);
    tof->depth.link(xoutDepth->input);
    camRgb->isp.link(xoutVideo->input);
    spatialLocationCalculator->out.link(xoutSpatialData->input);
    xinSpatialCalcConfig->out.link(spatialLocationCalculator->inputConfig);
    tof->depth.link(spatialLocationCalculator->inputDepth);
    // Connect to device and start pipeline 
    dai::Device device(pipeline);

    auto controlQueue = device.getInputQueue("control");
    auto video = device.getOutputQueue("video", 8, false);
    auto depth = device.getOutputQueue("depth", 8, false);
    auto spatialQueue = device.getOutputQueue("spatialData", 8, false);
    auto spatialCalcConfigQueue = device.getInputQueue("spatialCalcConfig");
    
    ctrl.setManualExposure(160, 100);
    controlQueue->send(ctrl);

    while(true) 
    { 
        serial_start = chrono::high_resolution_clock::now();
        double MiddleDepth = 0.0;
        Point MiddlePoint = Point(0,0);
        dai::Point3f normal = dai::Point3f(0,0,0);
        vector<Point> cornerVertices;
        vector<Point> correctCorner;
        dai::Point3f XdirectionVector;
        dai::Point3f YdirectionVector;
        dai::Point3f CorrectXV;
        dai::Point3f CorrectYV;
        std::vector<dai::Point3f> points3f;
        //预留下位机给我发回来的信息
        /*
        if (serial -> ReadData())
            {
                receive_data.CheckID = serial -> read_data.CheckID;
                receive_data.Exposure = serial -> read_data.Exposure;
                receive_data.LossPackage = serial -> read_data.LossPackage;
                receive_data.Switch = serial -> read_data.Switch;
                receive_data.EndFrame = serial -> read_data.EndFrame;
                
                
                if(isnan(receive_data.CheckID) || isnan(receive_data.Exposure) || isnan(receive_data.LossPackage) || isnan(receive_data.Switch) || isnan(receive_data.EndFrame))
                {
                    //串口接受错误重启程序
                    const char *program = "../bin/Visual_OreRedemption";
                    char *const argv[] = {const_cast<char *>(program), nullptr};
                    cv::waitKey(100);
                    execv(program, argv);
                    perror("execv failed");
                    exit(EXIT_FAILURE);
                }

                printf("CheckID:  %d\n", receive_data.CheckID);
                printf("Exposure:  %d\n", receive_data.Exposure);
                printf("LossPackage:  %d\n", receive_data.LossPackage);
                printf("Switch:  %d\n", receive_data.Switch);
                printf("EndFrame:  %d\n", receive_data.EndFrame);
                printf("\n");

            }
        */
        auto videoIn = video->get<dai::ImgFrame>();
        auto frame = videoIn->getCvFrame();
        resize(frame, frame, Size(600, 400));//640 480:中心(320,240)   
        auto Pvideo = Vfe.preprocessImage(frame);
        auto spatialData = spatialQueue->get<dai::SpatialLocationCalculatorData>();
        auto FirstContours = Vfe.FirstSelectContours(Pvideo);
        auto Contours = Vfe.SecondSelectContours(FirstContours);
        //cout<<"FirstContours.size():"<<FirstContours.size()<<endl;
        //cout<<"Contours.size():"<<Contours.size()<<endl;
        if (Contours.size() == 1)
        {
            vector<dai::Rect> rois;
            auto color = cv::Scalar(0, 255, 0);
            auto fontType = cv::FONT_HERSHEY_TRIPLEX;
            vector<Point2f> contourPoints;
            for (const auto& point : Contours[0]) 
            {
                contourPoints.push_back(point);
            }
            vector<Point2f> triangle;
            minEnclosingTriangle(contourPoints, triangle);
            for(const auto& corner : triangle)
            {
                cout<< "corner: "<< corner.x << " , "<< corner.y << ")"<<endl;
                cornerVertices.push_back(corner);
            }
            for(auto& point : cornerVertices)
            {
                rois.push_back(dai::Rect(dai::Point2f(point.x - 10, point.y - 10), dai::Point2f(point.x + 10, point.y + 10)));
            }
            dai::RawSpatialLocationCalculatorConfig newConfig;
            newConfig.config.clear();
            for(auto& roi : rois)
            {
                dai::SpatialLocationCalculatorConfigData config;
                config.depthThresholds.lowerThreshold = 150;
                config.depthThresholds.upperThreshold = 2000;
                config.roi = roi;
                config.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MIN;
                spatialLocationCalculator->initialConfig.addROI(config);
                newConfig.config.push_back(config);
            }
            const auto& spatialLocations = spatialData->getSpatialLocations();
            auto configMsg = std::make_shared<dai::SpatialLocationCalculatorConfig>();
            configMsg->set(newConfig);
            spatialCalcConfigQueue->send(configMsg);

            for(auto depthData : spatialLocations) 
            {
                auto roi = depthData.config.roi;
                roi = roi.denormalize(Pvideo.cols, Pvideo.rows);
                auto xmin = static_cast<int>(roi.topLeft().x);
                auto ymin = static_cast<int>(roi.topLeft().y);
                auto xmax = static_cast<int>(roi.bottomRight().x);
                auto ymax = static_cast<int>(roi.bottomRight().y);
                //cout<<"xmin:"<<xmin<<",ymin:"<<ymin<<",xmax:"<<xmax<<",ymax:"<<ymax<<endl;

                auto coords = depthData.spatialCoordinates;
                auto distance = coords.z;
                points3f.push_back(dai::Point3f(coords.x, coords.z, coords.y));
            }
            for(const auto& point3f : points3f)
            {
                cout<<"point3f: ("<< point3f.x<< " , "<< point3f.y<<" , "<< point3f.z<< ")"<< endl;
            }
        }
        else if(Contours.size() == 3)
        {
            vector<dai::Rect> rois;
            auto color = cv::Scalar(0, 255, 0);
            auto fontType = cv::FONT_HERSHEY_TRIPLEX;
            Point fourthCorner = Vfe.findFourthCorner(FirstContours, frame);
            cornerVertices = Vfe.FindCaseLocation(Contours, Pvideo);
            cornerVertices.push_back(fourthCorner);
            //cout<<"cornerVertices.size():"<<cornerVertices.size()<<endl;
            for(int i = 0; i < cornerVertices.size(); i++)
            {
                cout<<"cornerVertices["<<i<<"]:("<<cornerVertices[i].x<<","<<cornerVertices[i].y<<")"<<endl;
            }
            for(const auto& point : cornerVertices)
            {
                if(point.x > 0 && point.y > 0)
                {
                    MiddlePoint.x += point.x;
                    MiddlePoint.y += point.y;
                }
            }
            //cout<< "MiddlePoint.x:"<< MiddlePoint.x<< endl;
            MiddlePoint.x /= 4;
            MiddlePoint.y /= 4;
            cornerVertices.pop_back();
            for(auto& point : cornerVertices)
            {
                rois.push_back(dai::Rect(dai::Point2f(point.x - 20, point.y + 40), dai::Point2f(point.x + 10, point.y + 80)));
            }
            dai::RawSpatialLocationCalculatorConfig newConfig;
            newConfig.config.clear();
            for(auto& roi : rois)
            {
                dai::SpatialLocationCalculatorConfigData config;
                config.depthThresholds.lowerThreshold = 150;
                config.depthThresholds.upperThreshold = 2000;
                config.roi = roi;
                config.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MIN;
                spatialLocationCalculator->initialConfig.addROI(config);
                newConfig.config.push_back(config);
            }
            const auto& spatialLocations = spatialData->getSpatialLocations();
            auto configMsg = std::make_shared<dai::SpatialLocationCalculatorConfig>();
            configMsg->set(newConfig);

            // 发送配置消息到 SpatialLocationCalculator
            spatialCalcConfigQueue->send(configMsg);
            
            for(auto depthData : spatialLocations) 
            {
                auto roi = depthData.config.roi;
                roi = roi.denormalize(Pvideo.cols, Pvideo.rows);
                auto xmin = static_cast<int>(roi.topLeft().x);
                auto ymin = static_cast<int>(roi.topLeft().y);
                auto xmax = static_cast<int>(roi.bottomRight().x);
                auto ymax = static_cast<int>(roi.bottomRight().y);
                //cout<<"xmin:"<<xmin<<",ymin:"<<ymin<<",xmax:"<<xmax<<",ymax:"<<ymax<<endl;

                auto coords = depthData.spatialCoordinates;
                auto distance = coords.z;
                points3f.push_back(dai::Point3f(coords.x, coords.z, coords.y));
                //points3f.push_back(dai::Point3f(roi.topLeft().x,roi.topLeft().y, distance));
                cv::rectangle(frame, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color);
                std::stringstream depthDistance;
                depthDistance.precision(2);
                depthDistance << fixed << static_cast<float>(distance / 1.0f) << "mm";
                cv::putText(frame, depthDistance.str(), cv::Point(xmin + 10, ymin + 20), fontType, 0.5, color);
            }
            MiddleDepth = (points3f[0].y + points3f[2].y) / 2;
            if (!points3f.empty()) 
            {
                cout << "中心顶点坐标: (" << MiddlePoint.x << ", " << MiddlePoint.y << ", " << MiddleDepth << " mm)" << endl;
            }
            XdirectionVector = Vfe.calculateDirectionVector(points3f[0], points3f[1]);
            YdirectionVector = Vfe.calculateDirectionVector(points3f[0], points3f[3]);
            circle(frame, Point(fourthCorner.x, fourthCorner.y), 5, Scalar(3, 245, 225), -1);
            cout<<"XdirectionVector:("<<XdirectionVector.x <<","<<XdirectionVector.y<<","<<XdirectionVector.z<<")"<<endl;
            cout<<"YdirectionVector:("<<YdirectionVector.x <<","<<YdirectionVector.y<<","<<YdirectionVector.z<<")"<<endl;
        }   
        else if(Contours.size()==4)
        {
            vector<dai::Rect> rois;
            vector<Point> OuterCorner;
            vector<Point> InnerCorner;
            auto color = cv::Scalar(0, 255, 0);
            auto fontType = cv::FONT_HERSHEY_TRIPLEX;
            MiddlePoint = Vfe.calculateMidpoint(Contours);
            //InnerCorner = Vfe.FindCaseLocation(Contours, Pvideo);
            //OuterCorner = Vfe.FindOuterCaseLocation(Contours, Pvideo);
            /*
            for(int i = 0; i < 4; i++)
            {
                cornerVertices.push_back(Point(((InnerCorner[i].x + OuterCorner[i].x) / 2), 
                                                (InnerCorner[i].y + OuterCorner[i].y) / 2));
            }
            */
            //cout<<"cornerVertices.size()"<<cornerVertices.size()<<endl;
            cornerVertices = Vfe.FindCaseLocation(Contours, Pvideo);
            correctCorner = Vfe.sortCornersCounterClockwise(cornerVertices, MiddlePoint);
            for(auto& point : correctCorner)
            {
                rois.push_back(dai::Rect(dai::Point2f(point.x - 20, point.y + 30), dai::Point2f(point.x + 10, point.y + 80)));
            }
            dai::RawSpatialLocationCalculatorConfig newConfig;
            newConfig.config.clear();
            for(auto& roi : rois)
            {
                dai::SpatialLocationCalculatorConfigData config;
                config.depthThresholds.lowerThreshold = 150;
                config.depthThresholds.upperThreshold = 2000;
                config.roi = roi;
                config.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MIN;
                spatialLocationCalculator->initialConfig.addROI(config);
                newConfig.config.push_back(config);
            }
            const auto& spatialLocations = spatialData->getSpatialLocations();
            auto configMsg = std::make_shared<dai::SpatialLocationCalculatorConfig>();
            configMsg->set(newConfig);


            // 发送配置消息到 SpatialLocationCalculator
            spatialCalcConfigQueue->send(configMsg);
            for(auto depthData : spatialLocations) 
            {
                auto roi = depthData.config.roi;
                roi = roi.denormalize(Pvideo.cols, Pvideo.rows);
                auto xmin = static_cast<int>(roi.topLeft().x);
                auto ymin = static_cast<int>(roi.topLeft().y);
                auto xmax = static_cast<int>(roi.bottomRight().x);
                auto ymax = static_cast<int>(roi.bottomRight().y);

                auto coords = depthData.spatialCoordinates;
                //auto distance = std::sqrt(coords.x * coords.x + coords.y * coords.y + coords.z * coords.z);
                auto distance = coords.z;
                //points3f.push_back(dai::Point3f(roi.topLeft().x,roi.topLeft().y, distance)); 
                points3f.push_back(dai::Point3f(coords.x, coords.z, coords.y));
                MiddleDepth += distance;
                cv::rectangle(frame, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color);
                std::stringstream depthDistance;
                depthDistance.precision(2);
                depthDistance << fixed << static_cast<float>(distance / 1.0f) << "mm";
                cv::putText(frame, depthDistance.str(), cv::Point(xmin + 20, ymin + 10), fontType, 0.5, color);
            }
            MiddleDepth /= 4;
            if (!points3f.empty()) 
            {
                cout << "中心顶点坐标: (" << MiddlePoint.x - 300<< ", " << MiddleDepth<< ", " << -(MiddlePoint.y - 200)<<")" << endl;
            }
            normal = Vfe.calculatePlaneNormal(points3f[0],points3f[3],points3f[2]);
            //std::cout << "平面与 x 轴的夹角: " << angles[0] << " 度" << std::endl;
            //std::cout << "平面与 y 轴的夹角: " << angles[1] << " 度" << std::endl;
            //std::cout << "平面与 z 轴的夹角: " << angles[2] << " 度" << std::endl;

            XdirectionVector = Vfe.calculateDirectionVector(points3f[2], points3f[3]);
            YdirectionVector = Vfe.crossProduct(XdirectionVector, normal);
            
            CorrectXV = Vfe.unitization(XdirectionVector.x, XdirectionVector.y, XdirectionVector.z);
            CorrectYV = Vfe.unitization(YdirectionVector.x, YdirectionVector.y, YdirectionVector.z);
            normal = Vfe.unitization(normal.x, normal.y, normal.z);

            cout<<"XdirectionVector:("<<CorrectXV.x <<","<<CorrectXV.y<<","<<CorrectXV.z<<")"<<endl;
            cout<<"YdirectionVector:("<<CorrectYV.x <<","<<CorrectYV.y<<","<<CorrectYV.z<<")"<<endl;
            cout<< "normal:("<< normal.x<<","<< normal.y<< ","<< normal.z<< ")"<< endl;
        }
        if(!points3f.empty())
        {
            send_data.x = MiddlePoint.x - 300;//MiddlePoint.x - 320
            send_data.y = MiddleDepth;//MiddlePoint.z;
            send_data.z = -(MiddlePoint.y - 200);//-(MiddlePoint.y - 240)

            send_data.X_x = CorrectXV.x;
            send_data.X_y = CorrectXV.y;
            send_data.X_z = CorrectXV.z;

            send_data.Y_x = CorrectYV.x;
            send_data.Y_y = CorrectYV.y;
            send_data.Y_z = CorrectYV.z;

            send_data.Z_x = normal.x;//normal.x / 10000;//angles[0];
            send_data.Z_y = normal.y;//normal.y / 10000;//angles[1];
            send_data.Z_z = normal.z;//normal.z / 10000;//angles[2];
            serial -> send(send_data);
        }
#ifdef DEBUG
        for (const auto& contour : Contours) 
        {
            drawContours(frame, vector<vector<Point>>{contour}, -1, Scalar(255, 255, 255), 3);
        }
        if(Contours.size())
        {
            circle(frame,Point(MiddlePoint.x,MiddlePoint.y),5,Scalar(255, 255, 255), -1);
        }
        if(correctCorner.size())
        {
            for (int i = 0; i < correctCorner.size(); i++)
            {
                if (i == 0)
                {
                    circle(frame,Point(correctCorner[i].x, correctCorner[i].y),5,Scalar(0,255,0), -1);
                }
                else if(i == 1)
                {
                    circle(frame,Point(correctCorner[i].x, correctCorner[i].y),5,Scalar(0,255,255), -1);
                }
                else if(i == 2)
                {
                    circle(frame,Point(correctCorner[i].x, correctCorner[i].y),5,Scalar(0, 140, 255), -1);
                }
                else if(i == 3)
                {
                    circle(frame,Point(correctCorner[i].x, correctCorner[i].y),5,Scalar(0, 0, 255), -1);
                }
            }
        }
        cv::imshow("frame", frame);
        //cv::imshow("Pvideo",Pvideo);
        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') 
        {
            return 0;
        }
#endif
    }
    return 0;
}
