#include "BoardCalibrator.h"

using namespace std;
 
bool isEqual(const cv::Vec2f& _l1, const cv::Vec2f& _l2)
{
    cv::Vec2f l1(_l1), l2(_l2);

    float rho1   = l1[0];
    float theta1 = l1[1]*180/CV_PI;
    float rho2   = l2[0];
    float theta2 = l2[1]*180/CV_PI;

    float angle=abs(theta2-theta1);
    float dist=abs(rho1-rho2);

    ROS_DEBUG("[BoardCalibrator] Angle is %g\tDist is %g", angle, dist);

    if (angle > MIN_ANGLE || dist > MIN_DIST)
    {
        return false;
    }

    return true;
}

void BoardCalibrator::callback(const sensor_msgs::ImageConstPtr& msgIn)
{
    // Convert the ROS image to OpenCV image format
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msgIn, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert the image to black and white
    cv::Mat img_bw;
    cv::Mat img_res = cv_ptr->image.clone();
    cv::cvtColor(cv_ptr->image,img_bw,CV_BGR2GRAY);

    // Threshold the image to get only the black parts
    cv::threshold(img_bw,img_bw,150,200, cv::THRESH_BINARY_INV+cv::THRESH_OTSU);
    // publishImage(img_bw,sensor_msgs::image_encodings::MONO8);

    // Find the biggest blob in the image (hopefully, the board), and fill it to remove the hexagons
    cv::Mat img_board(findBiggestBlob(img_bw));

    if (doShow)
    {
        cv::imshow("camera_thresholded",img_board);
        cv::waitKey(5);
    }

    // Get the edge map for finding line segments with the Canny method.
    cv::Canny(img_board, img_bw, 30, 30*3, 5);
   
    if (doShow) cv::imshow("camera_edges",img_bw);

    // Detect lines with the Hough transform method.
    vector<cv::Vec2f> lines;
    HoughLines(img_bw, lines, 1, CV_PI/180, 100, 0, 0 );

    if (clusterLines(lines))
    {
        // Find the intersection between the lines
        std::vector<cv::Point2f> corners;
        for (int i = 0; i < lines.size(); i++)
        {
            for (int j = i+1; j < lines.size(); j++)
            {
                cv::Point2f pt = findIntersection(lines[i], lines[j]);
                if (pt.x > 0 && pt.y > 0 && pt.x < img_res.cols && pt.y < img_res.rows)
                    corners.push_back(pt);
            }
        }

        if (doShow) drawCameraLinesCorners(img_bw,lines,corners);

        if (corners.size()!=4)
        {
            printf("[BoardCalibrator] I got %zu Corners: ",corners.size()); 
            for (int i = 0; i < corners.size(); ++i)
            {
                printf("[%g %g]\t", corners[i].x, corners[i].y);
            }
            printf("\n");
        }
        else
        {
            cv::Mat quad = cv::Mat::zeros(OUT_IMG_H, OUT_IMG_W, CV_8UC3);  // Destination image
            
            // Determine top-left, bottom-left, top-right, and bottom-right corner
            sortCorners(corners);

            // Apply the perspective transformation

            // Corners of the destination image
            std::vector<cv::Point2f> quad_pts;
            quad_pts.push_back(cv::Point2f(0, 0));
            quad_pts.push_back(cv::Point2f(quad.cols, 0));
            quad_pts.push_back(cv::Point2f(quad.cols, quad.rows));
            quad_pts.push_back(cv::Point2f(0, quad.rows));

            // Get transformation matrix
            cv::Mat transmtx = cv::getPerspectiveTransform(corners, quad_pts);

            // Apply perspective transformation
            cv::warpPerspective(cv_ptr->image, quad, transmtx, quad.size());

            // if (doShow) cv::imshow("image_undistorted",quad);

            publishImage(quad,sensor_msgs::image_encodings::BGR8);
            ROS_DEBUG("Calibrated Board has been published");
        }
    }
};

bool BoardCalibrator::drawCameraLinesCorners(cv::Mat img_bw, 
                      std::vector<cv::Vec2f> lines, std::vector<cv::Point2f> corners)
{
        cv::cvtColor(img_bw,img_bw,CV_GRAY2RGB);

        // Draw the lines
        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho   = lines[i][0];
            float theta = lines[i][1];
            cv::Point pt1;
            cv::Point pt2;
            double a = cos(theta);
            double b = sin(theta);
            double x0 = a*rho;
            double y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            cv::line( img_bw, pt1, pt2, cv::Scalar(0,255,0), 2, CV_AA);
        }

        // Draw corners
        for (int i = 0; i < corners.size(); i++)
        {
            cv::circle(img_bw, corners[i], 3, CV_RGB(255,255,255), 2);
        }
        
        cv::imshow("camera_lines_and_corners",img_bw);

    return true;
};

bool BoardCalibrator::clusterLines(std::vector<cv::Vec2f> &lines)
{
    // Cluster lines that are relatively close and parallel one another
    std::vector<int> labels;
    int numberOfLines = cv::partition(lines, labels, isEqual);

    int numberOfClusters=0;
    for (int i = 0; i < labels.size(); ++i)
    {
        numberOfClusters=max(numberOfClusters, labels[i]);
    }
    numberOfClusters++;

    ROS_INFO("[BoardCalibrator] Number of lines: %zu\tNumber of clusters: %i ", lines.size(), numberOfClusters);

    if (numberOfClusters == 4)
    {
        // After clustering, we should filter out the unnecessary lines
        // Let's compute an average of all the lines that belong to a cluster        
        std::vector<cv::Vec2f> lines_filt;
        
        for (int i = 0; i < numberOfClusters; ++i)
        {
            cv::Vec2f line(0,0);
            int cnt=0;

            for (int j = 0; j < lines.size(); ++j)
            {
                if (labels[j]==i)
                {
                    line[0]=line[0]+lines[j][0];
                    line[1]=line[1]+lines[j][1];
                    cnt++;
                }
            }

            line[0]=line[0]/cnt;
            line[1]=line[1]/cnt;

            lines_filt.push_back(line);
        }

        // printf("[BoardCalibrator] I got %zu Lines Filt ",lines_filt.size()); 
        // for (int i = 0; i < lines_filt.size(); ++i)
        // {
        //     printf("[%g %g]\t", lines_filt[i][0], lines_filt[i][1]);
        // }
        // printf("\n");

        lines = lines_filt;

        return true;
    }

    return false;
};

bool BoardCalibrator::publishImage(cv::Mat &mat, const std::string encoding)
{
    cv_bridge::CvImage msgOut;
    msgOut.encoding = encoding;
    msgOut.image    = mat;

    imagePublisher.publish(msgOut.toImageMsg());
    return true;
};

cv::Mat BoardCalibrator::findBiggestBlob(cv::Mat & mat)
{
    pthread_mutex_lock(&this->mutex);
    
    int largest_area=0;
    int largest_contour_index=0;

    cv::Mat res = cv::Mat::zeros(mat.rows, mat.cols, CV_8UC3);

    vector< vector<cv::Point> > contours; // Vector for storing contour
    vector<cv::Vec4i> hierarchy;

    cv::findContours( mat, contours, hierarchy, 0, 2 ); // Find the extreme outer contours in the image

    for( int i = 0; i< contours.size(); i++ ) {     // iterate through each contour. 
        double a=contourArea( contours[i],false);   //  Find the area of contour
        if(a>largest_area){
            largest_area=a;
            largest_contour_index=i;                //Store the index of largest contour
        }
    }

    // Draw the largest contour using previously stored index.
    cv::drawContours( res, contours, largest_contour_index, cv::Scalar(255,255,255), CV_FILLED, 8, hierarchy ); 
    
    pthread_mutex_unlock(&this->mutex);
    return res;
};

cv::Point2f BoardCalibrator::findIntersection(cv::Vec2f _l1, cv::Vec2f _l2)
{
    vector<cv::Point2f> p1 = lineToPointPair(_l1);
    vector<cv::Point2f> p2 = lineToPointPair(_l2);

    float denom = (p1[0].x - p1[1].x)*(p2[0].y - p2[1].y) - (p1[0].y - p1[1].y)*(p2[0].x - p2[1].x);

    return cv::Point2f(((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].x - p2[1].x) -
                        (p1[0].x - p1[1].x)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom,
                       ((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].y - p2[1].y) -
                        (p1[0].y - p1[1].y)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom);

};

vector<cv::Point2f> BoardCalibrator::lineToPointPair(cv::Vec2f line)
{
    vector<cv::Point2f> points;

    float r = line[0], t = line[1];
    double cos_t = cos(t), sin_t = sin(t);
    double x0 = r*cos_t, y0 = r*sin_t;
    double alpha = 1000;

    points.push_back(cv::Point2f(x0 + alpha*(-sin_t), y0 + alpha*cos_t));
    points.push_back(cv::Point2f(x0 - alpha*(-sin_t), y0 - alpha*cos_t));

    return points;
};

bool BoardCalibrator::sortCorners(std::vector<cv::Point2f>& corners)
{
    if (corners.size() != 4)
    {
        ROS_ERROR("[BoardCalibrator::sortCorners] The number of corners should be 4.");
        return false;
    }

    // 1. Compute the mass center
    cv::Point2f center(0,0);
    for (int i = 0; i < corners.size(); i++)
        center += corners[i];

    center *= (1. / corners.size());
    
    // 2. Points that have lower y-axis than the mass center are the top points.
    std::vector<cv::Point2f> top, bot;

    for (int i = 0; i < corners.size(); i++)
    {
        if (corners[i].y < center.y)
            top.push_back(corners[i]);
        else
            bot.push_back(corners[i]);
    }

    // 3. Given two top points, the one with lower x-axis is the top-left.
    cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
    cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];

    // 4. Given two bottom points, apply the same rule as #3
    cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
    cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

    corners.clear();
    corners.push_back(tl);
    corners.push_back(tr);
    corners.push_back(br);
    corners.push_back(bl);

    return true;
};

BoardCalibrator::BoardCalibrator(string _name) : name(_name), imageTransport(nodeHandle)
{
    pthread_mutex_init(&this->mutex, NULL);

    nodeHandle.param(("/"+name+"/show").c_str(), doShow, true);
    nodeHandle.param<std::string>(("/"+name+"/sub").c_str(), sub, "/usb_cam/image_raw");
    nodeHandle.param<std::string>(("/"+name+"/pub").c_str(), pub, "/snap_circuits/image_undistorted");

    imageSubscriber = imageTransport.subscribe(sub.c_str(),1,&BoardCalibrator::callback, this);
    imagePublisher  = imageTransport.advertise(pub,1);

    ROS_INFO("[BoardCalibrator] Show param set to %i", doShow);
    ROS_INFO("[BoardCalibrator] Subscribing    to %s", sub.c_str());
    ROS_INFO("[BoardCalibrator] Publishing     to %s", pub.c_str());

    if (doShow)
    {
        ROS_INFO("[BoardCalibrator] Creating windows..");
        cv::namedWindow("camera_thresholded");
        cv::namedWindow("camera_edges");
        cv::namedWindow("camera_lines_and_corners");
        cv::startWindowThread();

        cv::moveWindow("camera_thresholded",3000,50);
        cv::moveWindow("camera_edges",3000,600);
        cv::moveWindow("camera_lines_and_corners",3600,200);

        cv::resizeWindow("camera_thresholded",857,600);
        cv::resizeWindow("camera_edges",857,600);
        cv::resizeWindow("camera_lines_and_corners",857,600);
    }
};

BoardCalibrator::~BoardCalibrator()
{
    if (doShow)
    {
        ROS_INFO("[BoardCalibrator] Destroying windows..");
        cv::destroyWindow("camera_lines_and_corners");
        cv::destroyWindow("camera_thresholded");
        cv::destroyWindow("camera_edges");
    }
};
