/**
 * Copyright (C) 2016 Social Robotics Lab, Yale University
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@yale.edu
 * website: www.scazlab.yale.edu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 3 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
**/

#include <string>

#include <pthread.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "BoardCalibrator.h"

#define MIN_ANGLE     5
#define MIN_DIST     15
#define OUT_IMG_H   400
#define OUT_IMG_W   571

using namespace std;

int main(int argc, char** argv)
{
    std::string name="board_calibrator";

    ros::init(argc, argv, name.c_str());
    std::string sub = "/usb_cam/image_raw";
    std::string pub = "/snap_circuits/image_undistorted";
    bool show=false;

    // Dirty way to process command line arguments. It seems that
    // there is not a straightforward standard ROS way, unfortunately.
    if (argc>1)
    {
        ros::NodeHandle nH;

        if (std::string(argv[1])=="--show")
        {
            show=std::string(argv[2])=="true"?true:false;
            nH.setParam(("/"+name+"/show").c_str(), show);
        }

        if (argc>3)
        {
            sub=std::string(argv[3]);
            nH.setParam( ("/"+name+"/sub").c_str(), sub.c_str());
        
            if (argc>4)
            {
                pub=std::string(argv[4]);
                nH.setParam( ("/"+name+"/pub").c_str(), pub.c_str());
            }
        }
    }

    BoardCalibrator board_calibrator(name);
    ros::spin();

    return 0;
}

