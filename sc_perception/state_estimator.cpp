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

#include "StateEstimator.h"

int main(int argc, char** argv)
{
    std::string name="state_estimator";

    ros::init(argc, argv, name.c_str());

    bool show=false;

    // Dirty way to process command line arguments. It seems that
    // there is not a straightforward standard ROS way, unfortunately.
    if (argc>1)
    {
        ros::NodeHandle nH;

        if (std::string(argv[1])=="--show")
        {
            show=std::string(argv[2])=="false"?false:true;
            nH.setParam(("/"+name+"/show").c_str(), show);
        }
    }

    StateEstimator state_estimator(name);
    ros::spin();

    return 0;
}

