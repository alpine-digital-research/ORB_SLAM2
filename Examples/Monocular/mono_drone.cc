/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <unistd.h>
using namespace std;

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_video" << endl;
        return 1;
    }

    cv::VideoCapture capture;
    capture.open(argv[3]);
    if( !capture.isOpened() )
        return fprintf( stderr, "Could not initialize video (%s) capture\n",argv[3] ), -2;

    double timeBetweenFrames = 1/capture.get(CV_CAP_PROP_FPS);
    int frameCount = int(capture.get(CV_CAP_PROP_FRAME_COUNT));

    printf("Time between frames: %0.4d\n", timeBetweenFrames);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2], ORB_SLAM2::System::MONOCULAR,true);

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<frameCount; ni++)
    {
        // Read image from file
        capture >> im;

        if( im.empty() )
        {
            cerr << endl << "Done reading" << endl;
            break;
        }

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,timeBetweenFrames);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

