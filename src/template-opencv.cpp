/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications 
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctime> 
#include <string>
#include <sstream>

// Create one vector of points for each color
std::vector<cv::Point> blueCones;
std::vector<cv::Point> yellowCones;
// Initiate variables for colors 
char blue = 'B';
char yellow = 'Y';

// High and low global variable values for blue and yellow colors
cv::Scalar blueLow = cv::Scalar(120, 50, 30);
cv::Scalar blueHigh = cv::Scalar(125, 255, 225);
cv::Scalar yellowLow = cv::Scalar(15, 20, 30);
cv::Scalar yellowHigh = cv::Scalar(42, 255, 255);

void getCones(cv::Mat hsvImg, cv::Mat img, cv::Scalar low, cv::Scalar high, char color)
{
    // Create a mask from the hvs image that only contains the colors in the specified range
    cv::Mat mask;
    cv::inRange(hsvImg, low, high, mask);

    // Initiate a variable to store the outlines of the cones
    std::vector<std::vector<cv::Point>> contours;
    // Find the outlines of cones from the mask and assign them to the contours variable
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // Go through all the contours
    for (size_t i = 0; i < contours.size(); ++i)
    {
        // Get a rectangle from the current outline
        cv::Rect boundRect = cv::boundingRect(contours[i]);
        // Check that the rectangle isn't too big or too small to eliminate noise
        if ((boundRect.area() > 200 && (boundRect.width < 60 || boundRect.height < 60)) && boundRect.area() < 1500)
        {
            // Check if the color is blue
            if (color == blue)
            {
                // Add the cone position and rectangle to the blue vector
                blueCones.emplace_back((boundRect.x + boundRect.width / 2), (boundRect.y + boundRect.height / 2));
            }
            // Check if the color is yellow
            if (color == yellow)
            {
                // Add the cone position and rectangle to the yellow vector
                yellowCones.emplace_back((boundRect.x + boundRect.width / 2), (boundRect.y + boundRect.height / 2));
            }
            // Draw the rectangle on the source image
            // boundRect.tl() is the top left of the rectangle; boundRect.br() is the bottom right corner
            cv::rectangle(img, boundRect.tl(), boundRect.br(), cvScalar(0, 255, 0), 3);
        }
    }
}

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env){
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                // OpenCV data structure to hold an image.
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }

                // Get the time for each image
                std::pair<bool, cluon::data::TimeStamp> imageTime = sharedMemory->getTimeStamp();
                // Convert the time to microseconds 
                std::string timestamp = std::to_string(cluon::time::toMicroseconds(imageTime.second));

                // Create an array of points
                cv::Point corners[1][6];
                // The inside of the shape contains the important data
                corners[0][0] = cv::Point( 0, 250);
                corners[0][1] = cv::Point( 640, 250 );
                corners[0][2] = cv::Point( 640, 420 );
                corners[0][3] = cv::Point( 430, 370 );
                corners[0][4] = cv::Point( 220, 370 );
                corners[0][5] = cv::Point( 0, 420 );
                const cv::Point* corner_list[1] = { corners[0] };
                int num_points = 6;

                // Create a new empty image
                cv::Mat mask(480,640,CV_8UC4, cv::Scalar(0,0,0));
                // Add the shape to the image 
                cv::fillPoly( mask, corner_list, &num_points, 1, cv::Scalar( 255, 255, 255 ), 8);

                // Combune the two images whilest only keeping the data inside the shape
                cv::bitwise_and(img, mask, img);
                // Crop some of the dead space
                img = img(cv::Rect(0,250,640,170));

                // Unlock the shared memory
                sharedMemory->unlock();

                // TODO: Do something with the frame.
                // Blue the image to reduce noise 
                cv::GaussianBlur(img, img, cv::Size(7, 7), 1.0);
                // Create a new mat image
                cv::Mat hsvImg;
                // Copy the original image to the new one
                img.copyTo(hsvImg);
                // Convert the new image to the hsv color space
                cv::cvtColor(hsvImg, hsvImg, cv::COLOR_BGR2HSV);

                // Empty the blue vector every frame
                blueCones.clear();
                // Add a initial point (to be deleted later)
                blueCones.emplace_back(0, 170);
                // Call the getCones function for blue
                getCones(hsvImg, img, blueLow, blueHigh, blue);
                // Empry the yellow vector every frame 
                yellowCones.clear();
                // Add a initial point (to be deleted later)
                yellowCones.emplace_back(640, 170);
                // Call the getCones function for yellow
                getCones(hsvImg, img, yellowLow, yellowHigh, yellow);

                // Draw the lanes from the blue cones 
                cv::polylines(img, blueCones, 0, cvScalar(0,0,255), 3, 8, 0);
                // Draw the lanes from the yellow cones 
                cv::polylines(img, yellowCones, 0, cvScalar(0,0,255), 3, 8, 0);

                // Get and format the time.
                std::time_t now = std::time(NULL);
                std::tm * ptm = std::localtime(&now);
                char buffer[22];
                // Format: UTC
                std::strftime(buffer, 22, "%FT%TZ", ptm);

                // Overlay my name on the images
                cv::putText(img, "Name", cv::Point(50, 50), CV_FONT_HERSHEY_TRIPLEX,  0.7, cvScalar(0,0,255), 1, CV_AA);
                // Overlay the current time on the images
                cv::putText(img, buffer, cv::Point(50, 70), CV_FONT_HERSHEY_TRIPLEX,  0.7, cvScalar(0,0,255), 1, CV_AA);
                // Overlay the time from the .rec file on the images
                cv::putText(img, timestamp, cv::Point(50, 90), CV_FONT_HERSHEY_TRIPLEX,  0.7, cvScalar(0,0,255), 1, CV_AA);

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }

                // Display image on your screen.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}

