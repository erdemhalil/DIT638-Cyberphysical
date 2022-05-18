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

// Create two points to store the coordinates of the cones
cv::Point blueCones;
cv::Point yellowCones;
// Create variables to store the coordinates of the previous cones
cv::Point lastBlueCone;
cv::Point lastYellowCone;
// Constants for color and direction
const char blue = 'B';
const char yellow = 'Y';
const char right = 'R';
const char left = 'L';
// Bools for cone detection
bool isBlue = false;
bool isYellow = false;
bool bLeft = false;
bool yLeft = false;
bool fBlue = false;
bool fYellow = false;
bool isMirrored = true;
// Variable for storing the direction of the car
double steeringWheelAngle = 0.0;
// Constant storing the number that is used to calculate the steering wheel angle
const double magicNumber = 0.145540;
// Range for the angle calculation
const double threshold = 0.08;
const double threshold2 = 0.9;
// Variable to keep count of the corect frames
double variable = 0;

// Function to calculate the number of frames that meet the performance requirement
void calculatePerformance(double groundSteering)
{
    // If original steering angle is 0, check if we are in range +/- 0.05
    if (groundSteering == 0)
    {
        if (steeringWheelAngle < 0.05 && steeringWheelAngle > -0.05)
        {
            variable++;
        }
    }
    // If original steering angle is greater than 0, check if we are within 50%
    else if (groundSteering > 0)
    {
        if (groundSteering * 0.5 < steeringWheelAngle && steeringWheelAngle < groundSteering * 1.5)
        {
            variable++;
        }
    }
    // If original steering angle is less than 0, check if we are within 50%
    else
    {
        if (groundSteering * 0.5 > steeringWheelAngle && steeringWheelAngle > groundSteering * 1.5)
        {
            variable++;
        }
    }
    // Print the result
    std::cout << variable << std::endl;
}

// High and low global variable values for blue and yellow colors
cv::Scalar blueLow = cv::Scalar(100, 50, 30);
cv::Scalar blueHigh = cv::Scalar(120, 255, 255);
cv::Scalar yellowLow = cv::Scalar(17, 60, 70);
cv::Scalar yellowHigh = cv::Scalar(40, 200, 200);

// Create a point for the center of the car
cv::Point car = cv::Point(320, 0);

// Function to draw the cones on the image
void getCones(cv::Mat hsvImg, cv::Mat img, cv::Scalar low, cv::Scalar high, char color);

// Create a funciton calculate the car direction
void calculate(char direction, double difference);

// Create a function to calculate the difference between cones and the car
void getDistance();

int32_t main(int32_t argc, char **argv)
{
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("name")) ||
        (0 == commandlineArguments.count("width")) ||
        (0 == commandlineArguments.count("height")))
    {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else
    {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid())
        {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env)
            {
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                std::cout << "groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning())
            {
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
                // Crop some of the dead space
                img = img(cv::Rect(0, 240, 640, 100));
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
                // Reset the cone variables every frame
                isBlue = false;
                isYellow = false;
                isMirrored = true;
                // Call the getCones function for blue
                getCones(hsvImg, img, blueLow, blueHigh, blue);
                // Call the getCones function for yellow
                getCones(hsvImg, img, yellowLow, yellowHigh, yellow);
                // Call the getDistance function
                getDistance();

                // Funciton to be used for measuring the preformance
                // double groundSteering = gsr.groundSteering();
                // calculatePerformance(groundSteering);

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "group_04; " << timestamp << "; " << steeringWheelAngle << std::endl;
                }
                // Display image on your screen.
                if (VERBOSE)
                {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}

void getCones(cv::Mat hsvImg, cv::Mat img, cv::Scalar low, cv::Scalar high, char color)
{
    // Create a mask from the hvs image that only contains the colors in the specified range
    cv::Mat mask;
    cv::inRange(hsvImg, low, high, mask);
    // Initiate a variable to store the outlines of the cones
    std::vector<std::vector<cv::Point>> contours;
    // Find the outlines of cones from the mask and assign them to the contours variable
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // Create a rectangle to store the previous position of the cone
    cv::Rect lastRect(0, 0, 0, 0);
    // Check if the color is blue
    if (color == blue)
    {
        // Go through all the contours
        for (size_t i = 0; i < contours.size(); ++i)
        {
            // Get a rectangle from the current outline
            cv::Rect boundRect = cv::boundingRect(contours[i]);
            // Check that the rectangle isn't too big or too small to eliminate noise
            if ((boundRect.area() > 200) && boundRect.area() < 1500)
            {
                // A blue cone has been found
                isBlue = true;
                // Check if the current cone is in the same position as the previous one
                if (boundRect.y > lastRect.y)
                {
                    // Draw the rectangle on the source image
                    // boundRect.tl() is the top left of the rectangle; boundRect.br() is the bottom right corner
                    cv::rectangle(img, boundRect.tl(), boundRect.br(), cvScalar(0, 255, 0), 3);
                }
                // Update the last rectangle
                lastRect = boundRect;
            }
        }
        // Save the closest rectangle to the blue cone variable
        blueCones = cv::Point(lastRect.x + lastRect.width / 2, lastRect.y + lastRect.height / 2);
        // Update the lastBlueCone variable with the new position
        if (!fBlue)
        {
            fBlue = true;
            lastBlueCone = blueCones;
            // Check if the blue cone is on the left or right side of the image
        }
        if (blueCones.x < car.x && isMirrored)
        {
            bLeft = true;
            yLeft = false;
            isMirrored = false;
        }
    }
    // Check if the color is yellow
    else if (color == yellow)
    {
        // Go through all the contours
        for (size_t i = 0; i < contours.size(); ++i)
        {
            // Get a rectangle from the current outline
            cv::Rect boundRect = cv::boundingRect(contours[i]);
            // Check that the rectangle isn't too big or too small to eliminate noise
            if ((boundRect.area() > 200) && boundRect.area() < 1500)
            {
                // change the isYellow variable to true
                isYellow = true;
                // Check if the current cone is in the same position as the previous one
                if (boundRect.y > lastRect.y)
                {
                    // Draw the rectangle on the source image
                    // boundRect.tl() is the top left of the rectangle; boundRect.br() is the bottom right corner
                    cv::rectangle(img, boundRect.tl(), boundRect.br(), cvScalar(0, 255, 0), 3);
                }
                // Update the last rectangle
                lastRect = boundRect;
            }
        }
        // Save the closest rectangle to the yellow cone variable
        yellowCones = cv::Point(lastRect.x + lastRect.width / 2, lastRect.y + lastRect.height / 2);
        // Update the lastYellowCone variable with the new position
        if (!fYellow)
        {
            fYellow = true;
            lastYellowCone = yellowCones;
        }
        // Check if the yellow cone is on the left or right side of the image
        if (yellowCones.x < car.x && isMirrored)
        {
            bLeft = false;
            yLeft = true;
            isMirrored = false;
        }
    }
}

void getDistance()
{
    // If both blue and yellow cones are found
    if (isBlue && isYellow)
    {
        // Set the steering wheel angle to 0
        steeringWheelAngle = 0.049;
    }
    else
    {
        // Initialize variable to store the differance between the cones and the car
        double difference = 0.0;
        // Check if the blue cone is on the left side of the image
        if (bLeft)
        {
            // Check if we have a blue cone in frame
            if (isBlue)
            {
                // Set the differance to the difference between the car and the blue cone
                difference = static_cast<double>(blueCones.x) / static_cast<double>(car.x);
                // Check that the calculation is in the correct range
                if (abs(difference) < threshold || abs(difference) > threshold2)
                {
                    steeringWheelAngle = -0.049;
                }
                else
                {
                    // Check if the current blue cone is the same as the last blue cone
                    if (blueCones.x > lastBlueCone.x)
                    {
                        // Call the steer function with the direction and the differance to calculate the steering wheel angle
                        calculate(right, difference || abs(difference) > threshold2);
                    }
                    else
                    {
                        steeringWheelAngle = -0.049;
                    }
                }
            }
            // Check if we have a yellow cone in frame
            else if (isYellow)
            {
                // Set the differance to the difference between the car and the yellow cone
                difference = static_cast<double>(car.x) / static_cast<double>(yellowCones.x);
                // Check that the calculation is in the correct range
                if (abs(difference) < threshold || abs(difference) > threshold2)
                {
                    steeringWheelAngle = 0.049;
                }
                else
                {
                    // Check if the current yellow cone is the same as the last yellow cone
                    if (yellowCones.y == lastYellowCone.y)
                    {
                        steeringWheelAngle = 0.049;
                    }
                    else if (yellowCones.y > lastYellowCone.y)
                    {
                        calculate(left, difference);
                    }
                }
            }
            // Set the differance to 0
            difference = 0.0;
        }
        // Check if the yellow cone is on the left side of the image
        else if (yLeft)
        {
            // Check if we have a yellow cone in frame
            if (isYellow)
            {
                // Set the differance to the difference between the car and the yellow cone
                difference = static_cast<double>(yellowCones.x) / static_cast<double>(car.x);
                // Check that the calculation is in the correct range
                if (abs(difference) < threshold || abs(difference) > threshold2)
                {
                    steeringWheelAngle = -0.049;
                }
                else
                {
                    // Check if the current yellow cone is the same as the last yellow cone
                    if (yellowCones.x > lastYellowCone.x)
                    {
                        calculate(right, difference);
                    }
                    else
                    {
                        steeringWheelAngle = -0.049;
                    }
                }
            }
            // Check if we have a blue cone in frame
            else if (isBlue)
            {
                // Set the differance to the difference between the car and the blue cone
                difference = static_cast<double>(car.x) / static_cast<double>(blueCones.x);
                // Check that the calculation is in the correct range
                if (abs(difference) < threshold || abs(difference) > threshold2)
                {
                    steeringWheelAngle = 0.049;
                }
                else
                {
                    if (blueCones.y == lastBlueCone.y)
                    {
                        steeringWheelAngle = 0.049;
                    }
                    else if (blueCones.y > lastBlueCone.y)
                    {
                        calculate(left, difference);
                    }
                }
            }
            // Set the differance to 0
            difference = 0.0;
        }
        else
        {
            steeringWheelAngle = 0.049;
        }
    }
    // Set the last blue cone to the current blue cone
    lastBlueCone = blueCones;
    // Set the last yellow cone to the current yellow cone
    lastYellowCone = yellowCones;
}

void calculate(char direction, double difference)
{
    int negative = 1;
    // Check if the direction is right
    if (direction == right)
    {
        // Check if the steering wheel angle more then 0
        if (steeringWheelAngle > 0.49)
        {
            // Set the steering wheel angle to 0
            steeringWheelAngle = 0.49;
        }
        negative = -1;
    }
    // Check if the direction is left
    else if (direction == left)
    {
        // Check if the steering wheel angle less then 0
        if (steeringWheelAngle < -0.49)
        {
            // Set the steering wheel angle to 0
            steeringWheelAngle = -0.49;
        }
        negative = 1;
    }
    // based if the differance, create a value that is between -0.29 and 0.29
    steeringWheelAngle = 0.09 * (1 + difference) * negative;
    // check if the steering wheel angle is more then 0.29
    if (steeringWheelAngle > magicNumber)
    {
        // Set the steering wheel angle to 0.29
        steeringWheelAngle = magicNumber;
    } // check if the steering wheel angle is less then -0.29
    else if (steeringWheelAngle < -(magicNumber))
    {
        // Set the steering wheel angle to -0.29
        steeringWheelAngle = -(magicNumber);
    }
}
