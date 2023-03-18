#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;

Mat frame, Matrix, framePerspective, frameGray, frameThreshold, frameCanny, frameFinal, frameFinalDuplicate;
Mat ROILane;

int leftLanePos, rightLanePos, laneCenterPos, frameCenter, Result;

RaspiCam_Cv Camera;

stringstream ss;

vector<int> histrogramLane;

Point2f Source[] = {Point2f(75, 160), Point2f(310, 160), Point2f(50, 210), Point2f(335, 210)};
Point2f Destination[] = {Point2f(80, 0), Point2f(280, 0), Point2f(80, 240), Point2f(280, 240)};

void Setup(int argc, char **argv, RaspiCam_Cv &Camera)
{
    Camera.set(CAP_PROP_FRAME_WIDTH, ("-w", argc, argv, 360));
    Camera.set(CAP_PROP_FRAME_HEIGHT, ("-h", argc, argv, 240));
    Camera.set(CAP_PROP_BRIGHTNESS, ("-br", argc, argv, 70));
    Camera.set(CAP_PROP_CONTRAST, ("-co", argc, argv, 50));
    Camera.set(CAP_PROP_SATURATION, ("-sa", argc, argv, 50));
    Camera.set(CAP_PROP_GAIN, ("-g", argc, argv, 50));
    Camera.set(CAP_PROP_FPS, ("-fps", argc, argv, 100));
}

void Perspective()
{
    line(frame, Source[0], Source[1], Scalar(0, 0, 255), 2);
    line(frame, Source[1], Source[3], Scalar(0, 0, 255), 2);
    line(frame, Source[3], Source[2], Scalar(0, 0, 255), 2);
    line(frame, Source[2], Source[0], Scalar(0, 0, 255), 2);

    // line (frame, Destination[0], Destination[1], Scalar(0,255,0), 2);
    // line (frame, Destination[1], Destination[3], Scalar(0,255,0), 2);
    // line (frame, Destination[3], Destination[2], Scalar(0,255,0), 2);
    // line (frame, Destination[2], Destination[0], Scalar(0,255,0), 2);

    Matrix = getPerspectiveTransform(Source, Destination);
    warpPerspective(frame, framePerspective, Matrix, Size(360, 240));
}

void Threshold()
{
    cvtColor(framePerspective, frameGray, COLOR_RGB2GRAY);

    inRange(frameGray, 120, 255, frameThreshold); // This can be change 230 to reduce the noise beside the road increase to remove the noise

    Canny(frameGray, frameCanny, 600, 700, 3, false);

    add(frameThreshold, frameCanny, frameFinal);

    cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);

    cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR); // Used in histrogram function only
}

void Capture()
{
    Camera.grab();
    Camera.retrieve(frame);
    // Convert BGR to RGB color
    // cvtColor(frame, frame, COLOR_BGR2RGB);
}

void Histrogram()
{
    histrogramLane.resize(400);
    histrogramLane.clear();

    for (int i = 0; i < frame.size().width; i++) // frame.size().width = 400
    {
        ROILane = frameFinalDuplicate(Rect(i, 140, 1, 100));
        divide(255, ROILane, ROILane);
        histrogramLane.push_back((int)(sum(ROILane)[0]));
    }
}

void laneFinder()
{
    vector<int>::iterator leftPtr;
    leftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 150);
    leftLanePos = distance(histrogramLane.begin(), leftPtr);

    vector<int>::iterator rightPtr;
    rightPtr = max_element(histrogramLane.begin() + 250, histrogramLane.end());
    rightLanePos = distance(histrogramLane.begin(), rightPtr);

    line(frameFinal, Point2f(leftLanePos, 0), Point2f(leftLanePos, 240), Scalar(0, 255, 0), 2);
    line(frameFinal, Point2f(rightLanePos, 0), Point2f(rightLanePos, 240), Scalar(0, 255, 0), 2);
}

void laneCenter()
{
    laneCenterPos = (rightLanePos - leftLanePos) / 2 + leftLanePos;

    frameCenter = 178; // Change the value to collabs the middle line with the default frame center.

    line(frameFinal, Point2f(laneCenterPos, 0), Point2f(laneCenterPos, 240), Scalar(0, 255, 0), 3);
    line(frameFinal, Point2f(frameCenter, 0), Point2f(frameCenter, 240), Scalar(255, 0, 0), 3);

    Result = laneCenterPos - frameCenter;
}

int main(int argc, char **argv)
{
    wiringPiSetup();
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);

    Setup(argc, argv, Camera);
    cout << "Connecting to camera" << endl;
    if (!Camera.open())
    {

        cout << "Failed to Connect" << endl;
    }

    cout << "Camera Id = " << Camera.getId() << endl;

    while (1)
    {
        auto start = std::chrono::system_clock::now();
        Capture();
        Perspective();
        Threshold();
        Histrogram();
        laneFinder();
        laneCenter();

        if (Result == 0)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 0); // Decimal = 0
            digitalWrite(23, 0);
            digitalWrite(24, 0);

            cout << "Forward"
                 << "\n";
        }
        else if (Result > 0 && Result < 10)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 0); // Decimal = 1
            digitalWrite(23, 0);
            digitalWrite(24, 0);

            cout << "Right1"
                 << "\n";
        }
        else if (Result >= 10 && Result < 20)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 1); // Decimal = 2
            digitalWrite(23, 0);
            digitalWrite(24, 0);

            cout << "Right2"
                 << "\n";
        }
        else if (Result > 20)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 1); // Decimal = 3
            digitalWrite(23, 0);
            digitalWrite(24, 0);

            cout << "Right3"
                 << "\n";
        }
        else if (Result < 0 && Result > -10)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 0); // Decimal = 4
            digitalWrite(23, 1);
            digitalWrite(24, 0);

            cout << "Left1"
                 << "\n";
        }
        else if (Result <= -10 && Result > -20)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 0); // Decimal = 5
            digitalWrite(23, 1);
            digitalWrite(24, 0);

            cout << "Left2"
                 << "\n";
        }
        else if (Result < -20)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 1); // Decimal = 6
            digitalWrite(23, 1);
            digitalWrite(24, 0);

            cout << "Left3"
                 << "\n";
        }

        ss.str(" ");
        ss.clear();
        ss << "Result = " << Result;
        putText(frame, ss.str(), Point2f(1, 50), 0, 1, Scalar(0, 0, 255), 2);

        namedWindow("Original", WINDOW_KEEPRATIO);
        moveWindow("Original", 0, 100);
        resizeWindow("Original", 640, 480);
        imshow("Original", frame);

        namedWindow("Perspective", WINDOW_KEEPRATIO);
        moveWindow("Perspective", 640, 100);
        resizeWindow("Perspective", 640, 480);
        imshow("Perspective", framePerspective);

        namedWindow("Final", WINDOW_KEEPRATIO);
        moveWindow("Final", 1280, 100);
        resizeWindow("Final", 640, 480);
        imshow("Final", frameFinal);

        waitKey(1);
        auto end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;

        float time = elapsed_seconds.count();
        int FPS = 1 / time;
        cout << "FPS = " << FPS << endl;

        waitKey(1);
    }

    return 0;
}
