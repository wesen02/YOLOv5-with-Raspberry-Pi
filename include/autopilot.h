#include <iostream>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>
#include <fstream>
#include <cmath>
#include <random>
#include <chrono>
#include <wiringPi.h>
#include <raspicam_cv.h>
#include <ctime>

using namespace cv;
using namespace std;
using namespace dnn;
using namespace raspicam;

struct Detection
{
    int class_id = 0;
    string className;
    float confidence = 0.0;
    Scalar color;
    Rect box;
};

class Yolo
{
public:
    Yolo();

private:
    Net net;

    void loadClasses();
    void loadModel();
    void loadSource();
    void detect(Mat &frame);

    void Setup(RaspiCam_Cv &Camera);
    string modelPath = "../source/model/yolov5n.onnx";
    string classPath = "../source/classes/classes.txt";
    string colorPath = "../source/classes/colors.txt";

    float modelConfidenceThreshold = 0.25;
    float modelScoreThreshold = 0.45;
    float modelNMSThreshold = 0.5;

    bool letterBoxForSquare = false;

    Size2f modelShape = Size(320, 320);

    Mat formatToSquare(const Mat &source);

    vector<string> classes;

    vector<Detection> detections{};
    vector<Detection> output;

    vector<Scalar> colors;

    void drawPred(int classId, float conf, int left, int top,
                  int right, int bottom, Mat &frame);
    void readColors();

    vector<int> histrogramLane;
    int leftLanePos, rightLanePos, laneCenterPos, frameCenter, Result;
    int dist_stop;
    stringstream ss;

    Mat original, frame, detectFrame, Matrix, framePerspective,
        frameGray, frameThreshold, frameCanny, frameFinal,
        frameFinalDuplicate, ROILane;

    void Perspective();
    void Threshold();
    void Histrogram();
    void laneFinder();
    void laneCenter();
};