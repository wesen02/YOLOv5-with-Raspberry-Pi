#include <autopilot.h>

Yolo::Yolo()
{
    RaspiCam_Cv camera;
    Setup(camera);
    cout << "Connecting to camera" << endl;

    if (!camera.open())
    {
        cerr << "Error opening camera" << endl;
        return;
    }

    cout << "Camera Id = " << camera.getId() << endl;

    loadModel();
    loadClasses();
    readColors();

    wiringPiSetup();
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);

    clock_t start;
    clock_t end;
    double ms, fpsLive, seconds;
    while (true)
    {
        start = clock();
        camera.grab();
        camera.retrieve(original);

        frame = original.clone();
        detectFrame = original.clone();

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

            // cout << "Forward"
            //      << "\n";
        }
        else if (Result > 0 && Result < 10)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 0); // Decimal = 1
            digitalWrite(23, 0);
            digitalWrite(24, 0);

            // cout << "Right1"
            //      << "\n";
        }
        else if (Result >= 10 && Result < 20)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 1); // Decimal = 2
            digitalWrite(23, 0);
            digitalWrite(24, 0);

            // cout << "Right2"
            //      << "\n";
        }
        else if (Result > 20)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 1); // Decimal = 3
            digitalWrite(23, 0);
            digitalWrite(24, 0);

            // cout << "Right3"
            //      << "\n";
        }
        else if (Result < 0 && Result > -10)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 0); // Decimal = 4
            digitalWrite(23, 1);
            digitalWrite(24, 0);

            // cout << "Left1"
            //      << "\n";
        }
        else if (Result <= -10 && Result > -20)
        {
            digitalWrite(21, 1);
            digitalWrite(22, 0); // Decimal = 5
            digitalWrite(23, 1);
            digitalWrite(24, 0);

            // cout << "Left2"
            //      << "\n";
        }
        else if (Result < -20)
        {
            digitalWrite(21, 0);
            digitalWrite(22, 1); // Decimal = 6
            digitalWrite(23, 1);
            digitalWrite(24, 0);

            // cout << "Left3"
            //      << "\n";
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

        detect(detectFrame);
        namedWindow("Detect", WINDOW_KEEPRATIO);
        moveWindow("Detect", 0, 580);
        resizeWindow("Detect", 640, 480);
        end = clock();
        seconds = (double(end) - double(start)) / double(CLOCKS_PER_SEC);
        int(fpsLive) = 1.0 / double(seconds);
        putText(detectFrame, "FPS: " + to_string(fpsLive), {50, 100},
                FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1);
        imshow("Detect", detectFrame);

        char key = waitKey(1);
        if (key == 'q')
        {
            break;
        }
    }
    cout << "Closing camera..."
         << "\n";
    camera.release();
}

void Yolo::Perspective()
{
    Point2f Source[] = {Point2f(75, 160), Point2f(310, 160), Point2f(50, 210), Point2f(335, 210)};
    Point2f Destination[] = {Point2f(80, 0), Point2f(280, 0), Point2f(80, 240), Point2f(280, 240)};

    line(frame, Source[0], Source[1], Scalar(0, 0, 255), 2);
    line(frame, Source[1], Source[3], Scalar(0, 0, 255), 2);
    line(frame, Source[3], Source[2], Scalar(0, 0, 255), 2);
    line(frame, Source[2], Source[0], Scalar(0, 0, 255), 2);

    Matrix = getPerspectiveTransform(Source, Destination);
    warpPerspective(frame, framePerspective, Matrix, Size(360, 240));
}

void Yolo::Threshold()
{
    cvtColor(framePerspective, frameGray, COLOR_RGB2GRAY);

    inRange(frameGray, 120, 255, frameThreshold); // This can be change 230 to reduce the noise beside the road increase to remove the noise

    Canny(frameGray, frameCanny, 600, 700, 3, false);

    add(frameThreshold, frameCanny, frameFinal);

    cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);

    cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR); // Used in histrogram function only
}

void Yolo::Histrogram()
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

void Yolo::laneFinder()
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

void Yolo::laneCenter()
{
    laneCenterPos = (rightLanePos - leftLanePos) / 2 + leftLanePos;

    frameCenter = 178; // Change the value to collabs the middle line with the default frame center.

    line(frameFinal, Point2f(laneCenterPos, 0), Point2f(laneCenterPos, 240), Scalar(0, 255, 0), 3);
    line(frameFinal, Point2f(frameCenter, 0), Point2f(frameCenter, 240), Scalar(255, 0, 0), 3);

    Result = laneCenterPos - frameCenter;
}

void Yolo::Setup(RaspiCam_Cv &Camera)
{
    Camera.set(CAP_PROP_FRAME_WIDTH, 360);
    Camera.set(CAP_PROP_FRAME_HEIGHT, 240);
    // Camera.set(CAP_PROP_BRIGHTNESS, 70);
    // Camera.set(CAP_PROP_CONTRAST, 50);
    // Camera.set(CAP_PROP_SATURATION, 50);
    // Camera.set(CAP_PROP_GAIN, 50);
    // Camera.set(CAP_PROP_FPS, 100);
}

void Yolo::loadModel()
{
    cout << "OpenCV version : " << CV_VERSION << endl;

    try
    {
        this->net = readNetFromONNX(modelPath);
        // cuda
        if (cuda::getCudaEnabledDeviceCount() > 0)
        {
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
            cout << "This computer is using CUDA"
                 << "\n";
        }
        // cpu
        else
        {
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            cout << "This computer is using CPU"
                 << "\n";
        }
    }
    catch (const Exception &e)
    {
        cerr << "Error Loading the model " << e.what()
             << "\n";
        return;
    }
    cout << "weights loaded successfully"
         << "\n";
}

void Yolo::loadClasses()
{
    ifstream inputFile(classPath);
    if (inputFile.is_open())
    {
        cout << "Classes file opened"
             << "\n";
        string classLine;
        while (std::getline(inputFile, classLine))
            classes.push_back(classLine);
        inputFile.close();
    }
}

void Yolo::readColors()
{
    ifstream file(colorPath);
    if (!file.is_open())
    {
        cout << "Unable to open file: " << colorPath << "\n";
        return;
    }

    int r, g, b;
    while (file >> r >> g >> b)
    {
        colors.push_back(Scalar(b, g, r));
    }
    file.close();
}

void Yolo::detect(Mat &frame)
{
    Mat modelInput = frame;

    Mat blob;
    blobFromImage(modelInput, blob, 1.0 / 255.0, modelShape,
                  cv::Scalar(), true, false); // Error occur here because of the size of the resolution
    net.setInput(blob);

    vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    int rows = outputs[0].size[1];
    int dimensions = outputs[0].size[2];

    // int rows = 6300;
    // int dimensions = 6;

    // cout << "row:" << rows << "\n";
    // cout << "dimension:" << dimensions << "\n";

    bool yolov8 = false;

    float *data = (float *)outputs[0].data;

    // cout << "Data:" << data << "\n";

    float x_factor = modelInput.cols / modelShape.width;
    float y_factor = modelInput.rows / modelShape.height;

    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;

    for (int i = 0; i < rows; ++i)
    {
        float confidence = data[4];

        if (confidence >= modelConfidenceThreshold)
        {
            float *classes_scores = yolov8 ? data + 4 : data + 5;

            Mat scores(1, classes.size(), CV_32FC1, classes_scores);
            Point class_id;
            double max_class_score;

            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

            if (max_class_score > modelScoreThreshold)
            {
                confidences.push_back(confidence);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];

                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);

                int width = int(w * x_factor);
                int height = int(h * y_factor);

                boxes.push_back(Rect(left, top, width, height));
            }
        }

        data += dimensions;
    }

    vector<int> nms_result;
    NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];

        Rect box = boxes[idx];
        this->drawPred(class_ids[idx], confidences[idx], box.x, box.y,
                       box.x + box.width, box.y + box.height, frame);
    }
}

void Yolo::drawPred(int classId, float conf, int left, int top, int right,
                    int bottom, Mat &frame)
{
    if (classId == 11)
    {
        rectangle(frame, Point(left, top), Point(right, bottom), colors[classId], 3);
        dist_stop = (0.75) * (left - right) + 57;
        ss.str(" ");
        ss.clear();
        ss << "Distance = " << dist_stop << "cm";
        putText(frame, ss.str(), Point2f(1, 130), 0, 1, Scalar(0, 0, 255), 2);

        string label = format("%.2f", conf);
        label = this->classes[classId] + ":" + label;

        int baseLine;
        Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = max(top, labelSize.height);

        putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, colors[classId], 1);
    }

    // rectangle(frame, Point(left, top), Point(right, bottom), colors[classId], 3);
    // dist_stop = (0.75) * (left - right) + 57;
    // ss.str(" ");
    // ss.clear();
    // ss << "Distance = " << dist_stop << "cm";
    // putText(frame, ss.str(), Point2f(1, 130), 0, 1, Scalar(0, 0, 255), 2);

    // string label = format("%.2f", conf);
    // label = this->classes[classId] + ":" + label;

    // int baseLine;
    // Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    // top = max(top, labelSize.height);

    // putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, colors[classId], 1);
}

int main()
{
    Yolo yolo;

    return 0;
}