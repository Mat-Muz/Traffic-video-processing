#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Trackbar variables
int threshold1 = 100;
int threshold2 = 200;
int minLineLength = 100;
int maxLineGap = 10;

Mat background, gray, edges, lineImage;

void updateLines(int, void*) {
    if (background.empty()) return; // Safety check

    // Edge detection
    Canny(gray, edges, threshold1, threshold2, 3);

    // Hough line detection
    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 40, minLineLength, maxLineGap);

    // Draw raw Hough lines
    lineImage = background.clone();
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(lineImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2, LINE_AA);
    }

    imshow("Raw Hough Lines", lineImage);
    imshow("Canny Edges", edges);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "Usage: ./hough_debug <video_file_or_camera_index>" << endl;
        return -1;
    }

    VideoCapture cap;
    string filename = argv[1];
    istringstream iss(filename);
    int device;
    if (!(iss >> device)) {
        cap.open(filename); // Open as video file
    } else {
        cap.open(device);   // Open as camera
    }

    if (!cap.isOpened()) {
        cerr << "Error: Cannot open video/camera!" << endl;
        return -1;
    }

    Mat firstFrame;

    // Read first frame (ignored)
    cap.read(firstFrame);

    // Read second frame as background
    cap.read(background);
    if (background.empty()) {
        cerr << "Error: Failed to read the background frame!" << endl;
        return -1;
    }

    cout << "Background frame captured successfully: " << background.cols << "x" << background.rows << endl;

    // Convert background to grayscale
    cvtColor(background, gray, COLOR_BGR2GRAY);

    // Create windows
    namedWindow("Raw Hough Lines", WINDOW_AUTOSIZE);
    namedWindow("Canny Edges", WINDOW_AUTOSIZE);
    imshow("Background Frame", background);

    // Create trackbars
    createTrackbar("Canny Thresh1", "Raw Hough Lines", &threshold1, 500, updateLines);
    createTrackbar("Canny Thresh2", "Raw Hough Lines", &threshold2, 500, updateLines);
    createTrackbar("Min Line Len", "Raw Hough Lines", &minLineLength, 500, updateLines);
    createTrackbar("Max Line Gap", "Raw Hough Lines", &maxLineGap, 50, updateLines);

    // Initial display
    updateLines(0, 0);

    // Wait until ESC pressed
    while (true) {
        int key = waitKey(0);
        if (key == 27) { // ESC
            cout << "\nFinal Trackbar Values:" << endl;
            cout << "Canny Threshold1: " << threshold1 << endl;
            cout << "Canny Threshold2: " << threshold2 << endl;
            cout << "Min Line Length: " << minLineLength << endl;
            cout << "Max Line Gap: " << maxLineGap << endl;
            break;
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
