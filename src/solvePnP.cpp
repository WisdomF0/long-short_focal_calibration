#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

void readImgCorner(string filename, vector<cv::Point2d>& corners)
{
    ifstream file(filename);
    string line;
    double x, y;

    while(getline(file, line)) {
        for (int i = 0; i < 4; i ++ ) {
            getline(file, line);
            getline(file, line);
            istringstream iss(line);
            iss >> x >> y;
            corners.push_back(cv::Point2d(x, y));
        }
    }

    file.close();
}

void readLidarCorner(string filename, vector<cv::Point3d>& corners)
{
    ifstream file(filename);
    string line;
    double x, y, z;

    while(getline(file, line)) {
        for (int i = 0; i < 4; i ++ ) {
            getline(file, line);
            getline(file, line);
            istringstream iss(line);
            iss >> x >> y >> z;
            corners.push_back(cv::Point3d(x, y, z));
        }
    }

    file.close();
}

int main()
{
    string img_file = "/home/fyh/data/GPcar/calib/front_30_corners.txt";
    string lid_file = "/home/fyh/data/GPcar/calib/lidar_corners.txt";
    vector<cv::Point2d> img_corners;
    vector<cv::Point3d> lid_corners;

    readImgCorner(img_file, img_corners);
    readLidarCorner(lid_file, lid_corners);

    // cout << "img corners: " << img_corners.size() << endl;
    // cout << "img corner 1: " << img_corners[0].x << "," << img_corners[0].y << endl;
    // cout << "lid corners: " << lid_corners.size() << endl;
    // cout << "lid corner 1: " << lid_corners[0].x << "," << lid_corners[0].y << "," << lid_corners[0].z << endl;

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 7216.899221372676, 0, 1667.892712874692,
                                                      0, 7214.795790605108, 984.6361655074427,
                                                      0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

    cv::solvePnP(lid_corners, img_corners, cameraMatrix, distCoeffs, rvec, tvec);

    cout << "rvec: " << endl << rvec << endl;
    cout << "tvec: " << endl << tvec << endl;

    double rm[9];
    cv::Mat rotM(3, 3, CV_64FC1, rm);
    cv::Rodrigues(rvec, rotM);
    cout << "Rotation Matrix: " << endl << rotM << endl;

    // result:
    // rvec: 
    // [-1.253648309634973;
    // -1.231610530992679;
    // -1.156422139470272]
    // tvec: 
    // [-0.5147208961758304;
    // -0.1098993653591364;
    // -11.03045338694802]
    // Rotation Matrix: 
    // [0.02744308909944382, 0.9995709528095927, -0.01023655996919065;
    // 0.05266535198153266, 0.008780425965069671, 0.9985736151233584;
    // 0.9982350612763213, -0.02794305672604164, -0.05240179404814277]

    return 0;
}