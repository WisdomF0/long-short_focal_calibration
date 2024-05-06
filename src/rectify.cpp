#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <fstream>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

void readINTParameters(const string& filename, cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Unable to open file " << filename << endl;
        return;
    }

    cameraMatrix = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
    distCoeffs = cv::Mat::zeros(cv::Size(5, 1), CV_64F);

    double value;
    string line;

    for (int i = 0; i < 3; i ++ ) {
        getline(file, line);
        istringstream iss(line);
        int idx = 0;
        while (iss >> value) 
            cameraMatrix.at<double>(i, idx ++ ) = value;
    }

    getline(file, line);
    istringstream iss(line);
    int idx = 0;
    while (iss >> value)
        distCoeffs.at<double>(0, idx ++ ) = value;
}

void readEXTParameters(const string& filename, cv::Mat& R, cv::Mat& T) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Unable to open file " << filename << endl;
        return;
    }

    R = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
    T = cv::Mat::zeros(cv::Size(1, 3), CV_64F);

    double value;
    string line;

    for (int i = 0; i < 3; i ++ ) {
        getline(file, line);
        istringstream iss(line);
        int idx = 0;
        while (iss >> value) 
            R.at<double>(i, idx ++ ) = value;
    }
    getline(file, line);
    getline(file, line);
    istringstream iss(line);
    int idx = 0;
    while (iss >> value)
        T.at<double>(idx ++ , 0) = value;
}

int main() 
{
    string left_camera_txt = "/home/fyh/data/GPcar/cam_front_30.txt";
    string right_camera_txt = "/home/fyh/data/GPcar/cam_side_right.txt";
    string extrinsic_txt = "/home/fyh/data/GPcar/extrinsic.txt";

    cv::Mat Kl, Kr, Dl, Dr, R_rl, T_rl;
    readINTParameters(left_camera_txt, Kl, Dl);
    readINTParameters(right_camera_txt, Kr, Dr);
    readEXTParameters(extrinsic_txt, R_rl, T_rl);

    cv::Size size_2k(2560, 1440);

    vector<fs::path> folders = {
        "/home/fyh/data/GPcar/calib/2024-04-22-17-05-48",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-06-01",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-06-19",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-09-14",
    };

    vector<fs::path> cam = {
        "cam_front_30",
        "cam_side_right",
    };

    cv::Mat leftframe, rightframe, origin;

    cv::namedWindow("Origin", cv::WINDOW_NORMAL);
    cv::namedWindow("Rectify", cv::WINDOW_NORMAL);

    const int IMG_P_FOLDER = 5;

    for (const auto& folder : folders) {
        fs::path left_folder = folder / cam[0];
        fs::path right_folder = folder / cam[1];

        string left_path = left_folder / "*.jpg";
        string right_path = right_folder / "*.jpg";
        vector<cv::String> left_files, right_files;
        cv::glob(left_path, left_files);
        cv::glob(right_path, right_files);

        for (int i = 0; i < IMG_P_FOLDER; i ++ ) {
            cout << "Left: " << left_files[i] << endl;
            cout << "Right: " << right_files[i] << endl;
            leftframe = cv::imread(left_files[i]);
            rightframe = cv::imread(right_files[i]);
            
            cv::Mat ltemp, rtemp;
            cv::resize(leftframe, ltemp, size_2k);
            cv::resize(rightframe, rtemp, size_2k);
            cv::hconcat(ltemp, rtemp, origin);
            cv::imshow("Origin", origin);
            if (cv::waitKey(0) == 27)  // 如果按下 ESC 就不使用当前帧
                continue;

            cv::Mat Rl, Rr, Pl, Pr, Q;
            cv::Mat undistmap1l, undistmap2l, undistmap1r, undistmap2r;

            cv::stereoRectify(Kl, Dl, Kr, Dr, size_2k, R_rl, T_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 0);
            cout << "stereo rectify finished." << endl;
            cv::initUndistortRectifyMap(Kl, Dl, Rl, Pl, size_2k, CV_16SC2, undistmap1l, undistmap2l);
            cv::initUndistortRectifyMap(Kr, Dr, Rr, Pr, size_2k, CV_16SC2, undistmap1r, undistmap2r);

            // 将 R_21 和 t_21 转换为两个校正相机坐标系的变换
            cv::Mat R, T;
            cout << "R_rl before rectification: " << endl << R_rl << endl;
            R = Rr * R_rl * Rl.t();
            cout << "R after rectification: " << endl << R << endl;

            cout << "t_rl before rectification: " << endl << T_rl.t() << endl;
            T = Rr * T_rl;
            cout << "t after rectification: " << endl << T.t() << endl;

            cout << "Pl: " << endl << Pl << endl;
            cout << "Pr: " << endl << Pr << endl;

            cv::Mat lframe_rtf, rframe_rtf;
            cv::remap(leftframe, lframe_rtf, undistmap1l, undistmap2l, cv::INTER_LINEAR);
            cv::remap(rightframe, rframe_rtf, undistmap1r, undistmap2r, cv::INTER_LINEAR);

            cv::hconcat(ltemp, rtemp, origin);
            cv::Mat rectify;
            cv::hconcat(lframe_rtf, rframe_rtf, rectify);

            for (int i = 1, iend = 8; i < iend; i ++ ) {
                int h = size_2k.height / iend * i;
                cv::line(origin, cv::Point2i(0, h), cv::Point2i(size_2k.width * 2, h), cv::Scalar(0, 0, 255));
                cv::line(rectify, cv::Point2i(0, h), cv::Point2i(size_2k.width * 2, h), cv::Scalar(0, 0, 255));
            }

            cv::imshow("Origin", origin);
            cv::imshow("Rectify", rectify);

            cv::waitKey();
        }
    }

    cv::destroyAllWindows();

    return 0;
}
