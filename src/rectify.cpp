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
    string left_camera_txt = "/home/fyh/data/GPcar/cam_front_30_undistorted.txt";
    string right_camera_txt = "/home/fyh/data/GPcar/cam_side_right_undistorted.txt";
    string extrinsic_txt = "/home/fyh/data/GPcar/extrinsic.txt";

    // cv::Mat Kl, Kr, Dl, Dr, R_rl, T_rl;
    // readINTParameters(left_camera_txt, Kl, Dl);
    // readINTParameters(right_camera_txt, Kr, Dr);
    // readEXTParameters(extrinsic_txt, R_rl, T_rl);
    // cout << Dl << endl;
    // cout << Dr << endl;

    cv::Mat Kl = (cv::Mat_<double>(3, 3) << 5075.705706803091, -26.955546561137076, 1239.0383093198182,
                                            0.0, 5076.262034300842, 578.6498015759931,
                                            0.0, 0.0, 1.0);
    cv::Mat Kr = (cv::Mat_<double>(3, 3) << 1394.4587015095824, 2.807374442696875, 1225.0312259198076,
                                            0.0, 1476.288224276559, 689.4153922809646,
                                            0.0, 0.0, 1.0);
    cv::Mat Dl = (cv::Mat_<double>(1, 5) << -0.22704271021659853, 1.5258030740216681, -0.01046364192558752, 0.008777958202082443, -12.489737783059466);
    cv::Mat Dr = (cv::Mat_<double>(1, 5) << 0.015340549897953791, -2.527123692514174, -0.005624479491394424, -0.007288253807626851, 28.062174734978758);
    cv::Mat R_rl = (cv::Mat_<double>(3, 3) << 0.9997786881136527, 0.0046600553034099315, 0.020514840440834188,
                                            -0.004538941509713027, 0.9999720192187217, -0.005946325722985069,
                                            -0.02054197662629205, 0.0058518940695804515, 0.9997718652433081);
    cv::Mat T_rl = (cv::Mat_<double>(3, 1) << -93.52047759791571, -1.7942793233303884, -9.658476868228188);
    cout << Dl << endl;
    cout << T_rl << endl;

    cv::Size size_2k(2560, 1440);

    vector<fs::path> folders = {
        "/home/fyh/data/GPcar/calib/2024-04-22-17-05-48",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-06-01",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-06-19",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-09-14",
    };

    vector<fs::path> cam = {
        "cam_front_30_undistorted",
        "cam_side_right_undistorted",
    };

    cv::Mat leftframe, rightframe, origin;

    cv::namedWindow("Origin", cv::WINDOW_NORMAL);
    cv::namedWindow("Rectify", cv::WINDOW_NORMAL);

    const int IMG_P_FOLDER = 5;
    double scale = 4.0;

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

            cv::stereoRectify(Kl, Dl, Kr, Dr, size_2k, R_rl, T_rl, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY);
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
            cv::remap(ltemp, lframe_rtf, undistmap1l, undistmap2l, cv::INTER_LINEAR);
            cv::remap(rtemp, rframe_rtf, undistmap1r, undistmap2r, cv::INTER_LINEAR);

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
