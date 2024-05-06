#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

const int IMG_P_FOLDER = 5;
const int BOARD_SIZE[2] = {8, 11};    // 棋盘格每行、列的内角点数，也即（行数-1）和（列数-1）

void getFolders(const string& txtPath, vector<fs::path>& folders)
{
    ifstream file(txtPath);
    if (!file.is_open()) {
        cerr << "Failed to open file " << txtPath << endl;
        return;
    }

    string line;
    while(getline(file, line)) {
        folders.push_back(fs::path(line));
    }

    file.close();
}

bool chooseImage(const cv::Mat& image, const string& window="Image") {
    cv::imshow(window, image);

    while (true) {
        int key = cv::waitKey(0);
        if (key == 13)  // 回车表示选择该图像
        {
            cout << window << " chosen" << endl;
            return true;
        }
        else if (key == 27) // ESC 表示不选择该图像
        {   
            cout << "Skip " << window << endl;
            return false;
        }
    }
}

void readCameraParameters(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    cameraMatrix = cv::Mat::zeros(cv::Size(3, 3), CV_64F);
    distCoeffs = cv::Mat::zeros(cv::Size(5, 1), CV_64F);

    double value;
    std::string line;

    for (int i = 0; i < 3; i ++ ) {
        std::getline(file, line);
        std::istringstream iss(line);
        int idx = 0;
        while (iss >> value) 
            cameraMatrix.at<double>(i, idx ++ ) = value;
    }

    std::getline(file, line);
    std::istringstream iss(line);
    int idx = 0;
    while (iss >> value)
        distCoeffs.at<double>(0, idx ++ ) = value;
}

int main()
{
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

    // Creating vector to store vectors of 3D points for each checkerboard image
    // 创建矢量以存储每个棋盘图像的三维点矢量
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    // 创建矢量以存储每个棋盘图像的二维点矢量
    std::vector<std::vector<cv::Point2f> > leftpoints;
    std::vector<std::vector<cv::Point2f> > rightpoints;

    std::vector<cv::Point3f> objp;
	for (int i = 0; i < BOARD_SIZE[1]; i++)
	{
		for (int j = 0; j < BOARD_SIZE[0]; j++)
		{
			objp.push_back(cv::Point3f(j, i, 0));
		}
	}

    cv::Mat leftframe, rightframe;
    cv::Mat leftgray, rightgray;

    cv::namedWindow("Left", cv::WINDOW_NORMAL);
    cv::namedWindow("Right", cv::WINDOW_NORMAL);

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

            cv::imshow("Left", leftframe);
            cv::imshow("Right", rightframe);
            int key = cv::waitKey(0);
            if (key == 27)  // ESC 则不使用当前帧
                continue;
            
            cv::cvtColor(leftframe, leftgray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(rightframe, rightgray, cv::COLOR_BGR2GRAY);

            vector<cv::Point2f> left_pts;
            vector<cv::Point2f> right_pts;

            if (cv::findChessboardCorners(leftgray, cv::Size(BOARD_SIZE[0], BOARD_SIZE[1]), left_pts,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE) && 
                cv::findChessboardCorners(rightgray, cv::Size(BOARD_SIZE[0], BOARD_SIZE[1]), right_pts,
                                          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE)) 
            {
                // 细化角点
                cv::TermCriteria right_criteria(cv::TermCriteria::EPS | cv::TermCriteria::Type::MAX_ITER, 50, 0.0001);
                cv::cornerSubPix(rightgray, right_pts, cv::Size(7, 7), cv::Size(-1, -1), right_criteria);

                cv::TermCriteria left_criteria(cv::TermCriteria::EPS | cv::TermCriteria::Type::MAX_ITER, 30, 0.001);
                cv::cornerSubPix(leftgray, left_pts, cv::Size(11, 11), cv::Size(-1, -1), left_criteria);

                cv::drawChessboardCorners(leftframe, cv::Size(BOARD_SIZE[0], BOARD_SIZE[1]), left_pts, true);
                cv::drawChessboardCorners(rightframe, cv::Size(BOARD_SIZE[0], BOARD_SIZE[1]), right_pts, true);

                cv::imshow("Left", leftframe);
                cv::imshow("Right", rightframe);

                int key = cv::waitKey(0);
                if(key == 27)   // 如果是 ESC 就舍弃
                {
                    cout << "result dropped" << endl;
                    continue;
                }
                else if (key == 13) {
                    cout << "result chosen." << endl;
                }

                objpoints.push_back(objp);
                leftpoints.push_back(left_pts);
                rightpoints.push_back(right_pts);
            }
            else {
                cerr << "Fail to find corners." << endl;
                continue;
            }
        }

        //
    }
    cv::destroyAllWindows();
    //
    cv::Mat left_cameraMatrix, right_cameraMatrix;
    cv::Mat left_distCoeffs, right_distCoeffs;
    string left_matrix = "/home/fyh/data/GPcar/cam_front_30.txt";
    string right_matrix = "/home/fyh/data/GPcar/cam_side_right.txt";

    readCameraParameters(left_matrix, left_cameraMatrix, left_distCoeffs);
    readCameraParameters(right_matrix, right_cameraMatrix, right_distCoeffs);

    cv::Mat R, T, E, F;

    cv::Size size_2k(2560, 1440);

    double rmsStereo = cv::stereoCalibrate(objpoints, leftpoints, rightpoints, 
                                            left_cameraMatrix, left_distCoeffs, 
                                            right_cameraMatrix, right_distCoeffs, 
                                            size_2k, R, T, E, F, cv::CALIB_FIX_INTRINSIC | cv::CALIB_USE_INTRINSIC_GUESS);

    std::cout << "重投影误差：" << rmsStereo << std::endl;
    std::cout << "旋转矩阵 R: " << std::endl << R << std::endl;
    std::cout << "平移向量 T: " << std::endl << T << std::endl;

    ofstream file("home/fyh/data/GPcar/extrinsic.txt");
    if (file.is_open()) {
        for (int i = 0; i < R.rows; i ++ ) {
            for (int j = 0; j < R.cols; j ++ ) {
                file << R.at<double>(i, j) << " ";
            }
            file << endl;
        }
        file << endl;
        for (int i = 0; i < T.rows; i ++ ) {
            for (int j = 0; j < T.cols; j ++ ) {
                file << T.at<double>(i, j) << " ";
            }
            file << endl;
        }
        cout << "Matrix written." << endl;
    }
    else
        cerr << "Fail to write cameraMatrix." << endl;
    file.close();

    return 0;
}