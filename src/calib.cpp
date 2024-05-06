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
            cout << "Image chosen" << endl;
            return true;
        }
        else if (key == 27) // ESC 表示不选择该图像
        {   
            cout << "Skip this image" << endl;
            return false;
        }
    }
}

void saveMat(const cv::Mat& matrix, const string& txtName)
{
    ofstream file(txtName);
    if (!file.is_open()) {
        cerr << "Fail to open " << txtName << endl;
        return;
    }

    file << matrix << endl;
    file.close();
}

int main()
{
    fs::path calib_output_dir = "/home/fyh/data/GPcar";
    fs::path root_dir = "/home/fyh/data/GPcar/calib";

    vector<fs::path> cams = {
        "cam_side_right",
        "cam_front_30",
    };

    bool more[2] = {true, false};

    // Defining the world coordinates for 3D points
	// 为三维点定义世界坐标系
	std::vector<cv::Point3f> objp;
	for (int i = 0; i < BOARD_SIZE[1]; i++)
	{
		for (int j = 0; j < BOARD_SIZE[0]; j++)
		{
			objp.push_back(cv::Point3f(j, i, 0));
		}
	}

    int midx = 0;
    for (const auto& cam : cams) {
        fs::path cam_txt = (root_dir / cam).replace_extension(".txt");
        cout << cam_txt << endl;
        vector<fs::path> folders;
        getFolders(cam_txt, folders);

        // Creating vector to store vectors of 3D points for each checkerboard image
        // 创建矢量以存储每个棋盘图像的三维点矢量
        std::vector<std::vector<cv::Point3f> > objpoints;

        // Creating vector to store vectors of 2D points for each checkerboard image
        // 创建矢量以存储每个棋盘图像的二维点矢量
        std::vector<std::vector<cv::Point2f> > imgpoints;

        cv::Mat gray;

        cv::namedWindow("Image", cv::WINDOW_NORMAL);

        for (const auto& folder : folders) {
            fs::path img_folder = folder / cam;
            if (!fs::exists(img_folder)) {
                cout << "Folder " << folder << " does not exist." << endl;
                continue;
            }

            string path = img_folder / "*.jpg";
            
            vector<cv::String> img_files;
            cv::glob(path, img_files);

            for (const auto& img_file : img_files) {
                cv::Mat image = cv::imread(img_file);
                cout << "Finding... " << img_file << endl;
                if (image.empty()) {
                    cerr << "Fail to read " << img_file << endl;
                    continue;
                }
                
                if (chooseImage(image)) {
                    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
                    
                    vector<cv::Point2f> corner_pts;
                    if (cv::findChessboardCorners(gray, cv::Size(BOARD_SIZE[0], BOARD_SIZE[1]), corner_pts,
                                                  cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE))
                    {
                        // 继续给找到的角点细化
                        if (more[midx]) {
                            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::Type::MAX_ITER, 50, 0.0001);
                            
                            cv::cornerSubPix(gray, corner_pts, cv::Size(7, 7), cv::Size(-1, -1), criteria);
                        }
                        else {
                            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::Type::MAX_ITER, 30, 0.001);
                            
                            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
                        }

                        // Displaying the detected corner points on the checker board
                        // 在棋盘上显示检测到的角点
                        cv::drawChessboardCorners(image, cv::Size(BOARD_SIZE[0], BOARD_SIZE[1]), corner_pts, true);

                        cv::imshow("Image", image);
                        int key = cv::waitKey();
                        if(key == 27)   // 如果是 ESC 就舍弃
                        {
                            cout << "result dropped" << endl;
                            continue;
                        }

                        objpoints.push_back(objp);
                        imgpoints.push_back(corner_pts);
                    }
                    else {
                        cerr << "Fail to find corners." << endl;
                        continue;
                    }
                }
            }
        }

        midx ++ ;

        cv::destroyAllWindows();

        cv::Mat cameraMatrix, distCoeffs, R, T;

        cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

        // 内参矩阵
        std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
        // 透镜畸变系数
        std::cout << "distCoeffs : " << distCoeffs << std::endl;
        // rvecs
        // std::cout << "Rotation vector : " << R << std::endl;
        // // tvecs
        // std::cout << "Translation vector : " << T << std::endl;

        ofstream file((calib_output_dir / cam).replace_extension(".txt").string());
        if (file.is_open()) {
            for (int i = 0; i < cameraMatrix.rows; i ++ ) {
                for (int j = 0; j < cameraMatrix.cols; j ++ ) {
                    file << cameraMatrix.at<double>(i, j) << " ";
                }
                file << endl;
            }
            for (int i = 0; i < distCoeffs.rows; i ++ ) {
                for (int j = 0; j < distCoeffs.cols; j ++ ) {
                    file << distCoeffs.at<double>(i, j) << " ";
                }
                file << endl;
            }
            cout << "Matrix written." << endl;
        }
        else
            cerr << "Fail to write cameraMatrix." << endl;
        file.close();
    }

    return 0;

}
