#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <unistd.h>

using namespace std;
namespace fs = std::filesystem;

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

int main()
{
    string undistort_list = "/home/fyh/data/GPcar/calib/undistort_list";
    vector<fs::path> folders;
    getFolders(undistort_list, folders);

    vector<fs::path> cams = {
        "cam_side_right",
        "cam_front_30",
    };

    fs::path root_dir = "/home/fyh/data/GPcar";
    
    cv::namedWindow("Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Undistorted", cv::WINDOW_NORMAL);

    for (auto& cam : cams) {
        fs::path txt_path = root_dir / cam;
        txt_path.replace_extension(".txt");
        
        cv::Mat cameraMatrix, distCoeffs, undistortMatrix;
        cv::Size imsize;
        readINTParameters(txt_path, cameraMatrix, distCoeffs);

        cout << "CameraMatrix: " << endl << cameraMatrix << endl;
        cout << "distCoeffs: " << endl << distCoeffs << endl;

        for (auto& folder : folders) {
            fs::path img_folder = folder / cam;
            fs::path undistorted_cam = folder / cam;
            undistorted_cam += "_undistorted";

            cout << img_folder.string() << endl;
            
            string path = img_folder / "*.jpg";
            vector<cv::String> img_files;
            cv::glob(path, img_files);

            fs::create_directories(undistorted_cam);

            for (auto& img_file : img_files) {
                fs::path img_path(img_file);
                fs::path img_out = undistorted_cam / img_path.filename();
                cout << img_out << endl;

                cv::Mat image = cv::imread(img_file);
                cv::Mat undistorted;
                cv::undistort(image, undistorted, cameraMatrix, distCoeffs);
                imsize = image.size();

                cv::imwrite(img_out.string(), undistorted);

                // cv::imshow("Image", image);
                // cv::imshow("Undistorted", undistorted);
                // cv::waitKey();
            }
            
        }

        undistortMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imsize, 0);
        cout << "Undistorted Matrix:" << endl << undistortMatrix << endl;
    }
    cv::destroyAllWindows();
    return 1;
}