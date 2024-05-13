#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

struct MouseCallbackParams {
    cv::Mat image;
    vector<cv::Point2d> corners;
    string name = "Image";

    MouseCallbackParams() {}
    MouseCallbackParams(cv::Mat image) : image(image.clone()) {}
    MouseCallbackParams(cv::Mat image, string name) : image(image.clone()), name(name) {}
};

void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        MouseCallbackParams* params = (MouseCallbackParams *)userdata;

        cout << "Coner: (" << x << "," << y << ") readed." << endl;

        params->corners.push_back(cv::Point2d(x, y));
        cv::circle(params->image, cv::Point2d(x, y), 3, cv::Scalar(0, 0, 255), 2);
        cv::imshow(params->name, params->image);
    }
}

void markCorner(string image_file, string corner_txt)
{
    cv::Mat image = cv::imread(image_file);
    if (image.empty()) {
        cerr << "Fail to read " << image_file << endl;
        return;
    }

    cv::namedWindow("Image", cv::WINDOW_NORMAL);
    MouseCallbackParams params(image, "Image");
    cv::setMouseCallback("Image", onMouse, &params);

    cv::imshow("Image", image);
    
    while (true) {
        int key = cv::waitKey(0);

        if (key == 13)  // 按回车确认选点并输出到 outfile 里
        {
            ofstream outFile(corner_txt, std::ios::app);
            if (!outFile.is_open()) {
                cerr << "Fail to open " << corner_txt << endl;
                return ;
            }

            outFile << image_file << endl;

            int idx = 1;
            for (const auto& corner : params.corners) {
                outFile << "point " << idx ++ << endl;
                outFile << corner.x << " " << corner.y << endl;
                cout << "Corners " << corner.x << " " << corner.y << ";";
            }
            cout << "Corners >> " << corner_txt << endl;

            outFile.close();
            
            break;
        }
        else if (key == 226 || key == 225) // 按 Shift 重新选点
        {
            cout << "Reselect corners." << endl;
            params.image = image.clone();
            params.corners.clear();
            cv::imshow("Image", params.image);
        }
        else if (key == 27) // ESC 跳过这张图
        {
            cout << "Skip this image." << endl;
            break;
        }   
    }
    cv::destroyAllWindows();
}

int main()
{
    vector<fs::path> folders = {
        "/home/fyh/data/GPcar/calib/2024-04-22-17-02-02",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-02-17",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-02-31",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-02-43",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-04-59",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-05-18",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-05-48",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-06-01",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-06-19",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-09-14",
    };

    vector<fs::path> cams = {
        "cam_front_30",
        "cam_side_right",
    };

    fs::path out_txt = "/home/fyh/data/GPcar/calib/front_30_corners.txt";

    cout << "点击顺序尽量为，左上、左下、右上、右下" << endl;

    for (const auto& folder : folders) {
        fs::path img_path = folder / "cam_front_30_undistorted" / "000000.jpg";
        markCorner(img_path.string(), out_txt.string());
    }

    return 0;
}