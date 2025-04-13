#include <opencv2/core/core.hpp>

#include "System.h"

#include <thread>
#include <string>
#include <atomic>

std::atomic<bool> stopSLAM(false);

void listenForStopCommand() {
    std::string input;
    while (!stopSLAM) {
        std::cin >> input;
        if (input == "stop") {
            stopSLAM = true;
            std::cout << "Stopping ORB-SLAM3..." << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
    if (argc < 3)
        std::cout << "USAGE: px4_video_mono PATH_TO_VOCABULARY PATH_TO_CONFIG" << std::endl;

    const std::string PATH_TO_VOCAB = argv[1];
    const std::string PATH_TO_CONFIG = argv[2];

    ORB_SLAM3::System SLAM(PATH_TO_VOCAB, PATH_TO_CONFIG, ORB_SLAM3::System::MONOCULAR, true);

    std::string pipeline = 
    "udpsrc port=5601 ! "
    "application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! "
    "appsink";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video stream." << std::endl;
        return -1;
    }

    std::thread stopListener(listenForStopCommand);

    while (!stopSLAM) {
        cv::Mat img;

        if (!cap.read(img)) {
            std::cerr << "Error: Could not read frame." << std::endl;
            break;
        }

        double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();

        SLAM.TrackMonocular(img, timestamp);
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_PX4.txt");
    std::cout << "Keyframe trajectory saved to KeyFrameTrajectory_PX4.txt" << std::endl;

    cap.release();

    stopListener.join();

    return 0;
}
