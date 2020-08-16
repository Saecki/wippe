#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

static volatile bool running = true;

static void* userInput(void*) {
    while (running) {
	if (std::cin.get() == 'q') running = false;
    }
}

uint64_t mstime() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
	std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

int main(int argc, char **argv) {
    pthread_t threadId;

    raspicam::RaspiCam_Cv camera;
    cv::Mat imgOriginal;
    cv::Mat imgThresh;
    std::vector<cv::Vec3f> v3fCircles;

    uint8_t lowB, highB, lowG, highG, lowR, highR;

    std::cout<<"Starting\n";

    camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);

    std::cout<<"Opening camera\n";
    if (!camera.open()) {
	std::cerr<<"error opening camera\n";
	return 1;
    }

    char in;
    std::cout<<"Press enter to capture a closeup piture of the obejct you want to track"<<std::flush;
    do {
        while (std::cin.get() != '\n');

        std::cout<<"Capturing samle data\n";
        camera.grab();
        camera.retrieve(imgOriginal);

        int width = imgOriginal.cols;
        int height = imgOriginal.rows;
        int radius = std::min(width, height) / 3;

        cv::Mat mask = cv::Mat::zeros(imgOriginal.size(), CV_8UC1);
        cv::circle(mask, cv::Point(width / 2, height / 2), radius, 1);

        cv::Scalar mean = cv::mean(imgOriginal, mask);

        uint8_t b = mean[0];
        uint8_t g = mean[1];
        uint8_t r = mean[2];

        std::cout<<"b: "<<unsigned(b)<<", g: "<<unsigned(g)<<", r: "<<unsigned(r)<<"\n";

        lowB = std::clamp(b - 80, 0, 255);
        highB = std::clamp(b + 50, 0, 255);
        lowG = std::clamp(g - 80, 0, 255);
        highG = std::clamp(g + 50, 0, 255);
        lowR = std::clamp(r - 80, 0, 255);
        highR = std::clamp(r + 50, 0, 255);

        std::cout<<"Press y enter to continue or any other character to retry:"<<std::flush;
        std::cin>>in;
    }
    while (in != 'y');

    (void) pthread_create(&threadId, 0, userInput, 0);
    
    std::cout<<"Press q enter to exit\n";

    std::cout<<"Capturing frames\n";

    uint64_t t1, t2, t3, t4, t5;
    float xpos, ypos;
    while (running) {
	t1 = mstime();

        camera.grab();
        camera.retrieve(imgOriginal);
        t2 = mstime();

        cv::inRange(imgOriginal, cv::Scalar(lowB, lowG, lowR), cv::Scalar(highB, highG, highR), imgThresh);
        t3 = mstime();
        
        cv::GaussianBlur(imgThresh, imgThresh, cv::Size(3, 3), 0);
        
        t4 = mstime();
        cv::HoughCircles(imgThresh, v3fCircles, CV_HOUGH_GRADIENT, 6, 1000, 100, 100, 50, 250);
        
        if (v3fCircles.size() == 0) {
                std::cout<<"nothing detected\n";
        }else {
            float factor = (float) imgThresh.cols / (float) imgThresh.rows;
            xpos = (float) v3fCircles[0][0] / (float) imgThresh.cols;
            ypos = (float) v3fCircles[0][1] / (float) imgThresh.rows;
            ypos = ypos / factor;

            std::cout<<"xpos: "<<xpos<<", ypos: "<<ypos<<"\n";

            for (int i = 0; i < v3fCircles.size(); i++) {
                std::cout<<"x: "<<v3fCircles[i][0]<<", y: "<<v3fCircles[i][1]<<", r: "<<v3fCircles[i][2]<<"\n";
            }
        }

        t5 = mstime();
        uint64_t capture = t2 - t1;
        uint64_t range = t3 - t2;
        uint64_t blur = t4 - t3;
        uint64_t circles = t5 - t4;
        std::cout<<"Capture: "<<capture<<"ms, InRange: "<<range<<"ms, Blur: "<<blur<<"ms, Circles: "<<circles<<"ms\n"; 
    }

    std::cout<<"Stopping camera\n";
    camera.release();

    (void) pthread_join(threadId, NULL);
    std::cout<<"Done\n";

    return 0;
}

