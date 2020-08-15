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
    cv::Mat imgHsv;
    cv::Mat imgThresh;
    std::vector<cv::Vec3f> v3fCircles;

    uint8_t lowH, highH, lowS, highS, lowV, highV;

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

        cv::cvtColor(imgOriginal, imgHsv, CV_BGR2HSV);
        cv::GaussianBlur(imgOriginal, imgThresh, cv::Size(9, 9), 0);
        cv::dilate(imgThresh, imgThresh, 0);
        cv::erode(imgThresh, imgThresh, 0);

        std::cout<<"writing image\n";
        cv::imwrite("/home/pi/original.jpg", imgOriginal);
        cv::imwrite("/home/pi/thresh.jpg", imgThresh);

        int width = imgHsv.cols;
        int height = imgHsv.rows;
        int radius = std::min(width, height) / 3;

        cv::Mat mask = cv::Mat::zeros(imgHsv.size(), CV_8UC1);
        cv::circle(mask, cv::Point(width / 2, height / 2), radius, 1);

        cv::Scalar mean = cv::mean(imgHsv, mask);

        uint8_t h = mean[0];
        uint8_t s = mean[1];
        uint8_t v = mean[2];

        std::cout<<"h: "<<unsigned(h)<<", s: "<<unsigned(s)<<", v: "<<unsigned(v)<<"\n";

        lowH = std::clamp(h - 20, 0, 160);
        highH = std::clamp(h + 20, 0, 160);
        lowS = std::clamp(s - 60, 0, 255);
        highS = std::clamp(s + 30, 0, 255);
        lowV = std::clamp(v - 100, 0, 255);
        highV = std::clamp(v + 40, 0, 255);

        std::cout<<"Press y enter to continue or any other character to retry:"<<std::flush;
        std::cin>>in;
    }
    while (in != 'y');

    (void) pthread_create(&threadId, 0, userInput, 0);
    
    std::cout<<"Press q enter to exit\n";

    std::cout<<"Capturing frames\n";

    uint64_t t1, t2, t2_1, t3;
    while (running) {
	t1 = mstime();

        camera.grab();
        camera.retrieve(imgOriginal);
        
        t2 = mstime();

        cv::cvtColor(imgOriginal, imgHsv, CV_BGR2HSV);
        t2_1 = mstime();
        cv::inRange(imgHsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), imgThresh);

        cv::GaussianBlur(imgThresh, imgThresh, cv::Size(9, 3), 0);

        cv::HoughCircles(imgThresh, v3fCircles, CV_HOUGH_GRADIENT, 4, 1000, 100, 100, 50, 250);
        
        if (v3fCircles.size() == 0) {
                std::cout<<"nothing detected\n";
        }else {
            for (int i = 0; i < v3fCircles.size(); i++) {
                std::cout<<"x: "<<v3fCircles[i][0]<<", y: "<<v3fCircles[i][1]<<", r: "<<v3fCircles[i][2]<<"\n";
            }
        }

        t3 = mstime();
        uint64_t diff12 = t2 - t1;
        uint64_t diffconversion = t2_1 - t2;
        uint64_t diff23 = t3 - t2;
        std::cout<<"Capturetime: "<<diff12<<"ms, Computationtime: "<<diff23<<"ms\n"; 
        std::cout<<"Conversiontime: "<<diffconversion<<"ms\n"; 
    }

    std::cout<<"Stopping camera\n";
    camera.release();

    (void) pthread_join(threadId, NULL);
    std::cout<<"Done\n";

    return 0;
}

