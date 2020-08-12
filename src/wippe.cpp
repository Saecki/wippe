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

    uint64_t t1, t2, t3;
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
    do {
        std::cout<<"Press enter to capture a closeup piture of the obejct you want to track"<<std::flush;
        while (std::cin.get() != '\n');

        std::cout<<"Capturing samle data\n";
        camera.grab();
        camera.retrieve(imgOriginal);

        cv::cvtColor(imgOriginal, imgHsv, CV_RGB2HSV);

        cv::GaussianBlur(imgHsv, imgThresh, cv::Size(3, 3), 0);
        cv::dilate(imgThresh, imgThresh, 0);
        cv::erode(imgThresh, imgThresh, 0);

        int px = imgThresh.cols / 2;
        int py = imgThresh.rows / 2;
        int pos = (py * imgThresh.cols + px) * imgThresh.step;

        uint8_t h = imgThresh.data[py * imgThresh.step + imgThresh.channels() * px + 0];
        uint8_t s = imgThresh.data[py * imgThresh.step + imgThresh.channels() * px + 1];
        uint8_t v = imgThresh.data[py * imgThresh.step + imgThresh.channels() * px + 2];

        std::cout<<"h: "<<unsigned(h)<<", s: "<<unsigned(s)<<", v: "<<unsigned(v)<<"\n";

        lowH = std::clamp(h - 20, 0, 255);
        highH = std::clamp(h + 20, 0, 255);
        lowS = std::clamp(s - 20, 0, 255);
        highS = std::clamp(s + 20, 0, 255);
        lowV = std::clamp(v - 20, 0, 255);
        highV = std::clamp(v + 20, 0, 255);

        std::cout<<"Press y enter to continue or any other character to retry"<<std::flush;
        std::cin>>in;
    }
    while (in! = 'y');

    (void) pthread_create(&threadId, 0, userInput, 0);
    
    std::cout<<"Press q enter to exit\n";

    std::cout<<"Capturing frames\n";

    uint8_t i = 0;
    while (running) {
	t1 = mstime();

        camera.grab();
        camera.retrieve(imgOriginal);
        
        t2 = mstime();

        cv::cvtColor(imgOriginal, imgHsv, CV_RGB2HSV);
        cv::inRange(imgHsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), imgThresh);

        cv::GaussianBlur(imgThresh, imgThresh, cv::Size(3, 3), 0);
        cv::dilate(imgThresh, imgThresh, 0);
        cv::erode(imgThresh, imgThresh, 0);

        //cv::HoughCircles(imgThresh, v3fCircles, CV_HOUGH_GRADIENT, 2, imgThresh.rows / 4, 100, 50, 10, 800);
        
        if (v3fCircles.size() == 0) {
                std::cout<<"nothing detected\n";
        }else {
            for (int i = 0; i < v3fCircles.size(); i++) {
                std::cout<<"x: "<<v3fCircles[i][0]<<", y: "<<v3fCircles[i][1]<<", r: "<<v3fCircles[i][2]<<"\n";
            }
        }

        t3 = mstime();
        uint64_t diff12 = t2 - t1;
        uint64_t diff23 = t3 - t2;
        std::cout<<"Capturetime: "<<diff12<<"ms, Computationtime: "<<diff23<<"ms\n"; 
        i = 0;
	++i;
    }

    std::cout<<"Stopping camera\n";
    camera.release();

    (void) pthread_join(threadId, NULL);
    std::cout<<"Done\n";

    return 0;
}

