#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <raspicam/raspicam_cv.h>
#include <wiringSerial.h>

#define RGB_UPPER_MARGIN 40
#define RGB_LOWER_MARGIN 40

#define DETECTING 0
#define TRACKING 1

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

    uint8_t lowB, highB, lowG, highG, lowR, highR;

    std::cout<<"Starting\n";
    
    std::cout<<"Opening serial port\n";
    int serialFD = serialOpen("/dev/ttyUSB0", 9600);
    if (serialFD == -1) {
        std::cerr<<"Error opening serial port\n";
        return 1;
    }

    camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    std::cout<<"Opening camera\n";
    if (!camera.open()) {
	std::cerr<<"Error opening camera\n";
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

        lowB = std::clamp(b - RGB_LOWER_MARGIN, 0, 255);
        highB = std::clamp(b + RGB_UPPER_MARGIN, 0, 255);
        lowG = std::clamp(g - RGB_LOWER_MARGIN, 0, 255);
        highG = std::clamp(g + RGB_UPPER_MARGIN, 0, 255);
        lowR = std::clamp(r - RGB_LOWER_MARGIN, 0, 255);
        highR = std::clamp(r + RGB_UPPER_MARGIN, 0, 255);

        std::cout<<"Press y enter to continue or any other character to retry:"<<std::flush;
        std::cin>>in;
    }
    while (in != 'y');

    (void) pthread_create(&threadId, 0, userInput, 0);
    
    std::cout<<"Press q enter to exit\n";
    std::cout<<"Capturing frames\n";

    uint64_t t1, t2, t3, t4, t5, t6;

    std::vector<cv::Vec3f> circles;
    cv::Ptr<cv::Tracker> tracker = cv::Tracker::create("KCF");
    cv::Rect2d box;
    int state = DETECTING;
    float xpos, ypos, radius;
    while (running) {
	t1 = mstime();

        camera.grab();
        camera.retrieve(imgOriginal);
        t2 = mstime();

        if (state == DETECTING) {
            cv::inRange(imgOriginal, cv::Scalar(lowB, lowG, lowR), cv::Scalar(highB, highG, highR), imgThresh);
            t3 = mstime();
            
            cv::GaussianBlur(imgThresh, imgThresh, cv::Size(3, 3), 0);
            t4 = mstime();

            cv::HoughCircles(imgThresh, circles, CV_HOUGH_GRADIENT, 6, 1000, 100, 100, 50, 250);

            if (circles.size() != 0) {
                int closestDiff = std::abs(circles[0][0] - xpos) + std::abs(circles[0][1] - ypos);
                int closest = 0;
                for (int i = 1; i < circles.size(); i++) {
                    int diff = std::abs(circles[i][0] - xpos) + std::abs(circles[i][1] - ypos);
                    if (diff < closestDiff) {
                        closestDiff = diff;
                        closest = i;
                    }
                }

                float factor = (float) imgThresh.rows / (float) imgThresh.cols;
                xpos = (float) circles[closest][0] / (float) imgThresh.cols;
                ypos = (float) circles[closest][1] / (float) imgThresh.rows;
                radius = (float) circles[closest][2];
                ypos = ypos * factor + (1 - factor) / 2;

                box = cv::Rect2d(xpos - radius, ypos - radius, radius, radius);
                tracker->init(imgOriginal, box);
                
                state = TRACKING;
            }
        } else if (state == TRACKING) {
            bool tracked = tracker->update(imgOriginal, box);

            if (tracked) {
                xpos = box.x + box.width / 2;
                ypos = box.y + box.height / 2;
            } else {
                state = DETECTING;
            }
        }
        t5 = mstime();
        
        if (state == TRACKING) {
            serialPrintf(serialFD, "%f,%f;\n", xpos, ypos);
            printf("%f,%f;\n", xpos, ypos);
        } else {
            serialPrintf(serialFD, "0.5,0.5;\n");
            std::cout<<"Nothing detected\n";
        }
        t6 = mstime();

        uint64_t capture = t2 - t1;
        uint64_t range = t3 - t2;
        uint64_t blur = t4 - t3;
        uint64_t output = t6 - t5;

        if (state == DETECTING) {
            uint64_t circles = t5 - t4;
            std::cout<<"Capture: "<<capture<<"ms, InRange: "<<range<<"ms, Blur: "<<blur<<"ms, Circles: "<<circles<<"ms, Output: "<<output<<"ms\n"; 
        } else if (state == TRACKING) {
            uint64_t tracking = t5 - t4;
            std::cout<<"Capture: "<<capture<<"ms, InRange: "<<range<<"ms, Blur: "<<blur<<"ms, Tracking: "<<tracking<<"ms, Output: "<<output<<"ms\n"; 
        }
    }

    std::cout<<"Stopping camera\n";
    camera.release();

    std::cout<<"Closing serial port\n";
    serialPrintf(serialFD, "0.5,0.5;\n");
    serialClose(serialFD);

    (void) pthread_join(threadId, NULL);
    std::cout<<"Done\n";

    return 0;
}
