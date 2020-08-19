#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <raspicam/raspicam_cv.h>
#include <wiringSerial.h>

#define RGB_UPPER_MARGIN 30
#define RGB_LOWER_MARGIN 30

enum class State {
    Detecting,
    Tracking
};

enum class Signal {
    None,
    Restart
};

static volatile bool running = true;
static volatile bool printColor = false;
static volatile Signal signal = Signal::None;
static uint8_t lowB, highB, lowG, highG, lowR, highR;

static void* userInput(void*) {
    char in;
    while (running) {
        std::cin>>in;
	if (in == 'q') {
            running = false; 
        } else if (in == 'r') {
            signal = Signal::Restart;
            while (signal == Signal::Restart) continue;
        } else if (in == 'c') {
            printColor = !printColor;
        }
    } 
}

uint64_t mstime() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
	std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

void captureColors(raspicam::RaspiCam_Cv& camera) {
    cv::Mat imgOriginal;
    cv::Mat imgThresh;
    char in;
    do {
        camera.grab();
        camera.retrieve(imgOriginal);

        int width = imgOriginal.cols;
        int height = imgOriginal.rows;
        int radius = std::min(width, height) / 40;

        cv::Mat mask = cv::Mat::zeros(imgOriginal.size(), CV_8UC1);
        cv::circle(mask, cv::Point(width / 2, height / 2), radius, 1);

        cv::Scalar mean = cv::mean(imgOriginal, mask);

        uint8_t b = mean[0];
        uint8_t g = mean[1];
        uint8_t r = mean[2];

        printf("b: %u, g: %u, r: %u\n", b, g, r);

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
}

int main(int argc, char **argv) {
    pthread_t threadId;
    raspicam::RaspiCam_Cv camera;

    printf("Starting\n");
    
    printf("Opening serial port\n");
    int serialFD = serialOpen("/dev/ttyUSB0", 9600);
    if (serialFD == -1) {
        std::cerr<<"Error opening serial port\n";
        return 1;
    }
    serialPrintf(serialFD, "0.5,0.5\n");

    camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    printf("Opening camera\n");
    if (!camera.open()) {
	std::cerr<<"Error opening camera\n";
	return 1;
    }

    std::cout<<"Press enter to capture a closeup piture of the obejct you want to track"<<std::flush;
    while (std::cin.get() != '\n');
    captureColors(camera);

    (void) pthread_create(&threadId, 0, userInput, 0);
    
    printf("Press q enter to exit\n");
    printf("Capturing frames\n");

    cv::Mat imgOriginal;
    cv::Mat imgThresh;
    std::vector<cv::Vec3f> circles;
    cv::Ptr<cv::Tracker> tracker;
    cv::Rect2d box;
    State state = State::Detecting;
    float xpos = 0.5;
    float ypos = 0.5;
    float radius = 0;
    uint64_t t1 = 0;
    uint64_t t2 = 0;
    uint64_t t3 = 0;
    while (running) {
        if (signal == Signal::Restart) {
            captureColors(camera);
            signal = Signal::None;
            state = State::Detecting;
        }

        if (state == State::Detecting) {
            printf("DETECTING ");
        } else if (state == State::Tracking) {
            printf("TRACKING ");
        }


	t1 = mstime();

        camera.grab();
        camera.retrieve(imgOriginal);
        t2 = mstime();

        if (state == State::Detecting) {
            cv::inRange(imgOriginal, cv::Scalar(lowB, lowG, lowR), cv::Scalar(highB, highG, highR), imgThresh);
            cv::GaussianBlur(imgThresh, imgThresh, cv::Size(3, 3), 0);
            cv::HoughCircles(imgThresh, circles, CV_HOUGH_GRADIENT, 2, 1000, 100, 100, imgOriginal.cols / 25, imgOriginal.cols / 5);

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
                float xraw = (float) circles[closest][0];
                float yraw = (float) circles[closest][1];
                
                xpos = xraw / (float) imgThresh.cols;
                ypos = yraw / (float) imgThresh.rows;
                radius = (float) circles[closest][2];
                ypos = ypos * factor + (1 - factor) / 2;

                tracker = cv::TrackerKCF::create();
                box = cv::Rect2d(xraw - radius, yraw - radius, radius * 2, radius * 2);
                bool ok = tracker->init(imgOriginal, box);
                
                if (ok) state = State::Tracking;
            }
        } else if (state == State::Tracking) {
            bool ok = tracker->update(imgOriginal, box);

            if (ok) {
                float factor = (float) imgThresh.rows / (float) imgThresh.cols;
                xpos = (box.x + box.width / 2) / (float) imgThresh.cols;
                ypos = (box.y + box.height / 2) / (float) imgThresh.rows;
                ypos = ypos * factor + (1 - factor) / 2;
            } else {
                state = State::Detecting;
                printf("Error tracking");
            }
        }
        t3 = mstime();
        
        serialPrintf(serialFD, "%f,%f\n", xpos, ypos);
        
        if (printColor) {
            int width = imgOriginal.cols;
            int height = imgOriginal.rows;
            int radius = std::min(width, height) / 40;
            cv::Mat mask = cv::Mat::zeros(imgOriginal.size(), CV_8UC1);
            cv::circle(mask, cv::Point(width / 2, height / 2), radius, 1);
            cv::Scalar mean = cv::mean(imgOriginal, mask);
            uint8_t b = mean[0];
            uint8_t g = mean[1];
            uint8_t r = mean[2];
            printf("b: %u, g: %u, r: %u\n", b, g, r);
        } else {
            uint64_t capture = t2 - t1;
            uint64_t calc = t3 - t2;
            printf("x: %f,y: %f", xpos, ypos);
            std::cout<<", capture: "<<capture<<"ms";
            std::cout<<", calc: "<<calc<<"ms\n";
        }
    }

    printf("Stopping camera\n");
    camera.release();

    printf("Closing serial port\n");
    serialPrintf(serialFD, "0.5,0.5\n");
    serialPrintf(serialFD, "0.5,0.5\n");
    serialClose(serialFD);
    
    (void) pthread_join(threadId, NULL);

    printf("Done\n");

    return 0;
}
