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

static volatile bool running = true;
static uint8_t lowB, highB, lowG, highG, lowR, highR;

static void* userInput(void*) {
    char in;
    while (running) {
        std::cin>>in;
	if (in == 'q') {
            running = false; 
        } 
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

    std::cout<<"Press enter to continue"<<std::flush;
    while (std::cin.get() != '\n');

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
            cv::cvtColor(imgOriginal, imgThresh, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(imgThresh, imgThresh, cv::Size(3, 3), 0);
            cv::HoughCircles(
                imgThresh,
                circles,
                CV_HOUGH_GRADIENT,
                2,
                imgOriginal.cols * 2,
                100,
                100,
                imgOriginal.cols / 25,
                imgOriginal.cols / 5
            );

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
        
        uint64_t capture = t2 - t1;
        uint64_t calc = t3 - t2;
        printf("x: %f,y: %f", xpos, ypos);
        std::cout<<", capture: "<<capture<<"ms";
        std::cout<<", calc: "<<calc<<"ms\n";
    }

    printf("Stopping camera\n");
    camera.release();

    printf("Closing serial port\n");
    serialPrintf(serialFD, "0.5,0.5\n");
    serialPrintf(serialFD, "0.5,0.5\n");
    serialPrintf(serialFD, "0.5,0.5\n");
    serialClose(serialFD);
    
    (void) pthread_join(threadId, NULL);

    printf("Done\n");

    return 0;
}
