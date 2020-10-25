#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <raspicam/raspicam_cv.h>
#include <wiringSerial.h>

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

enum class State {
    Detecting,
    Tracking
};

struct Position {
    float x;
    float y;
};

class PositionTracker {
    public:
        State state = State::Detecting;

    private:
        raspicam::RaspiCam_Cv camera;
        
        cv::Mat imgOriginal;
        cv::Mat imgThresh;
        
        cv::Ptr<cv::Tracker> tracker;
        cv::Rect2d box;

    public:
        PositionTracker() {
            camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
            camera.set(cv::CAP_PROP_FRAME_WIDTH, 320);
            camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

            if (!camera.open()) {
                throw std::runtime_error("Error opening camera");
            }
        }

        ~PositionTracker() {
            camera.release();
        }


        void capture() {
            camera.grab();
            camera.retrieve(imgOriginal);
        }

        std::optional<Position> track() {
            Position pos;

            if (state == State::Detecting) {
                std::vector<cv::Vec3f> circles;

                cv::cvtColor(imgOriginal, imgThresh, cv::COLOR_BGR2GRAY);
                cv::GaussianBlur(imgThresh, imgThresh, cv::Size(3, 3), 0);
                //cv::dilate(imgThresh, imgThresh, 5);
                //cv::erode(imgThresh, imgThresh, 5);
                cv::HoughCircles(
                    imgThresh,
                    circles,
                    CV_HOUGH_GRADIENT,      // method
                    2,                      // inverse ratio of accumulator resolution
                    imgOriginal.cols * 2,   // minDist
                    100,                    // param1 depends on method
                    100,                    // param2 depends on method
                    imgOriginal.cols / 25,  // minRadius
                    imgOriginal.cols / 5    // maxRadius
                );

                if (circles.size() != 0) {
                    int closestDiff = std::abs(circles[0][0] - pos.x) + std::abs(circles[0][1] - pos.y);
                    int closest = 0;
                    for (int i = 1; i < circles.size(); i++) {
                        int diff = std::abs(circles[i][0] - pos.x) + std::abs(circles[i][1] - pos.y);
                        if (diff < closestDiff) {
                            closestDiff = diff;
                            closest = i;
                        }
                    }

                    float factor = (float) imgThresh.rows / (float) imgThresh.cols;
                    float xraw = (float) circles[closest][0];
                    float yraw = (float) circles[closest][1];
                    
                    pos.x = xraw / (float) imgThresh.cols;
                    pos.y = yraw / (float) imgThresh.rows * (factor + (1 - factor) / 2);
                    float radius = (float) circles[closest][2];

                    tracker = cv::TrackerKCF::create();
                    box = cv::Rect2d(xraw - radius, yraw - radius, radius * 2, radius * 2);
                    bool ok = tracker->init(imgOriginal, box);
                    
                    if (ok) {
                        state = State::Tracking;
                    } else {
                        return {};
                    }
                } else {
                    return {};
                }
            } else if (state == State::Tracking) {
                bool ok = tracker->update(imgOriginal, box);

                if (ok) {
                    float factor = (float) imgThresh.rows / (float) imgThresh.cols;
                    pos.x = (box.x + box.width / 2) / (float) imgThresh.cols;
                    pos.y = (box.y + box.height / 2) / (float) imgThresh.rows * (factor + (1 - factor) / 2);
                } else {
                    state = State::Detecting;
                }
            }

            return pos;
        }
};

class PIDController {
    float PF = 0.7;
    float IF = 0;
    float DF = 180;
    
    uint64_t lastUpdateTime = 0;
    float lastDiffX = 0;
    float lastDiffY = 0;
    float ix = 0;
    float iy = 0;

    public:
        Position calc(float x, float y) {
            uint64_t currentTime = mstime();
            
            float diffX = 0.5 - x;
            float diffY = 0.5 - y;
            float timeDiff = currentTime - lastUpdateTime;
            
            float px = diffX * PF;
            float py = diffY * PF;

            float dx = (diffX - lastDiffX) * DF / timeDiff;
            float dy = (diffY - lastDiffY) * DF / timeDiff;
            
            ix += diffX * IF / timeDiff;
            iy += diffY * IF / timeDiff;

            float calcX = std::clamp(0.5f + px + ix + dx, 0.0f, 1.0f);
            float calcY = std::clamp(0.5f + py + iy + dy, 0.0f, 1.0f);

            lastDiffX = diffX;
            lastDiffY = diffY;
            lastUpdateTime = currentTime;

            return { calcX, calcY };
        }
};

int main(int argc, char **argv) {
    pthread_t threadId;
    uint64_t t1 = 0;
    uint64_t t2 = 0;
    uint64_t t3 = 0;

    printf("Starting\n");
    
    PositionTracker tracker;
    PIDController controller;
    
    printf("Opening serial port\n");
    int serialFD = serialOpen("/dev/ttyUSB0", 9600);
    if (serialFD == -1) {
        std::cerr<<"Error opening serial port\n";
        return 1;
    }
    serialPrintf(serialFD, "0.5,0.5\n");

    (void) pthread_create(&threadId, 0, userInput, 0);
    
    printf("Press q enter to exit\n");

    while (running) {
	t1 = mstime();
        
        tracker.capture();
        
        t2 = mstime();
       
        Position bpos;
        if (auto pos = tracker.track(); pos) {
            bpos = pos.value();
        } else {
            serialPrintf(serialFD, "0.5,0.5\n");
            printf("Nothing detected\n");
            continue;
        }
        if (tracker.state == State::Detecting) {
            printf("DETECTING ");
        } else if (tracker.state == State::Tracking) {
            printf("TRACKING ");
        }
        
        t3 = mstime();

        Position cpos = controller.calc(bpos.x, bpos.y);
        
        serialPrintf(serialFD,"%f,%f\n", cpos.x, cpos.y);
        
        uint64_t capture = t2 - t1;
        uint64_t calc = t3 - t2;
        printf("x: %f,y: %f, cx: %f, cy: %f", bpos.x, bpos.y, cpos.x, cpos.y);
        std::cout<<", capture: "<<capture<<"ms";
        std::cout<<", calc: "<<calc<<"ms\n";
    }

    printf("Closing serial port\n");
    serialPrintf(serialFD, "0.5,0.5\n");
    serialClose(serialFD);
    
    (void) pthread_join(threadId, NULL);

    printf("Done\n");

    return 0;
}
