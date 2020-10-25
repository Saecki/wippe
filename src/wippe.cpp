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

#define FACTOR 100.0
double flatten(double a) {
    if (a == 0.0) return a;
    if (a > 0.01) return a;
    
    double v = std::abs(a);
    double f = std::pow(v * FACTOR, 1.8) / FACTOR;

    if (f == 0.0) return f;

    return std::clamp(a * (f / v), -0.5, 0.5);
}

enum class State {
    Detecting,
    Tracking
};

struct Point {
    double x;
    double y;
};

class PositionTracker {
    public:
        State state = State::Detecting;

    private:
        raspicam::RaspiCam_Cv camera;
        
        cv::Mat imgOriginal;
        cv::Mat imgThresh;
        
        cv::Ptr<cv::Tracker> tracker;
        cv::Rect2d roi;

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

        std::optional<Point> track() {
            Point pos;

            if (state == State::Detecting) {
                std::vector<cv::Vec3f> circles;

                cv::cvtColor(imgOriginal, imgThresh, cv::COLOR_BGR2GRAY);
                cv::GaussianBlur(imgThresh, imgThresh, cv::Size(3, 3), 0);
                cv::dilate(imgThresh, imgThresh, 5);
                cv::erode(imgThresh, imgThresh, 5);
                cv::HoughCircles(
                    imgThresh,
                    circles,
                    CV_HOUGH_GRADIENT,      // method
                    2,                      // inverse ratio of accumulator resolution
                    imgOriginal.cols * 2,   // minDist
                    100,                    // param1: behavior depends on method
                    100,                    // param2: behavior depends on method
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

                    double factor = (double) imgThresh.rows / (double) imgThresh.cols;
                    double xraw = (double) circles[closest][0];
                    double yraw = (double) circles[closest][1];
                    
                    pos.x = xraw / (double) imgThresh.cols;
                    pos.y = yraw / (double) imgThresh.rows * (factor + (1 - factor) / 2);
                    double radius = (double) circles[closest][2];

                    tracker = cv::TrackerKCF::create();
                    roi = cv::Rect2d(xraw - radius, yraw - radius, radius * 2, radius * 2);
                    bool ok = tracker->init(imgOriginal, roi);
                    
                    if (ok) {
                        state = State::Tracking;
                    } else {
                        return {};
                    }
                } else {
                    return {};
                }
            } else if (state == State::Tracking) {
                bool ok = tracker->update(imgOriginal, roi);

                if (ok) {
                    double factor = (double) imgThresh.rows / (double) imgThresh.cols;
                    pos.x = (roi.x + (double) roi.width / 2.0) / (double) imgThresh.cols;
                    pos.y = ((roi.y + (double) roi.height / 2.0) / (double) imgThresh.rows) * (factor + (1 - factor) / 2);
                } else {
                    state = State::Detecting;
                }
            }

            return pos;
        }
};

class PIDController {
    #define HIST_SIZE 2
    double dxHist[HIST_SIZE];
    double dyHist[HIST_SIZE];

    double PF = 0.4;
    double IF = 0;
    double DF = 220;
    
    uint64_t lastUpdateTime = 0;
    Point lastDiff;
    Point i;

    public:
        void reinit(double lastX, double lastY) {
            lastDiff = { 0.5 - lastY, 0.5 - lastY };
        }

        Point calc(double x, double y) {
            uint64_t currentTime = mstime();
            
            Point diff = { 0.5 - x, 0.5 - y };
            double timeDiff = currentTime - lastUpdateTime;
            
            float px = diff.x * PF;
            float py = diff.y * PF;

            i.x += diff.x * IF / timeDiff;
            i.y += diff.y * IF / timeDiff;
            
            float ndx = flatten(diff.x - lastDiff.x) * DF / timeDiff;
            float ndy = flatten(diff.y - lastDiff.y) * DF / timeDiff;

            for (int i = 0; i < HIST_SIZE - 1; i++) {
                dxHist[i] = dxHist[i + 1];
                dyHist[i] = dyHist[i + 1];
            }
            dxHist[HIST_SIZE - 1] = ndx;
            dyHist[HIST_SIZE - 1] = ndy;
            
            double dxsum = 0.0;
            double dysum = 0.0;
            for (int i = 0; i < HIST_SIZE; i++) {
                dxsum += dxHist[i];
                dysum += dyHist[i];
            }
            double dx = dxsum / (double)HIST_SIZE; 
            double dy = dysum / (double)HIST_SIZE;

            double calcX = std::clamp(0.5 + px + i.x + dx, 0.0, 1.0);
            double calcY = std::clamp(0.5 + py + i.y + dy, 0.0, 1.0);

            lastDiff = diff;
            lastUpdateTime = currentTime;

            return { calcX, calcY };
        }
};

int main(int argc, char **argv) {
    pthread_t threadId;

    printf("Starting\n");
    
    PositionTracker tracker;
    PIDController controller;
    bool tracking = false;
    
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
	uint64_t t1 = mstime();
        
        tracker.capture();
        
        uint64_t t2 = mstime();
       
        Point bpos;
        if (auto pos = tracker.track(); pos) {
            bpos = pos.value();
            
            if (!tracking) {
                controller.reinit(bpos.x, bpos.y);
                tracking = true;
            }
        } else {
            serialPrintf(serialFD, "0.5,0.5\n");
            printf("Nothing detected\n");
            tracking = false;
            continue;
        }
        
        uint64_t t3 = mstime();

        Point cpos = controller.calc(bpos.x, bpos.y);

        uint64_t t4 = mstime();
        
        serialPrintf(serialFD,"%f,%f\n", cpos.x, cpos.y);
        
        uint64_t capture = t2 - t1;
        uint64_t track = t3 - t2;
        uint64_t calc = t4 - t3;
        
        if (tracker.state == State::Detecting) {
            printf("DETECTING ");
        } else if (tracker.state == State::Tracking) {
            printf("TRACKING ");
        }
        printf("bx: %f,by: %f, cx: %f, cy: %f", bpos.x, bpos.y, cpos.x, cpos.y);
        std::cout<<", capture: "<<capture<<"ms";
        std::cout<<", track: "<<track<<"ms";
        std::cout<<", calc: "<<calc<<"ms\n";
    }

    printf("Closing serial port\n");
    serialPrintf(serialFD, "0.5,0.5\n");
    serialClose(serialFD);
    
    (void) pthread_join(threadId, NULL);

    printf("Done\n");

    return 0;
}
