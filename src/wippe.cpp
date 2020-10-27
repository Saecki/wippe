#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <raspicam/raspicam_cv.h>
#include <vector>
#include <wiringSerial.h>
#include <ncurses.h>

static volatile bool running = true;

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

struct Point {
    double x;
    double y;
};

class PositionTracker {
    public:
        State state = State::Detecting;

    private:
        raspicam::RaspiCam_Cv camera;
        
        cv::Mat img_orig;
        cv::Mat img_thresh;
        
        cv::Ptr<cv::Tracker> tracker;
        cv::Rect2d roi;

    public:
        PositionTracker() {
            camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);
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
            camera.retrieve(img_orig);
        }

        std::optional<Point> track() {
            Point pos;

            if (state == State::Detecting) {
                std::vector<cv::Vec3f> circles;

                cv::cvtColor(img_orig, img_thresh, cv::COLOR_BGR2GRAY);
                //cv::GaussianBlur(img_thresh, img_thresh, cv::Size(3, 3), 0);
                cv::dilate(img_thresh, img_thresh, 3);
                cv::erode(img_thresh, img_thresh, 3);
                cv::HoughCircles(
                    img_thresh,
                    circles,
                    CV_HOUGH_GRADIENT,      // method
                    2,                      // inverse ratio of accumulator resolution
                    img_orig.cols * 2,   // minDist
                    100,                    // param1: behavior depends on method
                    100,                    // param2: behavior depends on method
                    img_orig.cols / 25,  // minRadius
                    img_orig.cols / 5    // maxRadius
                );

                if (circles.size() != 0) {
                    uint32_t closestDiff = std::abs(circles[0][0] - pos.x) + std::abs(circles[0][1] - pos.y);
                    uint32_t closest = 0;
                    for (auto i = 1; i < circles.size(); ++i) {
                        auto diff = std::abs(circles[i][0] - pos.x) + std::abs(circles[i][1] - pos.y);
                        if (diff < closestDiff) {
                            closestDiff = diff;
                            closest = i;
                        }
                    }

                    double factor = (double) img_thresh.rows / (double) img_thresh.cols;
                    double xraw = (double) circles[closest][0];
                    double yraw = (double) circles[closest][1];
                    
                    pos.x = xraw / (double) img_thresh.cols;
                    pos.y = yraw / (double) img_thresh.rows * (factor + (1 - factor) / 2);
                    double radius = (double) circles[closest][2];

                    tracker = cv::TrackerKCF::create();
                    roi = cv::Rect2d(xraw - radius, yraw - radius, radius * 2, radius * 2);
                    bool ok = tracker->init(img_orig, roi);
                    
                    if (ok) 
                        state = State::Tracking;
                    else 
                        return {};

                } else {
                    return {};
                }
            } else if (state == State::Tracking) {
                bool ok = tracker->update(img_orig, roi);

                if (ok) {
                    double factor = (double) img_thresh.rows / (double) img_thresh.cols;
                    pos.x = (roi.x + (double) roi.width / 2.0) / (double) img_thresh.cols;
                    pos.y = ((roi.y + (double) roi.height / 2.0) / (double) img_thresh.rows) * (factor + (1 - factor) / 2);
                } else {
                    state = State::Detecting;
                }
            }

            return pos;
        }
};

class PDController {
#define MAX_HIST_SIZE 10u
    public:
        double PF = 0.4;
        double DF = 220;

    private:
        uint64_t last_update_time = 0;
        Point last_diff;
        
        uint32_t hist_size;
        std::array<Point, MAX_HIST_SIZE> d_hist;
    
    public:
        PDController(uint32_t hist_size) {
            set_hist_size(hist_size);
        }

    private:
        double flatten(double a) {
            const double limit = 0.01;
            const double factor = 100;

            if (a == 0.0) return a;
            if (a > limit) return a;
            
            double v = std::abs(a);
            double f = std::pow(v * factor, 1.8) / factor;

            if (f == 0.0) return f;

            return std::clamp(a * (f / v), -0.5, 0.5);
        }

    public:
        void reinit(double last_x, double last_y) {
            last_diff = { 0.5 - last_x, 0.5 - last_y };
        }

        void set_hist_size(uint32_t hist_size) {
            this->hist_size = std::clamp(hist_size, 0u, MAX_HIST_SIZE);
        }

        Point calc(double x, double y) {
            uint64_t current_time = mstime();
            
            Point diff = { 0.5 - x, 0.5 - y };
            double time_diff = current_time - last_update_time;
            
            double px = diff.x * PF;
            double py = diff.y * PF;
            
            double ndx = flatten(diff.x - last_diff.x) * DF / time_diff;
            double ndy = flatten(diff.y - last_diff.y) * DF / time_diff;
            
            for (auto i = 0; i < hist_size - 1; ++i) {
                d_hist[i] = d_hist[i + 1];
            }
            d_hist[hist_size - 1] = { ndx, ndy };
            
            double dx_hist_sum = 0.0;
            double dy_hist_sum = 0.0;
            for (auto i = 0; i < hist_size; ++i) {
                dx_hist_sum += d_hist[i].x;
                dy_hist_sum += d_hist[i].y;
            }

            double dx = dx_hist_sum / static_cast<double>(hist_size); 
            double dy = dy_hist_sum / static_cast<double>(hist_size);

            double calc_x = std::clamp(0.5 + px + dx, 0.0, 1.0);
            double calc_y = std::clamp(0.5 + py + dy, 0.0, 1.0);

            last_diff = diff;
            last_update_time = current_time;

            return { calc_x, calc_y };
        }
};

int main(int argc, char **argv) {
    pthread_t threadId;

    printf("Starting\n");
    
    PositionTracker tracker;
    PDController controller(2);
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
