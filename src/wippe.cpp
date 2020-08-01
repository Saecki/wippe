#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <raspicam/raspicam.h>
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
    std::cout<<"Starting\n";
    pthread_t threadId;
    (void) pthread_create(&threadId, 0, userInput, 0);

    std::cout<<"Press q Enter to exit\n";

    uint64_t start, end;
    raspicam::RaspiCam_Cv camera;
    cv::Mat image;

    camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);

    std::cout<<"Opening camera\n";
    if (!camera.open()) {
	std::cerr<<"error opening camera\n";
	return 1;
    }

    std::cout<<"Capturing frames\n";

    int i = 0;
    while (running) {
	if (i % 5 == 0) start = mstime();
        camera.grab();
        camera.retrieve(image);
        
	if (i % 5 == 0) {
	    end = mstime();
	    uint64_t msdiff = end - start;
	    std::cout<<"\rFrametime: "<<msdiff<<"ms  "<<std::flush; 
	    i = 0;
	}
	++i;
    }

    std::cout<<"\nStopping camera\n";
    camera.release();

    (void) pthread_join(threadId, NULL);
    std::cout<<"Done\n";

    return 0;
}

