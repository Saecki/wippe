#include <chrono>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

uint64_t mstime() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
	std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

int main(int argc, char **argv) {
    std::cout<<"Starting\nPress q to exit\n";

    uint64_t start, end;
    raspicam::RaspiCam_Cv camera;
    cv::Mat imageOriginal;
    cv::Mat imageHsv;
    cv::Mat imageThresh;

    camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);

    std::cout<<"Opening camera\n";
    if (!camera.open()) {
	std::cerr<<"error opening camera\n";
	return 1;
    }

    std::cout<<"Capturing frames\n";

    uint8_t i = 0;
    char inputKey = 0;
    while (inputKey != 27) {
	if (i % 5 == 0) start = mstime();
        camera.grab();
        camera.retrieve(imageOriginal);
        
	if (i % 5 == 0) {
	    end = mstime();
	    uint64_t msdiff = end - start;
	    std::cout<<"\rCapturetime: "<<msdiff<<"ms  "<<std::flush; 
	    i = 0;
	}

        imageHsv = 

	++i;
        inputKey = cv::waitKey(1);
    }

    std::cout<<"\nStopping camera\n";
    camera.release();

    std::cout<<"Done\n";

    return 0;
}

