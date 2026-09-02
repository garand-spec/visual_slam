#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main() {
	cv::VideoCapture cap(0);
	if (!cap.isOpened()) return -1;

	cv::Mat oldFrame;
	cv::Mat oldGray;

	cap >> oldFrame;

	cv::cvtColor(oldFrame, oldGray, cv::COLOR_BGR2GRAY);
	
	std::vector<cv::Point2f> oldPoints;
	
	cv::goodFeaturesToTrack(
			oldGray,
			oldPoints,
			100,
			0.01,
			10
		);


	while (true) {

		cv::Mat frame;
		cv::Mat gray;
			
		cap >> frame;

		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		std::vector<cv::Point2f> newPoints;

		std::vector<uchar> status;

		std::vector<float> error;

		cv::calcOpticalFlowPyrLK(
				oldGray,
				gray,
				oldPoints,
				newPoints,
				status,
				error
			);

		if (!status.empty() && status[0]){
			
			for (int i = 0; i < newPoints.size(); i++) {
				cv::circle(
				frame,
				newPoints[0],
				6,
				cv::Scalar(0, 0, 255),
				-1
				);
			}
			
			oldPoints = newPoints;
		}

		cv::imshow("camera", frame);
		
		oldGray = gray.clone();

		if(cv::waitKey(1) == 'q') break;
	}

		cap.release();

		return 0;
}
