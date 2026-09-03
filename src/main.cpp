#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

void judgementPointStatus(const std::vector<uchar>& status, auto& newPOints) {
	
	int count = 0;

		for (int i = 0; i < status.size(); i++) {
			
			if (!status[i]) {
				
				count++;
				
				if (count > 50) {
					pointStatus = false;
					continue
				}
			}
			else {	
				CPoints.push_back(newPoints[i]);
			}
		}	

	return PointStatus;
}

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

		std::vector<cv::Point2f> CPoints;

		std::vector<uchar> status;

		bool pointStatus = true;

		std::vector<float> error;

		cv::calcOpticalFlowPyrLK(
				oldGray,
				gray,
				oldPoints,
				newPoints,
				status,
				error
			);
	
		if (newPoints.size() <= 50) {
			cv::goodFeaturesToTrack(
					oldGray,oldPoints,
					100,
					0.01,
					10
					);
		}	

		pointStatus = judgementPointStatus()

		if (!status.empty() && pointStatus){
			
			for (int i = 0; i < CPoints.size(); i++) {
				cv::circle(
				frame,
				CPoints[i],
				6,
				cv::Scalar(0, 0, 255),
				-1
				);
			}
			
		}

		cv::imshow("camera", frame);
		
		oldGray = gray.clone();

		if(cv::waitKey(1) == 'q') break;
	}

		cap.release();

		return 0;
}
