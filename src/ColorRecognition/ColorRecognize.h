#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
class ColorRecognize {
public:
	cv::Mat imageToRecognize;
	cv::Mat maskGenerated;
	bool ColorDetect;
	void ColorRecognizeImage(std::string imageFile, cv::Scalar maxColor, cv::Scalar minColor);
	void ColorRecognizeVid(int deviceID, cv::Scalar maxColor, cv::Scalar minColor);

};