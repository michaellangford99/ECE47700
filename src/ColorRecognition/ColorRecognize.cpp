#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "ColorRecognize.h"
#include <istream>

void ColorRecognize::ColorRecognizeImage(std::string imageFile, cv::Scalar maxColor, cv::Scalar minColor) {
	cv::Mat img = cv::imread(imageFile);
	imageToRecognize = img;
	//cv::imshow("image", img);
	cv::Mat converted;
	cv::cvtColor(img, converted, cv::COLOR_BGR2HSV);
	//cv::imshow("converted", converted);
	cv::Mat mask;
	cv::inRange(converted, minColor, maxColor, mask);
	//cv::imshow("mask", mask);
	cv::Mat output;
	cv::bitwise_and(img, converted, output ,mask);
	cv::imshow("output", output);
	cv::waitKey(0);
}

void ColorRecognize::ColorRecognizeVid(int deviceID, cv::Scalar maxColor, cv::Scalar minColor) {
	cv::VideoCapture Video = cv::VideoCapture(deviceID, cv::CAP_ANY);
	if (!(Video.isOpened())) {
		std::cerr << "Error reading Camera" << std::endl;
	}
	cv::Mat converted;
	cv::Mat frameCaught;
	cv::Mat output;
	cv::Mat mask;
	int width = 0;
	int height = 0;
	int counter = 0;
	while (1) {
		if (!(Video.read(frameCaught))) {
			std::cerr << "Problems reading data from camera" << std::endl;
		}
		width = frameCaught.size[1];
		height = frameCaught.size[0];
		cv::cvtColor(frameCaught, converted, cv::COLOR_BGR2HSV);
		cv::inRange(converted, minColor, maxColor, mask);
		cv::Moments oMoments = moments(mask);
		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;
		if (dArea > 100000) {
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;
			std::cerr << "Object found" << std::endl;
		}
		cv::Mat output;
		cv::bitwise_and(frameCaught, converted, output, mask);
		cv::imshow("output", output);
		if (cv::waitKey(5) >= 0) {
			break;
		}
		counter++;
	}
}