#include <stdio.h>

#include "opencv2/opencv.hpp"

#include "opencv_aee.hpp"

#include "main.hpp"

#include "pi2c.h"

#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/highgui.hpp"

#include "iostream"

#include <wiringPiI2C.h>

using namespace cv;

using namespace std;

void Linefinding(Mat hsvImage,Mat& output, int* error) {

//RED

Mat redMask;

inRange(hsvImage, Scalar(160, 35, 40), Scalar(185, 245, 245), redMask);

int c1 = countNonZero(redMask);

threshold(redMask, redMask, 1, 255, THRESH_BINARY);

//Green

Mat greenMask;

inRange(hsvImage, Scalar(75, 55, 40), Scalar(95, 255, 255), greenMask);

int c2 = countNonZero(greenMask);

threshold(greenMask, greenMask, 1, 255, THRESH_BINARY);

//Blue

Mat blueMask;

inRange(hsvImage, Scalar(97, 40, 23), Scalar(120, 255, 250), blueMask);

int c3 = countNonZero(blueMask);

threshold(blueMask, blueMask, 1, 255, THRESH_BINARY);

//Yellow

Mat yellowMask;

inRange(hsvImage, Scalar(20, 100, 50), Scalar(40, 255, 255), yellowMask);

int c4 = countNonZero(yellowMask);

threshold(yellowMask, yellowMask, 1, 255, THRESH_BINARY);

//Black

Mat blackMask;

inRange(hsvImage, Scalar(0, 0, 0), Scalar(360, 150, 100), blackMask);

int c5 = countNonZero(blackMask);

threshold(blackMask, blackMask, 1, 255, THRESH_BINARY);

//Pink

Mat pinkMask;

inRange(hsvImage, Scalar(128, 0, 128), Scalar(218, 112,214), pinkMask);

int c6 = countNonZero(pinkMask);

threshold(pinkMask, pinkMask, 1, 255, THRESH_BINARY);

// Display the counts

printf("Red: %d¥nGreen: %d¥nBlue: %d¥nYellow: %d¥nBlack: %d¥nPink: %d¥n", c1, c2, c3, c4, c5, c6);

// Find the color with the maximum count

int maxCount = std::max({c1, c2, c3, c4, c5});

// Combine the masks of the most dominant color

Mat dominantMask;

if (maxCount == c1) {

dominantMask = redMask;

} else if (maxCount == c2) {

dominantMask = greenMask;

} else if (maxCount == c3) {

dominantMask = blueMask;

} else if (maxCount == c4) {

dominantMask = yellowMask;

} else if (maxCount == c5) {

dominantMask = blackMask;

}

// Write the counts onto the output image

int width = dominantMask.cols;

int height = dominantMask.rows;

Mat countsImg = Mat::zeros(height, width, CV_8UC1);

countsImg.setTo(255, dominantMask);

//create boxes act as sensors

int col_width = countsImg.cols / 6; // divide into 6 sections

int line1_x = col_width;

int line2_x = col_width * 2;

int line3_x = col_width * 3;

int line4_x = col_width * 4;

int line5_x = col_width * 5;

int line_y1 = 0;

int line_y2 = countsImg.rows;

int thickness = 1;

line(countsImg, Point(line1_x, line_y1), Point(line1_x, line_y2), Scalar(255), thickness);

line(countsImg, Point(line2_x, line_y1), Point(line2_x, line_y2), Scalar(255), thickness);

line(countsImg, Point(line3_x, line_y1), Point(line3_x, line_y2), Scalar(255), thickness);

line(countsImg, Point(line4_x, line_y1), Point(line4_x, line_y2), Scalar(255), thickness);

line(countsImg, Point(line5_x, line_y1), Point(line5_x, line_y2), Scalar(255), thickness);

// count white pixels in each box

int box1 = 0;

int box2 = 0;

int box3 = 0;

int box4 = 0;

int box5 = 0;

int box6 = 0;

for (int y = 0; y < countsImg.rows; y++) {

for (int x = 0; x < countsImg.cols; x++) {

if (countsImg.at<uchar>(y, x) == 255) {

if (x <= line1_x - thickness) {

box1++;

} else if (x > line1_x && x <= line2_x - thickness) {

box2++;

} else if (x > line2_x && x <= line3_x - thickness) {

box3++;

} else if (x > line3_x && x <= line4_x - thickness) {

box4++;

} else if (x > line4_x && x <= line5_x - thickness) {

box5++;

} else if (x > line5_x && x <= countsImg.cols - thickness) {

box6++;

}

}

}

}

cout << "Number of white pixels in height: " << countsImg.cols << endl;

cout << "Number of white pixels in row: " <<countsImg.rows << endl;

cout << "Number of white pixels in box1 : " << box1 << endl;

cout << "Number of white pixels in box2: " << box2 << endl;

cout << "Number of white pixels in box3: " << box3 << endl;

cout << "Number of white pixels in box4: " << box4 << endl;

cout << "Number of white pixels in box5: " << box5 << endl;

cout << "Number of white pixels in box6: " << box6 << endl;

imshow("Counts", countsImg);

//l1||r1 +l2|| r2+l3||r3

//max pixel count is 1664

//PID solution to find error value

int fd = wiringPiI2CSetup(0x04); // address of the Arduino Nano on the I2C bus

double total_pixels = box1 + box2 + box3 + box4 + box5 + box6;

//reference point is the middle of frame with the pixel coordinate of 160

double num1 = -133.3333;

double num2 = -80.0000;

double num3 = -26.6666;

double num4 = 26.6666;

double num5 = 80.0000;

double num6 = 133.3333;

double weighted_avg = ((box1*num1)+(box2*num2)+(box3*num3)+(box4*num4)+(box5*num5)+(box6*num6))/total_pixels;

//double weighted_avg = ((box1*-21)+(box2*-12)+(box3*-5)+(box4*5)+(box5* 12)+(box6* 21))/total_pixels;

*error = (weighted_avg);

//max and min is 133.3333/133

}

//Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

void setup(void) {

setupCamera(320, 20); // Enable the camera for OpenCV

}

int main(int argc, char** argv) {

setup(); // Call a setup function to prepare IO and devices

namedWindow("Photo"); // Create a GUI window called photo

while(true) { // Main loop to perform image processing

Mat frame;

while(frame.empty()) {

frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable

}

// Rotate the frame by 180 degrees

rotate(frame, frame, cv::ROTATE_180);

// Apply bilateral filter to the image to reduce noise while preserving edges

Mat bblurred;

cv::bilateralFilter(frame, bblurred, 10, 20, 10);

// Convert the image from BGR to HSV format

Mat hsvImage;

cvtColor(bblurred, hsvImage, COLOR_BGR2HSV);

Mat output;

int error;

Linefinding(hsvImage,output, &error);

cout << "Error: " << error << endl;

Pi2c arduino(4); //Create a new object "arduino" using address "0x04"

arduino.i2cWriteArduinoInt(error); //send error value to Arudino Nano
