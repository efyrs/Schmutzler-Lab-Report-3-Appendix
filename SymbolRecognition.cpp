//symbol code

//symbol coode 29/04/23 draft 2

#include <stdio.h>

#include "opencv2/opencv.hpp"

#include "opencv_aee.hpp"

#include "main.hpp"

#include "pi2c.h"

#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/highgui.hpp"

#include "iostream"

#include <wiringPiI2C.h>

#include <opencv2/objdetect/objdetect.hpp>

#include <opencv2/highgui/highgui.hpp>

using namespace cv;

using namespace std;

//function prototype

void symbolfind(int pinkCount, int* match1 , int* match2 , int* match3 , int* match4, cv::Mat* symbolImage);

//symbol

void Linefinding(Mat hsvImage,Mat& output, int* error, int* pinkCount, Mat* symbolImage) {

//RED

Mat redMask;

inRange(hsvImage, Scalar(160, 35, 40), Scalar(185, 245, 245), redMask);

int c1 = countNonZero(redMask);

//Pink

Mat pinkMask;

inRange(hsvImage, Scalar(128, 0, 128), Scalar(218, 112,214), pinkMask);

int c6 = countNonZero(pinkMask);

*pinkCount = c6;

*symbolImage = pinkMask;

// Write the counts onto the output image

int width = redMask.cols;

int height = redMask.rows;

Mat countsImg = Mat::zeros(height, width, CV_8UC1);

countsImg.setTo(255, redMask);

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

}

//Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

void setup(void) {

setupCamera(325, 20); // Enable the camera for OpenCV

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

//resize(frame, frame, Size(320, 60));

// Apply bilateral filter to the image to reduce noise while preserving edges

Mat bblurred;

cv::bilateralFilter(frame, bblurred, 10, 20, 10);

// Convert the image from BGR to HSV format

Mat hsvImage;

cvtColor(bblurred, hsvImage, COLOR_BGR2HSV);

Mat output;

int error;

int pinkCount;

Mat symbolImage;

Linefinding(hsvImage,output, &error, &pinkCount, &symbolImage);

cout << "Error: " << error << endl;

cout << "Pink count: " << pinkCount << endl;

int match1;

int match2;

int match3;

int match4;

symbolfind(pinkCount, &match1 , &match2 , &match3 , &match4, &symbolImage);

Linefinding(hsvImage,output, &error, &pinkCount, &symbolImage);

if (match1 > 80){

printf("Symbol match percentage: %f" ,match1);

} else{

printf("No match (Cirle)");

}

if (match2 > 80){

printf("Symbol match percentage: %f" ,match2);

} else{

printf("No match (Star)");

}

if (match3 > 80){

printf("Symbol match percentage: %f" ,match3);

} else{

printf("No match (Triangle)");

}

if (match4 > 80){

printf("Symbol match percentage: %f" ,match4);

} else{

printf("No match (Umbrella)");

}

//Pi2c arduino(4); //Create a new object "arduino" using address "0x04"

//arduino.i2cWriteArduinoInt(error); //send error value to Arudino Nano

cv::imshow("Photo", frame); // Display the image in the window

int key = cv::waitKey(1); // Wait 1ms for a keypress (required to update windows)

if (key == 27) { // Check if the ESC key has been pressed

break;

}

}

closeCV(); // Disable the camera and close any windows

return 0;

}

void symbolfind(int pinkCount, int* match1 , int* match2 , int* match3 , int* match4, cv::Mat* symbolImage){

//if there are pink pixels code executes

if (pinkCount > 1000){

resizeCamera(400,400); // Resize the camera frame

std::vector<std::vector<cv::Point> > contours;

std::vector<cv::Vec4i> hierarchy;

cv::findContours(symbolImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0)); // Find contours

std::vector<std::vector<cv::Point> > approxedcontours(contours.size()); // Array for new contours

for (int i = 0; i < contours.size(); i++)

{

cv::approxPolyDP(contours[i], approxedcontours[i], 10, true); // Approximate the contour

}

cv::drawContours(symbolImage, approxedcontours, -1, cv::Scalar(0, 0, 0), 2); // Draw the approximated contours on the image

cv::Rect bounding_rect = cv::boundingRect(approxedcontours[0]); // Find the bounding rectangle around the contours

for (int i = 1; i < approxedcontours.size(); i++)

{

bounding_rect |= cv::boundingRect(approxedcontours[i]);

}

cv::rectangle(image_HSV1, bounding_rect, cv::Scalar(0, 0, 0),LINE_8); // Draw the bounding rectangle on the image

double maxArea = 0;

int maxAreaIdx = -1;

for (int i = 0; i < contours.size(); i++) {

double area = cv::contourArea(contours[i]);

if (area > maxArea) {

maxArea = area;

maxAreaIdx = i;

}

}

Rect bRect = bounding_Rect(contours[maxAreaIdx]);

Point2f src_corners[4];

src_corners[0] = Point2f(bRect.x, bRect.y);

src_corners[1] = Point2f(bRect.x + bRect.width, bRect.y);

src_corners[2] = Point2f(bRect.x + bRect.width, bRect.y + bRect.height);

src_corners[3] = Point2f(bRect.x, bRect.y + bRect.height);

Point2f dst_corners[4];

dst_corners[0] = Point2f(0, 0);

dst_corners[1] = Point2f(349, 0);

dst_corners[2] = Point2f(349, 349);

dst_corners[3] = Point2f(0, 349);

//transform symbol

Mat transform_matrix = cv::getPerspectiveTransform(src_corners, dst_corners);

Mat output_image;

warpPerspective(symbolImage, output_image, transform_matrix, cv::Size(350, 350));

//loads images

Mat sym1;

sym1 = imread("CirleRL.png");

Mat sym2;

sym2 = imread("StarGL.png");

Mat sym3;

sym3 = imread("TriangleBL.png");

Mat sym4;

sym4 = imread("UmbrellaYL.png");

//checks if image has loaded

if (sym1.empty() || sym2.empty() || sym3.empty() || sym4.empty()) {

// check if any of the images failed to load

printf("Failed to load image(s)!¥n");

}

imshow("s1", sym1);

imshow("s2",sym2);

imshow("s3",sym3);

imshow("s4", sym4);

//converts to gray scale

Mat sym1Gray;

cvtColor(sym1, sym1Gray, COLOR_BGR2GRAY);

Mat sym2Gray;

cvtColor(sym2, sym2Gray, COLOR_BGR2GRAY);

Mat sym3Gray;

cvtColor(sym3, sym3Gray, COLOR_BGR2GRAY);

Mat sym4Gray;

cvtColor(sym4, sym4Gray, COLOR_BGR2GRAY);

//converts gray image to binary

Mat sym1Bin;

threshold(sym1Gray, sym1Bin, 200, 255, THRESH_BINARY);

Mat sym2Bin;

threshold(sym2Gray, sym2Bin, 200, 255, THRESH_BINARY);

Mat sym3Bin;

threshold(sym3Gray, sym3Bin, 200, 255, THRESH_BINARY);

Mat sym4Bin;

threshold(sym4Gray, sym4Bin, 200, 255, THRESH_BINARY);

//compares frame to new binary image

*match1 = compareImages(output_image,sym1Bin);

*match2 = compareImages(output_image,sym2Bin);

*match3 = compareImages(output_image,sym3Bin);

*match4 = compareImages(output_image,sym4Bin);

}

}
