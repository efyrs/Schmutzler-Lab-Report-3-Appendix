#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
    Mat image = imread("RedApple.png");
    Mat image_HSV, image_RED, image_BLUE, image_GREEN;
    GaussianBlur(image, image, Size(11,11),0);

    cvtColor(image, image_HSV, COLOR_BGR2HSV); // Convert the image to HSV
    inRange(image_HSV, Scalar(0,70,50), Scalar(19,255,255), image_RED); // red
    inRange(image_HSV, Scalar(78, 150, 50), Scalar(120, 255, 255), image_BLUE); // blue
    inRange(image_HSV, Scalar(35,50,50), Scalar (75,255,255), image_GREEN); // green

    // count pixels
    int redPixels = countNonZero(image_RED);
    int bluePixels = countNonZero(image_BLUE);
    int greenPixels = countNonZero(image_GREEN);

    //output colour with most pixels
    if (redPixels > bluePixels && redPixels > greenPixels) {
        cout << "The image contains mostly red pixels" << endl;
        cout << "Red Pixels: " << redPixels << endl;
        imshow("Red Pixels", image_RED);
    }
    else if (bluePixels > redPixels && bluePixels > greenPixels) {
        cout << "The image contains mostly blue pixels" << endl;
        cout << "Blue Pixels: " << bluePixels << endl;
        imshow("Blue Pixels", image_BLUE);
    }
    else if (greenPixels > redPixels && greenPixels > bluePixels) {
        cout << "The image contains mostly green pixels" << endl;
        cout << "Green Pixels: " << greenPixels << endl;
        imshow("Green Pixels", image_GREEN);
    }


    // display original
    imshow("Original Image", image);
    waitKey (0);
    return 0;

}
