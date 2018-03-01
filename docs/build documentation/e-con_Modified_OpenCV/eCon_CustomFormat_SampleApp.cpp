/* OpenCV Streaming Sample Custom format - Y16 and BY8
To test the Camera's Set the corresponding defines to 1 (CU40 - RGB IR Camera, CU51_12CUNIR - CU51/ 12CUNIR camera)
For 10CUG make both defines as 0 */

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <conio.h>

using namespace cv;
using namespace std;

#define CU40	0
#define CU51_12CUNIR	0 //Both CU51 or 12CUNIR

#define ImageWidth	1280
#define	ImageHeight  720

// Actual Data format BGIRR after conversion BGGR - IR is replaced with the G 
//IR data is collected as a separate image
bool ConvertRGIR2RGGB(Mat BayerRGIR, Mat &BayerRGGB, Mat &IRimage)
{
	//Result image after replacing the IR pixel with the G data
	BayerRGGB = BayerRGIR.clone();
	
	//IR data will be half the size of Bayer Image
	IRimage = Mat(BayerRGIR.size() / 2, CV_8UC1);

	//copying the IR data and replacing the IR data with G
	for (int Row = 0; Row < BayerRGIR.rows; Row += 2)
	{
		for (int Col = 0; Col < BayerRGIR.cols; Col += 2)
		{
			//Set the IR Data with Nearby Green 
			BayerRGGB.at<uchar>(Row + 1, Col) = BayerRGIR.at<uchar>(Row, Col + 1);
			//Set the IR Data 
			IRimage.at<uchar>(Row / 2, Col / 2) = BayerRGIR.at<uchar>(Row + 1, Col);
		}
	}

	return true;
}

// Main Function
int main()
{	
	cout << "	e-con's Sample OpenCV Application to  Custom Formats" << endl;
	cout << endl <<"Demonstrates the working of e-con's Custom Format(Y16 / BY8)";
	cout <<" cameras with the modified libraries of OpenCV" << endl << endl;

	char keyPressed;
	VideoCapture _CameraDevice;	
	Mat ResultImage, InputImage;
	Mat BayerFrame8, IRImage, BGRImage;
	
	//Open the device at the ID 0
	_CameraDevice.open(0);

	if( !_CameraDevice.isOpened()) //Check for the device
	{
		cout << endl << "\tCamera Device not Initialised Successfully" << endl;
		cout << endl << "Press any Key to exit the application" << endl << endl;
	
		_getch();
		return 0;
	}

	//Set up the width and height of the camera
	_CameraDevice.set(CV_CAP_PROP_FRAME_WIDTH,  ImageWidth);
	_CameraDevice.set(CV_CAP_PROP_FRAME_HEIGHT, ImageHeight);

	cout << "Press 'Q / q /Esc' key on the image winodw to exit the application" << endl;

	while(1)
	{
		_CameraDevice >> InputImage; //Read the input image

		if(InputImage.empty()) //Check for the vlid image
		{
			cout << "No frame grabbed!!, check whether the camera is free!!" << endl;
			break;
		}

#if CU51_12CUNIR
				
		//Convert to 8 Bit: 
		//Scale the 12 Bit (4096) Pixels into 8 Bit(255) (255/4096)= 0.06226
		convertScaleAbs(InputImage, ResultImage, 0.06226);

		namedWindow("Y16 to Y8", WINDOW_AUTOSIZE);
		imshow("Y16 to Y8", ResultImage);

#elif CU40
		
		//Convert to 8 Bit: 
		//Scale the 10 Bit (1024) Pixels into 8 Bit(255) (255/1024)= 0.249023
		convertScaleAbs(InputImage, BayerFrame8, 0.249023);

		//Filling the missing G -channel bayer data
		ConvertRGIR2RGGB(BayerFrame8, BayerFrame8, IRImage);
		
		//Actual Bayer format BG but Opencv uses BGR & Not RGB So taking RG Bayer format
		cvtColor(BayerFrame8, BGRImage, COLOR_BayerRG2BGR);

		namedWindow("Camera BGR Frame", WINDOW_AUTOSIZE);
		imshow("Camera BGR Frame", BGRImage);

		namedWindow("Camera IR Frame", WINDOW_AUTOSIZE);
		imshow("Camera IR Frame", IRImage);

#else //10CUG and other camera's

		namedWindow("Camera Frame", WINDOW_AUTOSIZE);
		imshow("Camera Frame", InputImage);
#endif

		keyPressed = waitKey(1); //Waits for a user input to quit the application

		if(keyPressed == 27 || keyPressed == 'q'  || keyPressed == 'Q' )
		{
			destroyAllWindows();
			break;
		}
	}

	//Release the devices
	_CameraDevice.release();
	
	cout << endl << "Press any Key to exit the application" << endl << endl;	
	_getch();
	
	return 1;
}
