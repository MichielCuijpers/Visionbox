#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <cctype>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <string>
#include <stdint.h>
#include </opt/mvIMPACT_acquire/apps/Common/exampleHelper.h>
#include </opt/mvIMPACT_acquire/mvIMPACT_CPP/mvIMPACT_acquire.h>
#include </opt/mvIMPACT_acquire/mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>

using namespace std;
using namespace cv;
using namespace mvIMPACT::acquire;
using namespace mvIMPACT::acquire::GenICam;

int main(int argc, const char** argv)
{
	CvCapture* capture = 0;
	Mat frame, frameCopy, img, img2, grayImg, canny_output;
	/// Global variables

	Mat src, src_gray;
	Mat dst, detected_edges;
	RNG rng(12345);
	int edgeThresh = 1;
	int lowThreshold = 165;
	int const max_lowThreshold = 250;
	int ratio = 3;
	int kernel_size = 3;
	char* window_name = "Edge Map";


	int max_thresh = 255;

	cout << "Started - will wait for 1 min" << endl;
	//       sleep(15);
	cout << "Getting in...";
	//       sleep(5);
	DeviceManager devMgr;

	Device* pDev = devMgr[0]; //getDeviceFromUserInput( devMgr );
	if (!pDev)
	{
		cout << "Unable to continue!";
		cout << "Press [ENTER] to end the application" << endl;
		cin.get();
		return 0;
	}

	try
	{
		pDev->open();

	}
	catch (const ImpactAcquireException& e)
	{
		// this e.g. might happen if the same device is already opened in another process...
		cout << "An error occurred while opening the device(error code: " << e.getErrorCode() << "). Press [ENTER] to end the application..." << endl;
		cout << "Press [ENTER] to end the application" << endl;
		cin.get();
		return 0;
	}

	FunctionInterface fi(pDev);



	//     img2.create((pRequest2->imageHeight.read()),(pRequest2->imageWidth.read()),CV_8UC1);
	//img.create(960, 1280, CV_8UC4);

	int requestNr = 0, requestNr2 = 0;
	IOSubSystemBlueFOX ioss(pDev);

	DigitalOutput* pOutput = ioss.output(0);
	pOutput->set();
	cout << "values of output   :";
	cout << pOutput->get() << endl;
	cout << "Input read value  :" << ioss.input(0)->get() << " :  Value printed" << endl;
	while (1)
	{
		if (ioss.input(0)->get() == 0)
		{
			cout << "enterd loop";
			// send a request to the default request queue of the device and wait for the result.
			fi.imageRequestSingle();
			// Start the acquisition manually if this was requested(this is to prepare the driver for data capture and tell the device to start streaming data)
			if (pDev->acquisitionStartStopBehaviour.read() == assbUser)
			{
				TDMR_ERROR result = DMR_NO_ERROR;
				if ((result = static_cast<TDMR_ERROR>(fi.acquisitionStart())) != DMR_NO_ERROR)
				{
					cout << "'FunctionInterface.acquisitionStart' returned with an unexpected result: " << result
						<< "(" << ImpactAcquireException::getErrorCodeAsString(result) << ")" << endl;
				}
			}
			// Define the Image Result Timeout (The maximum time allowed for the Application
			// to wait for a Result). Infinity value:-1
			const int iMaxWaitTime_ms = -1;   // USB 1.1 on an embedded system needs a large timeout for the first image.
			// wait for results from the default capture queue.
			int requestNr = fi.imageRequestWaitFor(iMaxWaitTime_ms);

			// check if the image has been captured without any problems.
			if (!fi.isRequestNrValid(requestNr))
			{
				// If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
				// additional information under TDMR_ERROR in the interface reference
				cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString(requestNr) << ")"
					<< ", timeout value too small?" << endl;
				return 0;
			}

			const Request* pRequest = fi.getRequest(requestNr);
			if (!pRequest->isOK())
			{
				cout << "Error: " << pRequest->requestResult.readS() << endl;
				return 0;
			}

#ifndef NO_DISPLAY
			//    cout << "Please note that there will be just one refresh for the display window, so if it is" << endl
			//        << "hidden under another window the result will not be visible." << endl;
			// everything went well. Display the result
			// initialise display window
			//    ImageDisplayWindow display( "mvIMPACT_acquire sample" );
			//   display.GetImageDisplay().SetImage( pRequest );
			//  display.GetImageDisplay().Update();
#endif // NO_DISPLAY
			//  cout << "Image captured( " << pRequest->imagePixelFormat.readS() << " " << pRequest->imageWidth.read() << "x" << pRequest->imageHeight.read() << " )" << endl;
			//cout << pRequest->imageSize.read()<<endl;
		        img.create((pRequest->imageHeight.read()),(pRequest->imageWidth.read()),CV_8UC4);

			memcpy(img.data, (uchar*)pRequest->imageData.read(), (pRequest->imageSize.read()));


			fi.imageRequestUnlock(requestNr);
			char* source_window = "source";
			namedWindow(source_window, CV_WINDOW_NORMAL);
			int thresh = 50;
			/// Load source image and convert it to gray
			/// Convert image to gray and blur it
			cvtColor(img, src_gray, CV_BGR2GRAY);
			//  blur( src_gray, src_gray, Size(3,3) );

			/// Reduce noise with a kernel 3x3
			blur(src_gray, detected_edges, Size(3, 3));

			/// Canny detector
			Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
			//	Canny(detected_edges, detected_edges, 255, 255*ratio, kernel_size);

			/// Using Canny's output as a mask, we display our result
			dst = Scalar::all(0);

			src.copyTo(dst, detected_edges);
			dilate(detected_edges, detected_edges, 0);
			dilate(detected_edges, detected_edges, 1);
			//imshow(window_name, dst);
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			/// Find contours
			findContours(detected_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			/// Draw contours
			Mat drawing = Mat::zeros(detected_edges.size(), CV_8UC3);
			for (int i = 0; i< contours.size(); i++)
			{
				Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
				drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());

			}
			cvtColor(drawing, src_gray, CV_BGR2GRAY);

			/// Show in a window
			//	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
			//imshow("Contours", drawing);
			//waitKey(0);
			//	namedWindow("Contours1", CV_WINDOW_AUTOSIZE);
			threshold(src_gray, detected_edges, 0, 255, 0);
			dilate(detected_edges, detected_edges, 0);
			dilate(detected_edges, detected_edges, Mat(), Point(-1, -1), 2, 1, 1);

			/// Find contours
			findContours(detected_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

			int count = 0;
			for (int i = 0; i < contours.size(); i++)
			{
				double area = contourArea(contours[i]);
				Rect rect = boundingRect(contours[i]);
				//	cout << "Area  :" << area << endl;
				if (area <= 400 && area >= 75)
				{

					Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
					drawContours(drawing, contours, i, Scalar(255, 255, 255), 1, 8, hierarchy, 0, Point());
					count = count + 1;
					//	cout << "Rect" << i << "  :" << rect << endl;

				}
			}

			cout << "No of particles  : " << count << endl;
			if (count >= 4)
			{
				cout << "Component Passed" << endl;
				DigitalOutput* pOutput = ioss.output(0);
				pOutput->set();
				cout << "current state: " << pOutput->get() <<endl;
			}
			else
				cout << "fail" << endl;
			//	imshow("Contours1", drawing);
			//waitKey(0);
		}

	}
	cout << "Press [ENTER] to end the application" << endl;
	cin.get();
	return 0;

}
