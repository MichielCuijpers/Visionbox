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

int main( int argc, const char** argv )
{
       CvCapture* capture = 0;
      Mat frame, frameCopy, img, img2, grayImg, canny_output;
	   Mat src; Mat src_gray;
	   int max_thresh = 255;
	   RNG rng(12345);

       cout << "Started - will wait for 1 min" << endl;
//       sleep(15);
       cout << "Getting in...";
//       sleep(5);
    DeviceManager devMgr;

    Device* pDev = devMgr[0]; //getDeviceFromUserInput( devMgr );
      if( !pDev )
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
    catch( const ImpactAcquireException& e )
    {
        // this e.g. might happen if the same device is already opened in another process...
        cout << "An error occurred while opening the device(error code: " << e.getErrorCode() << "). Press [ENTER] to end the application..." << endl;
        cout << "Press [ENTER] to end the application" << endl;
        cin.get();
        return 0;
    }

    FunctionInterface fi( pDev );

  

//     img.create((pRequest->imageHeight.read()),(pRequest->imageWidth.read()),CV_8UC4);
//     img2.create((pRequest2->imageHeight.read()),(pRequest2->imageWidth.read()),CV_8UC1);
   img.create(960,1280,CV_8UC4);
  
int requestNr = 0, requestNr2= 0;
IOSubSystemBlueFOX ioss (pDev);

DigitalOutput* pOutput = ioss.output (0);
pOutput->set();
cout << "values of output   :";
cout << pOutput->get()<< endl;
cout << "Input read value  :" << ioss.input(0)->get()<< " :  Value printed" <<endl;
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
		memcpy(img.data, (uchar*)pRequest->imageData.read(), (pRequest->imageSize.read()));


		fi.imageRequestUnlock(requestNr);
	char* source_window = "source";	
	namedWindow(source_window, CV_WINDOW_NORMAL);
		int thresh = 50;
		/// Load source image and convert it to gray
		/// Convert image to gray and blur it
		cvtColor(img, src_gray, CV_BGR2GRAY);
		//  blur( src_gray, src_gray, Size(3,3) );

		vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	blur(src_gray, src_gray, Size(3, 3));
	/// Detect edges using canny
	//Canny(src_gray, canny_output, thresh, 255, 3);
	threshold(src_gray, canny_output, 38, 255, 0);
	imshow(source_window, img);
	waitKey(0);
	dilate(canny_output, canny_output, 1);
	/// Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/// Draw contours
	Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
	for (int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		Rect rect = boundingRect(contours[i]);
		if (area <= 6000 && area >= 100)
		{

			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			drawContours(drawing, contours, i, Scalar(255, 255, 255), -1, 8, hierarchy, 0, Point());
		}
	}
	//dilate(drawing, drawing, 1);

	/// Show in a window
	namedWindow("Contours", CV_WINDOW_NORMAL);

	int erosion_type = 1;
	int erosion_size = 2;
	Mat element = getStructuringElement(erosion_type,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));

	//Apply morphological opening operation to eliminate unwanted region.
	Size kernalSize(5, 4); //kernal size may change for differnt image
	Mat element1 = getStructuringElement(MORPH_RECT, kernalSize, Point(1, 1));
	morphologyEx(drawing, drawing, MORPH_CLOSE, element1);
	cvtColor(drawing, drawing, CV_BGR2GRAY);
	threshold(drawing, drawing, 0, 255, 0);
	imshow("Contours", drawing);
	waitKey(0);
	findContours(drawing, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	/// Draw contours
	Mat drawing1 = Mat::zeros(drawing.size(), CV_8UC3);
	int count = 0;
	for (int i = 0; i < contours.size(); i++)
	{
		double area = contourArea(contours[i]);
		Rect rect = boundingRect(contours[i]);
		if (area >= 100 && area <= 8000 && rect.width <= 600 && rect.height <= 600)
		{

			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			drawContours(drawing1, contours, i, Scalar(255, 255, 255), 2, 8, hierarchy, 0, Point());
			count++;
		}
	}
	cout << "Total count is " << count << endl;
	if ( count >= 3 )
	cout << "Object Present"<<endl;
	imshow("Contours", drawing1);
	waitKey(0);


	}
	else
	cout << "Read Value is 0" << endl;
}
    cout << "Press [ENTER] to end the application" << endl;
    cin.get();
    return 0;

}
