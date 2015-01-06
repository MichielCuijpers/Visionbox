#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cctype>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <string>
#include <stdint.h>
#include </opt/mvIMPACT_acquire/apps/Common/exampleHelper.h>
#include </opt/mvIMPACT_acquire/mvIMPACT_CPP/mvIMPACT_acquire.h>

using namespace std;
using namespace cv;
using namespace mvIMPACT::acquire;

int main( int argc, const char** argv )
{
       CvCapture* capture = 0;
       Mat frame, frameCopy, img, img2, grayImg;
       cvNamedWindow( "result", CV_WINDOW_NORMAL );
       cvSetWindowProperty("result", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

       cout << "Started - will wait for 1 min" << endl;
//       sleep(15);
       cout << "Getting in...";
//       sleep(5);
    DeviceManager devMgr;

    Device* pDev = devMgr[0]; //getDeviceFromUserInput( devMgr );
    Device* pDev2 = devMgr[1];

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
	pDev2->open();
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

    FunctionInterface fi2(pDev2);

//     img.create((pRequest->imageHeight.read()),(pRequest->imageWidth.read()),CV_8UC4);
//     img2.create((pRequest2->imageHeight.read()),(pRequest2->imageWidth.read()),CV_8UC1);
   img.create(800,800,CV_8UC4);
   // img.create(960,1280,CV_8UC1);
   img2.create(800,800,CV_8UC4);
int requestNr = 0, requestNr2= 0;

//Point img2_Start_x = Point(0, 480);
Point img2_Start_x = Point(0, 400);
//Point img2_End_x = Point(1280+1280, 480);
Point img2_End_x = Point(800+800, 400);
//Point img2_Start_y = Point(640, 0);
Point img2_Start_y = Point(400, 0);
//Point img2_End_y = Point(640, 960);
Point img2_End_y = Point(400, 800);
//Point img2_Start_y1 = Point(1280+640,0);
Point img2_Start_y1 = Point(800+400,0);
//Point img2_End_y1 = Point(1280+640,960);
Point img2_End_y1 = Point(800+400,800);

Point img2_Start_y2 = Point(800,0);
Point img2_End_y2 = Point(800,800);

while (1)
{
    // send a request to the default request queue of the device and wait for the result.
    fi.imageRequestSingle();
    // Start the acquisition manually if this was requested(this is to prepare the driver for data capture and tell the device to start streaming data)
    if( pDev->acquisitionStartStopBehaviour.read() == assbUser )
    {
        TDMR_ERROR result = DMR_NO_ERROR;
        if( ( result = static_cast<TDMR_ERROR>( fi.acquisitionStart() ) ) != DMR_NO_ERROR )
        {
            cout << "'FunctionInterface.acquisitionStart' returned with an unexpected result: " << result
                 << "(" << ImpactAcquireException::getErrorCodeAsString( result ) << ")" << endl;
        }
    }
    // Define the Image Result Timeout (The maximum time allowed for the Application
    // to wait for a Result). Infinity value:-1
    const int iMaxWaitTime_ms = -1;   // USB 1.1 on an embedded system needs a large timeout for the first image.
    // wait for results from the default capture queue.
    int requestNr = fi.imageRequestWaitFor( iMaxWaitTime_ms );

    // check if the image has been captured without any problems.
    if( !fi.isRequestNrValid( requestNr ) )
    {
        // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
        // additional information under TDMR_ERROR in the interface reference
        cout << "imageRequestWaitFor failed (" << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr ) << ")"
             << ", timeout value too small?" << endl;
        return 0;
    }

    const Request* pRequest = fi.getRequest( requestNr );
    if( !pRequest->isOK() )
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
    memcpy(img.data,(uchar*)pRequest->imageData.read(), (pRequest->imageSize.read()));
  //  cv::imshow( "result", img );
 //   waitKey(1000);
    //	Second Camera ##############################################
    // send a request to the default request queue of the device and wait for the result.
    fi2.imageRequestSingle();
    // Start the acquisition manually if this was requested(this is to prepare the driver for data capture and tell the device to start streaming data)
    if( pDev2->acquisitionStartStopBehaviour.read() == assbUser )
    {
        TDMR_ERROR result = DMR_NO_ERROR;
        if( ( result = static_cast<TDMR_ERROR>( fi2.acquisitionStart() ) ) != DMR_NO_ERROR )
        {
            cout << "'FunctionInterface.acquisitionStart' returned with an unexpected result: " << result
                 << "(" << ImpactAcquireException::getErrorCodeAsString( result ) << ")" << endl;
        }
    }
    // Define the Image Result Timeout (The maximum time allowed for the Application
    // to wait for a Result). Infinity value:-1
   // iMaxWaitTime_ms = -1;   // USB 1.1 on an embedded system needs a large timeout for the first image.
    // wait for results from the default capture queue.
    requestNr2 = fi2.imageRequestWaitFor( iMaxWaitTime_ms );

    // check if the image has been captured without any problems.
    if( !fi2.isRequestNrValid( requestNr2 ) )
    {
        // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
        // additional information under TDMR_ERROR in the interface reference
        cout << "imageRequestWaitFor failed (" << requestNr2 << ", " << ImpactAcquireException::getErrorCodeAsString( requestNr2 ) << ")"
             << ", timeout value too small?" << endl;
        return 0;
    }

    const Request* pRequest2 = fi2.getRequest( requestNr2 );
    if( !pRequest2->isOK() )
    {
        cout << "Error: " << pRequest2->requestResult.readS() << endl;
        // if the application wouldn't terminate at this point this buffer HAS TO be unlocked before
        // it can be used again as currently it is under control of the user. However terminating the application
        // will free the resources anyway thus the call
        // fi.imageRequestUnlock( requestNr );
        // can be omitted here.
        return 0;
    }
//    cout << "2nd Image captured( " << pRequest2->imagePixelFormat.readS() << " " << pRequest2->imageWidth.read() << "x" << pRequest2->imageHeight.read() << " )" << endl;
    // unlock the buffer to let the driver know that you no longer need this buffer.
   // Mat cimg;
  //      Mat left (img, Rect(0, 0, pRequest->imageWidth.read(),pRequest->imageHeight,read());
//	pRequest->imageData.read().copyTo (left);
 //       Mat right(img, Rect(pRequest->imageWidth.read(), 0, pRequest->image.Width.read(),pRequest->imageHeight.read()));
//	pRequest->imageData.read().copyTo (right);
  //	memcpy(img.data,(uchar*)pRequest->imageData.read(), (pRequest->imageSize.read()));
	memcpy(img2.data,(uchar*)pRequest2->imageData.read(), (pRequest2->imageSize.read()));
	// cvtColor(img2, grayImg, CV_RGB2GRAY);
	//Mat trial(Size(732,732),CV_8UC4);
//	Mat roi1(img, Rect(0,0,800,800));
//	Mat roi2(img2, Rect(0,0,800,800));
//	cv::imshow( "result", roi );
//	waitKey(0);

//	Size sz1 =img.size();
	Size sz1 =img.size();
//	Size sz2 =img2.size();
	Size sz2 =img2.size();

	Mat merge(sz1.height, sz1.width+sz2.width, CV_8UC4);
	Mat left(merge, Rect(0, 0,sz1.width, sz1.height));
	img.copyTo(left);

	Mat right(merge, Rect(sz1.width, 0, sz2.width, sz2.height));
	img2.copyTo(right);
//        cvtColor(img,cimg,CV_RGB2GRAY);
//	img.cols=640;//(pRequest->imageWidth);
//	img.rows=480;//(pRequest->imageHeight);
//	img.data=(uchar*) pRequest->imageData.read();

	fi.imageRequestUnlock( requestNr );
	fi2.imageRequestUnlock( requestNr2 );

     //  cv::imshow( "result", merge );
//	cv::imshow( "result", img );
//	waitKey(1000);

	line(merge, img2_Start_x, img2_End_x, Scalar(255,255,255), 2, 8);
	line(merge, img2_Start_y, img2_End_y, Scalar(255,255,255), 2, 8);
	line(merge, img2_Start_y1, img2_End_y1, Scalar(255,255,255), 2, 8);
	line(merge, img2_Start_y2, img2_End_y2, Scalar(122,252,55), 4, 8);

	cv::imshow( "result", merge );
	waitKey(5);

//	cvtColor(img,cimg,CV_GRAY2RGB);
//       cv::imshow("result", cimg);
//        waitKey(0);


}
    cout << "Press [ENTER] to end the application" << endl;
    cin.get();
    return 0;

}
