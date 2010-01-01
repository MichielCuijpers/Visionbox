#include </opt/mvIMPACT_acquire/common/minmax.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>

using namespace mvIMPACT::acquire::GenICam;


#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <cctype>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <string>
#include <stdint.h>
#include <unistd.h>
#include </opt/mvIMPACT_acquire/apps/Common/exampleHelper.h>
#include </opt/mvIMPACT_acquire/mvIMPACT_CPP/mvIMPACT_acquire.h>

/* For UART */
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <sys/ioctl.h>

using namespace std;
using namespace cv;
using namespace mvIMPACT::acquire;

#define TRUE 1
#define FALSE -1


const char* trackbar_value = "Value";
int Tvalue =0;


Mat src; Mat src_gray;
int thresh = 62;
int max_thresh = 255;
RNG rng(12345);

Mat CV_EXPORTS FillHoles(Mat _src);
///////////////////////////
typedef uint8_t BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
typedef int32_t LONG;
typedef bool BOOLEAN;

#   ifdef __GNUC__
#       define BMP_ATTR_PACK __attribute__((packed)) __attribute__ ((aligned (2)))
#   else
#       define BMP_ATTR_PACK
#   endif // #ifdef __GNUC__

typedef struct tagRGBQUAD
{
    BYTE    rgbBlue;
    BYTE    rgbGreen;
    BYTE    rgbRed;
    BYTE    rgbReserved;
} BMP_ATTR_PACK RGBQUAD;

typedef struct tagBITMAPINFOHEADER
{
    DWORD  biSize;
    LONG   biWidth;
    LONG   biHeight;
    WORD   biPlanes;
    WORD   biBitCount;
    DWORD  biCompression;
    DWORD  biSizeImage;
    LONG   biXPelsPerMeter;
    LONG   biYPelsPerMeter;
    DWORD  biClrUsed;
    DWORD  biClrImportant;
} BMP_ATTR_PACK BITMAPINFOHEADER, *PBITMAPINFOHEADER;

typedef struct tagBITMAPFILEHEADER
{
    WORD    bfType;
    DWORD   bfSize;
    WORD    bfReserved1;
    WORD    bfReserved2;
    DWORD   bfOffBits;
} BMP_ATTR_PACK BITMAPFILEHEADER, *PBITMAPFILEHEADER;

//-----------------------------------------------------------------------------
int SaveBMP( const string& filename, const char* pdata, int XSize, int YSize, int pitch, int bitsPerPixel )
//------------------------------------------------------------------------------
{
    static const WORD PALETTE_ENTRIES = 256;

    if( pdata )
    {
        FILE* pFile = fopen( filename.c_str(), "wb" );
        if( pFile )
        {
            BITMAPINFOHEADER    bih;
            BITMAPFILEHEADER    bfh;
            WORD                linelen = static_cast<WORD>( ( XSize * bitsPerPixel + 31 ) / 32 * 4 );  // DWORD aligned
            int                 YPos;
            int                 YStart = 0;

            memset( &bfh, 0, sizeof( BITMAPFILEHEADER ) );
            memset( &bih, 0, sizeof( BITMAPINFOHEADER ) );
            bfh.bfType          = 0x4d42;
            bfh.bfSize          = sizeof( bih ) + sizeof( bfh ) + sizeof( RGBQUAD ) * PALETTE_ENTRIES + static_cast<LONG>( linelen ) * static_cast<LONG>( YSize );
            bfh.bfOffBits       = sizeof( bih ) + sizeof( bfh ) + sizeof( RGBQUAD ) * PALETTE_ENTRIES;
            bih.biSize          = sizeof( bih );
            bih.biWidth         = XSize;
            bih.biHeight        = YSize;
            bih.biPlanes        = 1;
            bih.biBitCount      = static_cast<WORD>( bitsPerPixel );
            bih.biSizeImage     = static_cast<DWORD>( linelen ) * static_cast<DWORD>( YSize );

            if( ( fwrite( &bfh, sizeof( bfh ), 1, pFile ) == 1 ) && ( fwrite( &bih, sizeof( bih ), 1, pFile ) == 1 ) )
            {
                RGBQUAD rgbQ;
                for( int i = 0; i < PALETTE_ENTRIES; i++ )
                {
                    rgbQ.rgbRed      = static_cast<BYTE>( i );
                    rgbQ.rgbGreen    = static_cast<BYTE>( i );
                    rgbQ.rgbBlue     = static_cast<BYTE>( i );
                    rgbQ.rgbReserved = static_cast<BYTE>( 0 );
                    fwrite( &rgbQ, sizeof( rgbQ ), 1, pFile );
                }

                for( YPos = YStart + YSize - 1; YPos >= YStart; YPos-- )
                {
                    if( fwrite( &pdata[YPos * pitch], linelen, 1, pFile ) != 1 )
                    {
                        cout << "SaveBmp: ERR_WRITE_FILE: " << filename << endl;
                    }
                }
            }
            else
            {
                cout << "SaveBmp: ERR_WRITE_FILE: " << filename << endl;
            }
            fclose( pFile );
	return 0;
        }
        else
        {
            cout << "SaveBmp: ERR_CREATE_FILE: " << filename << endl;
	    return 1;
}
    }
    else
    {
        cout << "SaveBmp: ERR_DATA_INVALID:" << filename << endl;
    }
}


int main(int argc, const char** argv)
{
	char s[33];
	int fp;
	size_t itime;
	struct tm tim;
	time_t now;
	int bmperror;
	int fd;
	int nread;
	int nwrite;
	int n = 0;
	int i = 0;
	char buffer[20];
	char test;
	int k;
	k=1;
	string filename;
	ifstream gpioread;
	char value;
	char oldValue;
	oldValue ='0';
	const char valueset = '1';
	const char valuereset = '0';
	CvCapture* capture = 0;
	Mat frame, frameCopy, img, img2, grayImg;
	cvNamedWindow("result", CV_WINDOW_NORMAL);
	cvSetWindowProperty("result", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	cout << "Getting in...";
	DeviceManager devMgr;

	Device* pDev = devMgr[0]; //getDeviceFromUserInput( devMgr );
	struct stat st;

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
	img.create(1944, 2592, CV_8UC4);

	int requestNr = 0;

int bytes = 0;
struct timespec tstart = {0,0}, tend = {0,0};

double dblTimeDiff;

	while (1)
	{
			oldValue='1';
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
			memcpy(img.data, (uchar*)pRequest->imageData.read(), (pRequest->imageSize.read()));

			IOSubSystemBlueFOX ioss( pDev );
			if ( ioss.input ( 0 )->get() == 1 && k == 1 )
			{
			k=2;
			}
			if ( ioss.input ( 0 )->get() ==  0)
			{
			k=1;
			}

	 		if(stat("/SDCARD/",&st) == 0)
		        {
			if (k == 2)
			{
			time_t now;
			s[0] = '\0';
			now = time(NULL);
			strftime(s,33,"/SDCARD/%Y%m%d%H%M%S.bmp",gmtime(&now));
			string filename(s);
			cout<< "Saving to the file "<<endl;
			cout << filename<<endl;
			bmperror=SaveBMP( filename, reinterpret_cast<char*>( pRequest->imageData.read() ), pRequest->imageWidth.read(), pRequest->imageHeight.read(), pRequest->imageLinePitch.read(), pRequest->imagePixelPitch.read() * 8 );
			cout << "Bmp error: "<<bmperror<<endl;
			k=0;
			if ( bmperror == 0 )
			cv::putText(img,"IMAGE STORED", Point(30, 600), FONT_HERSHEY_SIMPLEX, 4, Scalar(200, 200, 250), 10);

			}
			}
			cv::imshow("result", img);
			waitKey(5);
			fi.imageRequestUnlock(requestNr);

		}

	cout << "Press [ENTER] to end the application" << endl;
	cin.get();
	return 0;

}

