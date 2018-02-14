//------------------------------------------------------------------------------
// <copyright file="ColorBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "ColorBasics.h"

#include "stdafx.h"
#include <strsafe.h>
#include <iostream>
#include <fstream>
#include <ctime>
//#include "resource.h"
#include <vector>
#include <unordered_map>

#include <iostream>


class DisjointSet {
public:
	DisjointSet(int num_nodes);
	int find(int element);
	void do_union(int elment_a, int element_b);
	void dup(vector<int> &v);
private:
	vector<int> parent_, rank_;
};

DisjointSet::DisjointSet(int num_nodes) :
	parent_(num_nodes), rank_(num_nodes, 0)
{
	for (int i = 0; i<num_nodes; ++i) parent_[i] = i;
}

int DisjointSet::find(int element)
{
	if (parent_[element] == element) return element;
	//1)path compression
	return parent_[element] = find(parent_[element]);
}

void DisjointSet::do_union(int a, int b)
{
	if (parent_[a] == parent_[b]) return;
	int fa = find(a), fb = find(b);
	//2)union by rank
	if (rank_[fa] < rank_[fb]) {
		parent_[fa] = fb;
	}
	else {
		parent_[fb] = fa;
		if (rank_[fa] == rank_[fb]) rank_[fa]++;
	}
}

enum joint {
	NECK,
	SHOULDER_LEFT,
	SHOULDER_RIGHT,
	HIP_LEFT,
	HIP_RIGHT,
	ELBOW_LEFT,
	ELBOW_RIGHT,
	INVALID_JOINT
};

UINT16 CColorBasics::getValue(int y, int x, int width)
{
	return m_depthBuffer[(y * width) + x];
}

#define grid(y, x) getValue(y, x, width)
void CColorBasics::Trianglez(UINT nCapacity, int width, int height, DepthSpacePoint dsp, short threshold)
{
	DisjointSet ds(height * width);

    for (int i = 1; i < width; i++) {
        if (abs(grid(0, i) - grid(0, i-1)) < threshold) {
            ds.do_union(i, i-1);
        }
    }
    
    for (int i = 1; i < height; i++) {
        if (abs(grid(i, 0) - grid(i-1, 0)) < threshold) {
            ds.do_union(i*width, (i-1)*width);
        }
    }
    
    for (int i = 1; i < height; i++) {
        for (int j = 1; j < width; j++) {
			if (abs(grid(i, j) - grid(i-1, j)) < threshold) {
                ds.do_union(i*width + j, (i-1)*width+j);
            }
            if (abs(grid(i, j-1) - grid(i, j)) < threshold) {
                ds.do_union(i*width + j, i*width+j-1);
            }
        }
    }

	int personComponent = ds.find(width * ((int) (dsp.Y)) + (int) (dsp.X));
	Output("personComponent: %d\n", personComponent);

	string OUTPUT_DIRECTORY = "C:\\Users\\Ryan\\~\\code\\fydp-kinect-app\\out\\depth";
	string FILE_EXTENSION = "-good.pgm";
	time_t timestamp = time(nullptr);
	char* filepath = new char[OUTPUT_DIRECTORY.length() + FILE_EXTENSION.length() + 32];
	sprintf(filepath, "%s-%u-%d%s", OUTPUT_DIRECTORY.c_str(), (int)timestamp, threshold, FILE_EXTENSION.c_str());
	ofstream myfile(filepath, ios::out | ios::binary);

	if (myfile.is_open())
	{
		char* header = new char[200];
		sprintf(header, "P5 %d %d %d\n", width, height, 4500);

		myfile << header;
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				int num = ds.find(width * i + j);
				if (num == personComponent) {
					myfile << "11";
				}
				else {
					char zero = 0;
					myfile << zero << zero;
				}
			}
		}

		myfile.close();
	}
}

joint convertStringToJoint(string jointName) {
	if (jointName == "Neck") return NECK;
	if (jointName == "ShoulderLeft") return SHOULDER_LEFT;
	if (jointName == "ShoulderRight") return SHOULDER_RIGHT;
	if (jointName == "HipLeft") return HIP_LEFT;
	if (jointName == "HipRight") return HIP_RIGHT;
	if (jointName == "ElbowLeft") return ELBOW_LEFT;
	if (jointName == "ElbowRight") return ELBOW_RIGHT;
	return INVALID_JOINT;
}

void CColorBasics::Output(const char* szFormat, ...)
{
	char szBuff[1024];
	va_list arg;
	va_start(arg, szFormat);
	_vsnprintf(szBuff, sizeof(szBuff), szFormat, arg);
	va_end(arg);

	OutputDebugStringA(szBuff);
}

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(    
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
    )
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CColorBasics application;
    application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CColorBasics::CColorBasics() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_bSaveScreenshot(false),
    m_pKinectSensor(NULL),
    m_pColorFrameReader(NULL),
    m_pD2DFactory(NULL),
    m_pDrawColor(NULL),
    m_pColorRGBX(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

    // create heap storage for color pixel data in RGBX format
    m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
}
  

/// <summary>
/// Destructor
/// </summary>
CColorBasics::~CColorBasics()
{
    // clean up Direct2D renderer
    if (m_pDrawColor)
    {
        delete m_pDrawColor;
        m_pDrawColor = NULL;
    }

    if (m_pColorRGBX)
    {
        delete [] m_pColorRGBX;
        m_pColorRGBX = NULL;
    }

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with color frame reader
    SafeRelease(m_pColorFrameReader);
    SafeRelease(m_pDepthFrameReader);
    SafeRelease(m_pBodyFrameReader);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CColorBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"ColorBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CColorBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

	m_depthBuffer = new UINT16[cDepthHeight * cDepthWidth];
	m_joints = new Joint[JointType_Count];

    // Main message loop
    while (WM_QUIT != msg.message)
    {
        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CColorBasics::Update()
{
	UpdateColor();

	UINT16* depthBuffer = NULL;
	UINT capacity = 0;
 	int width = 0;
	int height = 0;

	DepthSpacePoint dsp = {0};

	if (m_bSaveScreenshot)
	{
		UpdateBody(&dsp);
		UpdateDepth(&capacity, &width, &height, dsp);

		Output("dsp: ", dsp.X, dsp.Y);

		Trianglez(capacity, width, height, dsp, 25);

		KevinsCode();

		WCHAR szStatusMessage[64 + MAX_PATH];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Saved files", NULL);

		SetStatusMessage(szStatusMessage, 5000, true);

		// toggle off so we don't save a screenshot again next frame
		m_bSaveScreenshot = false;
	}
}

void CColorBasics::UpdateColor()
{
    if (!m_pColorFrameReader)
    {
        return;
    }

    IColorFrame* pColorFrame = NULL;

    HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        ColorImageFormat imageFormat = ColorImageFormat_None;
        UINT nBufferSize = 0;
        RGBQUAD *pBuffer = NULL;

        hr = pColorFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr))
        {
            hr = pColorFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
        }

        if (SUCCEEDED(hr))
        {
            if (imageFormat == ColorImageFormat_Bgra)
            {
                hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
            }
            else if (m_pColorRGBX)
            {
                pBuffer = m_pColorRGBX;
                nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
                hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);            
            }
            else
            {
                hr = E_FAIL;
            }
        }

        if (SUCCEEDED(hr))
        {
            ProcessColor(nTime, pBuffer, nWidth, nHeight);
        }

        SafeRelease(pFrameDescription);
    }

    SafeRelease(pColorFrame);
} 

DepthSpacePoint CColorBasics::JointToDepthSpacePoint(JointType jointType) {
	DepthSpacePoint dsp = { 0 };
	Joint joint = m_joints[jointType];
	m_pCoordinateMapper->MapCameraPointToDepthSpace(joint.Position, &dsp);
	return dsp;
}

void CColorBasics::KevinsCode()
{
	time_t timehash = time(NULL);
	Mat rawImage, depthFile1, depthFile2, mappingFile, cannyEdgeImage;

	depthFile1 = Mat(cDepthHeight, cDepthWidth, DataType<UINT16>::type, m_depthBuffer);

	DepthSpacePoint dsp;

	// Neck
	dsp = JointToDepthSpacePoint(JointType_Neck);
	m_skeletalPoints.neck_x = (int) dsp.X;
	m_skeletalPoints.neck_y = (int) dsp.Y;
	Mat leftNeckRoi = depthFile1(
		Range(m_skeletalPoints.neck_y, m_skeletalPoints.neck_y + 1),
		Range(m_skeletalPoints.neck_x - 100, m_skeletalPoints.neck_x)
	);
	Point maxX;
	minMaxLoc(leftNeckRoi, NULL, NULL, NULL, &maxX);
	m_tracePoints.leftNeck = Point(m_skeletalPoints.neck_x - 100 + maxX.x, m_skeletalPoints.neck_y);
	// circle(rawImage, Point(x - 100 + leftX.x, y), 5, Scalar(0, 0, 255), FILLED, LINE_8);

	Mat rightNeckRoi = depthFile1(
		Range(m_skeletalPoints.neck_y, m_skeletalPoints.neck_y + 1),
		Range(m_skeletalPoints.neck_x, m_skeletalPoints.neck_x + 100)
	);
	Point minX;
	minMaxLoc(rightNeckRoi, NULL, NULL, &minX, NULL);
	m_tracePoints.rightNeck = Point(m_skeletalPoints.neck_x + minX.x, m_skeletalPoints.neck_y);
	circle(rawImage, Point(m_tracePoints.rightNeck.x, m_tracePoints.rightNeck.y), 5, Scalar(0, 0, 255), FILLED, LINE_8);


	// Shoulder Left
	dsp = JointToDepthSpacePoint(JointType_ShoulderLeft);
	m_skeletalPoints.leftShoulder_x = (int) dsp.X;
	m_skeletalPoints.leftShoulder_y = (int) dsp.Y;
	//rectangle(rawImage, Point(x, y), Point(x - 50, y - 50), Scalar(255, 0, 0), 2);
	Mat leftShoulderRoi = depthFile1(
		Range(m_skeletalPoints.leftShoulder_y - 50, m_skeletalPoints.leftShoulder_y),
		Range(m_skeletalPoints.leftShoulder_x - 50, m_skeletalPoints.leftShoulder_x)
);
	int diagonalArr[50];
	// construct diagonal array
	for (int i = 0; i < 50; i++) {
		Scalar intensity = leftShoulderRoi.at<uchar>(i, i);
		diagonalArr[i] = intensity.val[0];
	}
	Mat diagonalRoi = Mat(1, 50, DataType<int>::type, &diagonalArr);
	Point maxPoint;
	minMaxLoc(diagonalRoi, NULL, NULL, NULL, &maxPoint);
	//circle(rawImage, Point(x - 50 + maxPoint.x, y - 50 + maxPoint.x), 5, Scalar(0, 0, 255), FILLED, LINE_8);
	m_tracePoints.leftShoulder = Point(
		m_skeletalPoints.leftShoulder_x - 50 + maxPoint.x,
		m_skeletalPoints.leftShoulder_y - 50 + maxPoint.x
	);

	// Right Shoulder
	dsp = JointToDepthSpacePoint(JointType_ShoulderRight);
	m_skeletalPoints.rightShoulder_x = (int) dsp.X;
	m_skeletalPoints.rightShoulder_y = (int) dsp.Y;
	//rectangle(rawImage, Point(x, y), Point(x + 50, y - 50), Scalar(255, 0, 0), 2);
	Mat rightShoulderRoi = depthFile1(
		Range(m_skeletalPoints.rightShoulder_y - 50, m_skeletalPoints.rightShoulder_y),
		Range(m_skeletalPoints.rightShoulder_x, m_skeletalPoints.rightShoulder_x + 50)
	);
	int diagonalArr[50];
	// construct diagonal array
	for (int i = 0; i < 50; i++) {
		Scalar intensity = rightShoulderRoi.at<uchar>(i, 50 - i);
		diagonalArr[i] = intensity.val[0];
	}
	Mat diagonalRoi = Mat(1, 50, DataType<int>::type, &diagonalArr);
	Point maxPoint;
	minMaxLoc(diagonalRoi, NULL, NULL, NULL, &maxPoint);
	// circle(rawImage, Point(x + 50 - maxPoint.x, y - 50 + maxPoint.x), 5, Scalar(0, 0, 255), FILLED, LINE_8);
	m_tracePoints.rightShoulder = Point(
		m_skeletalPoints.rightShoulder_x + 50 - maxPoint.x,
		m_skeletalPoints.rightShoulder_y - 50 + maxPoint.x
	);

	// Left Hip
	dsp = JointToDepthSpacePoint(JointType_HipLeft);
	m_skeletalPoints.leftHip_x = (int) dsp.X;
	m_skeletalPoints.leftHip_y = (int) dsp.Y;
	Mat leftHipRoi = depthFile1(
		Range(m_skeletalPoints.leftHip_y, m_skeletalPoints.leftHip_y + 1),
		Range(m_skeletalPoints.leftHip_x - 150, m_skeletalPoints.leftHip_x - 50)
	);
	Point maxX;
	minMaxLoc(leftHipRoi, NULL, NULL, NULL, &maxX);
	m_tracePoints.leftHip = Point(
		m_skeletalPoints.leftHip_x - 150 + maxX.x,
		m_skeletalPoints.leftHip_y
	);
	// circle(rawImage, Point(x - 125 + leftX.x, y), 5, Scalar(0, 0, 255), FILLED, LINE_8);


	// Right Hip
	dsp = JointToDepthSpacePoint(JointType_HipLeft);
	m_skeletalPoints.rightHip_x = (int) dsp.X;
	m_skeletalPoints.rightHip_y = (int) dsp.Y;
	Mat rightHipRoi = depthFile1(
		Range(m_skeletalPoints.rightHip_y, m_skeletalPoints.rightHip_y + 1),
		Range(m_skeletalPoints.rightHip_x + 50, m_skeletalPoints.rightHip_x + 150)
	);
	Point minX;
	minMaxLoc(rightHipRoi, NULL, NULL, &minX, NULL);
	m_tracePoints.rightHip = Point(
		m_skeletalPoints.rightHip_x + 50 + minX.x,
		m_skeletalPoints.rightHip_y
	);
	//circle(rawImage, Point(x + 50 + rightX.x, y), 5, Scalar(0, 0, 255), FILLED, LINE_8);

	dsp = JointToDepthSpacePoint(JointType_ElbowLeft);
	m_skeletalPoints.leftElbow_x = (int) dsp.X;
	m_skeletalPoints.leftElbow_y = (int) dsp.Y;

	dsp = JointToDepthSpacePoint(JointType_ElbowRight);
	m_skeletalPoints.rightElbow_x = (int) dsp.X;
	m_skeletalPoints.rightElbow_y = (int) dsp.Y;

	// Left Hem
	Point leftBicep = Point(
		(m_skeletalPoints.leftElbow_x + m_skeletalPoints.leftShoulder_x) / 2,
		(m_skeletalPoints.leftElbow_y + m_skeletalPoints.leftShoulder_y) / 2
	);
	Mat leftHemRoi = depthFile1(
		Range(leftBicep.y, leftBicep.y + 1),
		Range(leftBicep.x - 100, leftBicep.x)
	);
	Point maxPoint;
	minMaxLoc(leftHemRoi, NULL, NULL, NULL, &maxPoint);
	m_tracePoints.leftOuterHem = Point(leftBicep.x - 100 + maxPoint.x, leftBicep.y);
	m_tracePoints.leftInnerHem = Point(leftBicep.x + 100 - maxPoint.x, leftBicep.y);
	// Left Outer Hem
	// circle(rawImage, Point(leftBicep.x - 100 + leftX.x, leftBicep.y), 5, Scalar(0, 0, 255), FILLED, LINE_8);
	// Left Inner Hem
	// circle(rawImage, Point(leftBicep.x + 100 - leftX.x, leftBicep.y), 5, Scalar(0, 0, 255), FILLED, LINE_8);

	// Right Hem
	Point rightBicep = Point(
		(m_skeletalPoints.rightElbow_x + m_skeletalPoints.rightShoulder_x) / 2,
		(m_skeletalPoints.rightElbow_y + m_skeletalPoints.rightShoulder_y) / 2
	);
	Mat rightHemRoi = depthFile1(
		Range(rightBicep.y, rightBicep.y + 1),
		Range(rightBicep.x , rightBicep.x + 100)
	);
	Point minPoint;
	minMaxLoc(rightHemRoi, NULL, NULL, &minPoint, NULL);
	m_tracePoints.rightOuterHem = Point(rightBicep.x + minPoint.x, rightBicep.y);
	m_tracePoints.rightInnerHem = Point(rightBicep.x - minPoint.x, rightBicep.y);

	Mat rightHemRoi = depthFile1(Range(rightBicep.y, rightBicep.y + 1), Range(rightBicep.x, rightBicep.x + 100));
	Point rightX;
	minMaxLoc(rightHemRoi, NULL, NULL, NULL, &rightX);

	// resize(cannyEdgeImage, cannyEdgeImage, Size(), 0.5, 0.5, INTER_AREA);
	// resize(rawImage, rawImage, Size(), 0.5, 0.5, INTER_AREA);

	namedWindow("Canny Edge", WINDOW_AUTOSIZE);
	imshow("Canny Edge", cannyEdgeImage );
		namedWindow("Raw Image", WINDOW_AUTOSIZE);
	imshow("Raw Image", rawImage);
	waitKey(0);
}

void CColorBasics::UpdateDepth(UINT* capacity, int* width, int* height, DepthSpacePoint dsp)
{
    if (!m_pDepthFrameReader)
    {
        return;
    }

    IDepthFrame* pDepthFrame = NULL;

    HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
		UINT nBytesPerPixel = 0;
        USHORT nDepthMinReliableDistance = 0;
        USHORT nDepthMaxDistance = 0;
        UINT nBufferSize = 0;
        //UINT16 *pBuffer = NULL;

        hr = pDepthFrame->get_RelativeTime(&nTime);

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Width(&nWidth);
			*width = nWidth;
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
			*height = nHeight;
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_BytesPerPixel(&nBytesPerPixel);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
        }

        if (SUCCEEDED(hr))
        {
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			//nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
            hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
        }

        if (SUCCEEDED(hr))
        {
			UINT16* underlyingBuffer = NULL;
            hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &underlyingBuffer);            
			*capacity = nBufferSize;

			for (int i = 0; i < nBufferSize; i++) {
				UINT16 value = underlyingBuffer[i];
				value = (UINT16)((value >= nDepthMinReliableDistance && value <= nDepthMaxDistance) ? value : 0);
				m_depthBuffer[i] = value;
			}
        }

        if (SUCCEEDED(hr))
        {
			string OUTPUT_DIRECTORY = "C:\\Users\\Ryan\\~\\code\\fydp-kinect-app\\out\\depth";
			string FILE_EXTENSION = "-bad.pgm";
			time_t timestamp = time(nullptr);
			char* filepath = new char[OUTPUT_DIRECTORY.length() + FILE_EXTENSION.length() + 32];
			sprintf(filepath, "%s-%d%s", OUTPUT_DIRECTORY.c_str(), (int)timestamp, FILE_EXTENSION.c_str());
			ofstream myfile(filepath, ios::out | ios::binary);

			char* header = new char[200];
			sprintf(header, "P5 %d %d %d\n", nWidth, nHeight, nDepthMaxDistance);
			myfile << header;

			// nBufferSize contains length of array rather than SIZE of array in bytes
			for (int i = 0; i < nBufferSize; i++) {
				myfile.write((char *) &m_depthBuffer[i], sizeof(UINT16));
			}

			myfile.close();


			// DONT DEAD OPEN INSIDE
			// ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
        }

        SafeRelease(pFrameDescription);
    }

    SafeRelease(pDepthFrame);
}

void CColorBasics::UpdateBody(DepthSpacePoint *dsp) 
{
    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;

        hr = pBodyFrame->get_RelativeTime(&nTime);

        IBody* ppBodies[BODY_COUNT] = {0};

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {
            // ProcessBody(nTime, BODY_COUNT, ppBodies);

            for (int i = 0; i < BODY_COUNT; ++i)
            {
                IBody* pBody = ppBodies[i];
				if (!pBody)
				{
					continue;
				}

				BOOLEAN bTracked = false;
				hr = pBody->get_IsTracked(&bTracked);

				if (!SUCCEEDED(hr) || !bTracked)
				{
					continue;
				}

				Output("Found at least one body and it is tracked");

				Joint joints[JointType_Count]; 

				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr))
				{
					for (int j = 0; j < _countof(joints); ++j)
					{
						Joint joint = joints[i];

						m_joints[i] = joints[j];

						if (j == 0) {
							m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[j].Position, dsp);
						}

						DepthSpacePoint depthPoint = {0};
						m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[j].Position, &depthPoint);
						Output("X: %f, Y: %f, Type: %d\n", depthPoint.X, depthPoint.Y, static_cast<underlying_type<JointType>::type>(joints[j].JointType));
					}
				}
            }
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }

    SafeRelease(pBodyFrame);
	// Do nothing
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CColorBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CColorBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CColorBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CColorBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CColorBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawColor = new ImageRenderer();
            HRESULT hr = m_pDrawColor->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD)); 
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND:
            // If it was for the screenshot control and a button clicked event, save a screenshot next frame 
            if (IDC_BUTTON_SCREENSHOT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
            {
                m_bSaveScreenshot = true;
            }
            break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CColorBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get the color reader
        IColorFrameSource* pColorFrameSource = NULL;
		IDepthFrameSource* pDepthFrameSource = NULL;
		IBodyFrameSource* pBodyFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

		// Frame Sources
        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

		// Readers
        if (SUCCEEDED(hr))
        {
            hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
        }

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}

        SafeRelease(pColorFrameSource);
		SafeRelease(pDepthFrameSource);
		SafeRelease(pBodyFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new color data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// </summary>
void CColorBasics::ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight) 
{
    if (m_hWnd)
    {
        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
    }

    // Make sure we've received valid data
    if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
    {
        // Draw the data with Direct2D
        m_pDrawColor->Draw(reinterpret_cast<BYTE*>(pBuffer), cColorWidth * cColorHeight * sizeof(RGBQUAD));

    }
}

/// <summary>
/// Handle new depth data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// <param name="nMinDepth">minimum reliable depth</param>
/// <param name="nMaxDepth">maximum reliable depth</param>
/// </summary>
//void CColorBasics::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
//{
//	if (m_hWnd)
//	{
//		if (!m_nStartTime)
//		{
//			m_nStartTime = nTime;
//		}
//
//		double fps = 0.0;
//
//		LARGE_INTEGER qpcNow = { 0 };
//		if (m_fFreq)
//		{
//			if (QueryPerformanceCounter(&qpcNow))
//			{
//				if (m_nLastCounter)
//				{
//					m_nFramesSinceUpdate++;
//					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
//				}
//			}
//		}
//
//		WCHAR szStatusMessage[64];
//		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));
//
//		if (SetStatusMessage(szStatusMessage, 1000, false))
//		{
//			m_nLastCounter = qpcNow.QuadPart;
//			m_nFramesSinceUpdate = 0;
//		}
//	}
//
//	// Make sure we've received valid data
//	if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
//	{
//		RGBQUAD* pRGBX = m_pDepthRGBX;
//
//		// end pixel is start + width*height - 1
//		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
//
//		while (pBuffer < pBufferEnd)
//		{
//			USHORT depth = *pBuffer;
//
//			// To convert to a byte, we're discarding the most-significant
//			// rather than least-significant bits.
//			// We're preserving detail, although the intensity will "wrap."
//			// Values outside the reliable depth range are mapped to 0 (black).
//
//			// Note: Using conditionals in this loop could degrade performance.
//			// Consider using a lookup table instead when writing production code.
//			BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);
//
//			pRGBX->rgbRed = intensity;
//			pRGBX->rgbGreen = intensity;
//			pRGBX->rgbBlue = intensity;
//
//			++pRGBX;
//			++pBuffer;
//		}
//
//		// Draw the data with Direct2D
//		m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));
//
//		if (m_bSaveScreenshot)
//		{
//			WCHAR szScreenshotPath[MAX_PATH];
//
//			// Retrieve the path to My Photos
//			GetScreenshotFileName(szScreenshotPath, _countof(szScreenshotPath));
//
//			// Write out the bitmap to disk
//			HRESULT hr = SaveBitmapToFile(reinterpret_cast<BYTE*>(m_pDepthRGBX), nWidth, nHeight, sizeof(RGBQUAD) * 8, szScreenshotPath);
//
//			WCHAR szStatusMessage[64 + MAX_PATH];
//			if (SUCCEEDED(hr))
//			{
//				// Set the status bar to show where the screenshot was saved
//				StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Screenshot saved to %s", szScreenshotPath);
//			}
//			else
//			{
//				StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Failed to write screenshot to %s", szScreenshotPath);
//			}
//
//			SetStatusMessage(szStatusMessage, 5000, true);
//
//			// toggle off so we don't save a screenshot again next frame
//			m_bSaveScreenshot = false;
//		}
//	}
//}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CColorBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Get the name of the file where screenshot will be stored.
/// </summary>
/// <param name="lpszFilePath">string buffer that will receive screenshot file name.</param>
/// <param name="nFilePathSize">number of characters in lpszFilePath string buffer.</param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT CColorBasics::GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize)
{
    WCHAR* pszKnownPath = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &pszKnownPath);

    if (SUCCEEDED(hr))
    {
        // Get the time
        WCHAR szTimeString[MAX_PATH];
        GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", szTimeString, _countof(szTimeString));

        // File name will be KinectScreenshotColor-HH-MM-SS.bmp
        StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\KinectScreenshot-Color-%s.bmp", pszKnownPath, szTimeString);
    }

    if (pszKnownPath)
    {
        CoTaskMemFree(pszKnownPath);
    }

    return hr;
}

/// <summary>
/// Save passed in image data to disk as a bitmap
/// </summary>
/// <param name="pBitmapBits">image data to save</param>
/// <param name="lWidth">width (in pixels) of input image data</param>
/// <param name="lHeight">height (in pixels) of input image data</param>
/// <param name="wBitsPerPixel">bits per pixel of image data</param>
/// <param name="lpszFilePath">full file path to output bitmap to</param>
/// <returns>indicates success or failure</returns>
HRESULT CColorBasics::SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
    DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

    BITMAPINFOHEADER bmpInfoHeader = {0};

    bmpInfoHeader.biSize        = sizeof(BITMAPINFOHEADER);  // Size of the header
    bmpInfoHeader.biBitCount    = wBitsPerPixel;             // Bit count
    bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
    bmpInfoHeader.biWidth       = lWidth;                    // Width in pixels
    bmpInfoHeader.biHeight      = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
    bmpInfoHeader.biPlanes      = 1;                         // Default
    bmpInfoHeader.biSizeImage   = dwByteCount;               // Image size in bytes

    BITMAPFILEHEADER bfh = {0};

    bfh.bfType    = 0x4D42;                                           // 'M''B', indicates bitmap
    bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
    bfh.bfSize    = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

    // Create the file on disk to write to
    HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    // Return if error opening file
    if (NULL == hFile) 
    {
        return E_ACCESSDENIED;
    }

    DWORD dwBytesWritten = 0;
    
    // Write the bitmap file header
    if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the bitmap info header
    if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the RGB Data
    if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }    

    // Close the file
    CloseHandle(hFile);
    return S_OK;
}