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
#include <vector>
#include <unordered_map>

#include <iostream>

#define RED Scalar(0, 0, USHRT_MAX)
#define BLUE Scalar(USHRT_MAX, 0, 0)
#define GREEN Scalar(0, USHRT_MAX, 0)

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

UINT16 CColorBasics::dGrid(int y, int x)
{
	return m_depthBuffer[(y * cDepthWidth) + x];
}

void CColorBasics::DisjointEdgeDetection(DepthSpacePoint dsp)
{
	short threshold = 25;
	DisjointSet ds(cDepthHeight * cDepthWidth);

    for (int i = 1; i < cDepthWidth; i++) {
        if (abs(dGrid(0, i) - dGrid(0, i-1)) < threshold) {
            ds.do_union(i, i-1);
        }
    }

    for (int i = 1; i < cDepthHeight; i++) {
        if (abs(dGrid(i, 0) - dGrid(i-1, 0)) < threshold) {
            ds.do_union(i*cDepthWidth, (i-1)*cDepthWidth);
        }
    }
    
	for (int i = 1; i < cDepthHeight; i++) {
        for (int j = 1; j < cDepthWidth; j++) {
			if (abs(dGrid(i, j) - dGrid(i-1, j)) < threshold) {
                ds.do_union(i*cDepthWidth + j, (i-1)*cDepthWidth+j);
            }
            if (abs(dGrid(i, j-1) - dGrid(i, j)) < threshold) {
                ds.do_union(i*cDepthWidth + j, i*cDepthWidth+j-1);
            }
        }
    }

	int personComponent = ds.find(cDepthWidth * ((int) (dsp.Y)) + (int) (dsp.X));
	Output("personComponent: %d\n", personComponent);

	for (int i = 0; i < cDepthHeight; i++) {
		for (int j = 0; j < cDepthWidth; j++) {
			int num = ds.find(cDepthWidth * i + j);
			if (num == personComponent) {
				m_depthBuffer[i * cDepthWidth + j] = USHRT_MAX;
			} else {
				char zero = 0;
				m_depthBuffer[i * cDepthWidth + j] = 0;
			}
		}
	}
}

void CColorBasics::MapTriangle(vector<Point> &source_t, vector<Point> &destination_t) {
	// Find bounding rectangle for each triangle
	Rect source_rect = boundingRect(source_t);
	Rect destination_rect = boundingRect(destination_t);

	// Offset points by left top corner of the respective rectangles
	Point2f source_triangle_bounded[3];
	Point2f destination_triangle_bounded[3];
	Point destination_triangle_bounded_int[3];

	for (int i = 0; i < 3; i++) {
		source_triangle_bounded[i] = Point2f(source_t[i].x - source_rect.x, source_t[i].y - source_rect.y);
		destination_triangle_bounded[i] = Point2f(destination_t[i].x - destination_rect.x, destination_t[i].y - destination_rect.y);
		destination_triangle_bounded_int[i] = Point(destination_t[i].x - destination_rect.x, destination_t[i].y - destination_rect.y);
	}

	//	Get mask by filling triangle
	//Mat mask = Mat::zeros(destination_rect.height, destination_rect.width, CV_32FC4);
	//cv::fillConvexPoly(mask, destination_triangle_bounded_int, 3, Scalar(1.0, 1.0, 1.0, 1.0));

	//	Apply warpImage to small rectangular patches
	Mat source_crop;
	m_clothingImage(source_rect).copyTo(source_crop);

	Mat dest_crop = Mat::zeros(destination_rect.height, destination_rect.width, source_crop.type());

	// Given a pair of triangles, find the affine transform.
	Mat warp_mat = getAffineTransform(source_triangle_bounded, destination_triangle_bounded);

	// Apply the Affine Transform just found to the src image
	cv::warpAffine(source_crop, dest_crop, warp_mat, dest_crop.size());

	Mat alpha_mask;
	cv::extractChannel(dest_crop, alpha_mask, 3);
	cvtColor(alpha_mask, alpha_mask, COLOR_GRAY2BGRA);
	//cv::multiply(mask, alpha_mask, mask);

	cv::multiply(dest_crop, alpha_mask, dest_crop);
	cv::multiply(m_personImage(destination_rect), Scalar(1.0, 1.0, 1.0, 1.0) - alpha_mask, m_personImage(destination_rect));
	m_personImage(destination_rect) = m_personImage(destination_rect) + dest_crop;

}

void CColorBasics::Output(const char* szFormat, ...)
{
	char szBuff[1024];
	va_list arg;
	va_start(arg, szFormat);
	_vsnprintf(szBuff, sizeof(szBuff), szFormat, arg);
	va_end(arg);

	OutputDebugStringA(szBuff);
	OutputDebugStringA("\n");
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
	m_colorBuffer = new RGBQUAD[cColorHeight * cColorWidth];

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

void CColorBasics::LoadBinaryData() {
	FILE *file;

	file = fopen("C:\\Users\\Ryan\\~\\code\\fydp-2\\bin_dumps\\color.dump", "rb");
	fread(m_colorBuffer, sizeof(RGBQUAD), cColorWidth * cColorHeight, file);
	fclose(file);

	file = fopen("C:\\Users\\Ryan\\~\\code\\fydp-2\\bin_dumps\\depth.dump", "rb");
	fread(m_depthBuffer, sizeof(USHORT), cDepthWidth * cDepthHeight, file);
	fclose(file);

	file = fopen("C:\\Users\\Ryan\\~\\code\\fydp-2\\bin_dumps\\bonez.dump", "rb");
	fread(m_joints, sizeof(Joint), JointType_Count, file);
	fclose(file);
}

void CColorBasics::StoreBinaryData() {
	FILE *file;

	file = fopen("C:\\Users\\Ryan\\~\\code\\fydp-2\\bin_dumps\\color.dump", "wb");
	fwrite(m_colorBuffer, sizeof(RGBQUAD), cColorWidth * cColorHeight, file);
	fclose(file);

	file = fopen("C:\\Users\\Ryan\\~\\code\\fydp-2\\bin_dumps\\depth.dump", "wb");
	fwrite(m_depthBuffer, sizeof(USHORT), cDepthWidth * cDepthHeight, file);
	fclose(file);

	file = fopen("C:\\Users\\Ryan\\~\\code\\fydp-2\\bin_dumps\\bonez.dump", "wb");
	fwrite(m_joints, sizeof(Joint), JointType_Count, file);
	fclose(file);
}

//Read points from text file
vector<Point> CColorBasics::readClothingPoints(string filename) {
	vector<Point> points;
	ifstream ifs(filename.c_str());
	float x, y;
	int count = 0;
	while (ifs >> x >> y) {
		points.push_back(Point(x, y));
	}

	return points;
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

	DepthSpacePoint dspHipJoint = { 0 };

	bool loadBinaryData = true;
	bool storeBinaryData = false;

	if (loadBinaryData && !m_ranOnceAlready) {
		LoadBinaryData();
	}

	if (m_bSaveScreenshot || (loadBinaryData && !m_ranOnceAlready))
	{
		if (!loadBinaryData) {
			UpdateBody();
			UpdateDepth(&capacity, &width, &height);
		}

		if (storeBinaryData) {
			StoreBinaryData();
		}

		m_pCoordinateMapper->MapCameraPointToDepthSpace(m_joints[0].Position, &dspHipJoint);
		if (isinf(dspHipJoint.X) || isinf(dspHipJoint.Y)) {
			Output("Kinect not wok??");
			return;
		}

		DisjointEdgeDetection(dspHipJoint);
		vector<Point> personPoints = LandmarkRecognition();
		ApplyClothing(personPoints);

		WCHAR szStatusMessage[64 + MAX_PATH];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Saved files", NULL);
		SetStatusMessage(szStatusMessage, 5000, true);

		// toggle off so we don't save a screenshot again next frame
		m_bSaveScreenshot = false;
		m_ranOnceAlready = true;
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
                //hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);            
                pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(m_colorBuffer), ColorImageFormat_Bgra);            
            }
            else
            {
                hr = E_FAIL;
            }
        }

        if (SUCCEEDED(hr))
        {
            ProcessColor(nTime, m_colorBuffer, nWidth, nHeight);
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

ColorSpacePoint CColorBasics::DepthSpaceToColorSpace(int x, int y) {
	DepthSpacePoint dsp = { 0 };
	ColorSpacePoint csp = { 0 };
	dsp.X = (float) x;
	dsp.Y = (float) y;
	m_pCoordinateMapper->MapDepthPointToColorSpace(dsp, USHRT_MAX, &csp);
	return csp;
}

Point CColorBasics::GetOffsetForJoint(Joint joint) {
	ColorSpacePoint csp_control = { 0 };
	ColorSpacePoint csp = { 0 };
	DepthSpacePoint dsp = { 0 };

	m_pCoordinateMapper->MapCameraPointToColorSpace(joint.Position, &csp_control);

	m_pCoordinateMapper->MapCameraPointToDepthSpace(joint.Position, &dsp);
	m_pCoordinateMapper->MapDepthPointToColorSpace(dsp, dGrid((int) dsp.Y, (int) dsp.X), &csp);
	return Point(csp_control.X - csp.X, csp_control.Y - csp.Y);
}

vector<Point> CColorBasics::LandmarkRecognition()
{
	time_t timehash = time(NULL);
	Mat matDepth, matDepthRaw, matDepthColor;

	vector<Point> points(10);
	ColorSpacePoint csp;
	DepthSpacePoint dsp;
	Point maxPoint;
	Point minPoint;
	Point offset;
	int diagonalArr[50];
	Mat diagonalRoi = Mat(1, 50, DataType<int>::type, &diagonalArr);

	// depthFile1 = Mat(cDepthHeight, cDepthWidth, DataType<UINT16>::type, m_depthBuffer, sizeof(UINT16) * cDepthWidth);
	matDepthRaw = Mat(cDepthHeight, cDepthWidth, CV_16UC1, m_depthBuffer, sizeof(UINT16) * cDepthWidth);
	cvtColor(matDepthRaw, matDepth, COLOR_GRAY2BGR);

	matDepthColor = Mat(cColorHeight, cColorWidth, CV_8UC4, m_colorBuffer, sizeof(RGBQUAD) * cColorWidth);

	// Neck
	dsp = JointToDepthSpacePoint(JointType_Neck);
	m_skeletalPoints.neck_x = (int) dsp.X;
	m_skeletalPoints.neck_y = (int) dsp.Y;
	Mat leftNeckRoi = matDepthRaw(
		Range(m_skeletalPoints.neck_y, m_skeletalPoints.neck_y + 1),
		Range(m_skeletalPoints.neck_x - 100, m_skeletalPoints.neck_x)
	);
	Point maxX;
	minMaxLoc(leftNeckRoi, NULL, NULL, NULL, &maxX);
	m_tracePoints.leftNeck = Point(m_skeletalPoints.neck_x - 100 + maxX.x, m_skeletalPoints.neck_y);
	csp = DepthSpaceToColorSpace(m_tracePoints.leftNeck.x, m_tracePoints.leftNeck.y);
	offset = GetOffsetForJoint(m_joints[JointType_Neck]);
	points[0] = Point(csp.X, csp.Y) + offset;
	circle(matDepthColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(matDepth, Point(m_tracePoints.leftNeck.x, m_tracePoints.leftNeck.y), 5, BLUE, FILLED, LINE_8);

	Mat rightNeckRoi = matDepthRaw(
		Range(m_skeletalPoints.neck_y, m_skeletalPoints.neck_y + 1),
		Range(m_skeletalPoints.neck_x, m_skeletalPoints.neck_x + 100)
	);
	Point minX;
	minMaxLoc(rightNeckRoi, NULL, NULL, &minX, NULL);
	m_tracePoints.rightNeck = Point(m_skeletalPoints.neck_x + minX.x, m_skeletalPoints.neck_y);
	csp = DepthSpaceToColorSpace(m_tracePoints.rightNeck.x, m_tracePoints.rightNeck.y);
	points[1] = Point(csp.X, csp.Y) + offset;
	circle(matDepthColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(matDepth, Point(m_tracePoints.rightNeck.x, m_tracePoints.rightNeck.y), 5, BLUE, FILLED, LINE_8);

	// Shoulder Left
	dsp = JointToDepthSpacePoint(JointType_ShoulderLeft);
	m_skeletalPoints.leftShoulder_x = (int) dsp.X;
	m_skeletalPoints.leftShoulder_y = (int) dsp.Y;
	//rectangle(rawImage, Point(x, y), Point(x - 50, y - 50), Scalar(255, 0, 0), 2);
	Mat leftShoulderRoi = matDepthRaw(
		Range(m_skeletalPoints.leftShoulder_y - 50, m_skeletalPoints.leftShoulder_y),
		Range(m_skeletalPoints.leftShoulder_x - 50, m_skeletalPoints.leftShoulder_x)
	);
	// construct diagonal array
	for (int i = 0; i < 50; i++) {
		Scalar intensity = leftShoulderRoi.at<UINT16>(i, i);
		diagonalArr[i] = intensity.val[0];
	}
	minMaxLoc(diagonalRoi, NULL, NULL, NULL, &maxPoint);
	m_tracePoints.leftShoulder = Point(
		m_skeletalPoints.leftShoulder_x - 50 + maxPoint.x,
		m_skeletalPoints.leftShoulder_y - 50 + maxPoint.x
	);
	csp = DepthSpaceToColorSpace(m_tracePoints.leftShoulder.x, m_tracePoints.leftShoulder.y);
	offset = GetOffsetForJoint(m_joints[JointType_ShoulderLeft]);
	points[2] = Point(csp.X, csp.Y) + offset;
	circle(matDepthColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(matDepth, Point(m_tracePoints.leftShoulder.x, m_tracePoints.leftShoulder.y), 5, BLUE, FILLED, LINE_8);

	// Right Shoulder
	dsp = JointToDepthSpacePoint(JointType_ShoulderRight);
	m_skeletalPoints.rightShoulder_x = (int) dsp.X;
	m_skeletalPoints.rightShoulder_y = (int) dsp.Y;
	//rectangle(rawImage, Point(x, y), Point(x + 50, y - 50), Scalar(255, 0, 0), 2);
	Mat rightShoulderRoi = matDepthRaw(
		Range(m_skeletalPoints.rightShoulder_y - 50, m_skeletalPoints.rightShoulder_y),
		Range(m_skeletalPoints.rightShoulder_x, m_skeletalPoints.rightShoulder_x + 50)
	);

	// construct diagonal array
	for (int i = 0; i < 50; i++) {
		Scalar intensity = rightShoulderRoi.at<UINT16>(i, 49 - i);
		diagonalArr[i] = intensity.val[0];
	}
	minMaxLoc(diagonalRoi, NULL, NULL, NULL, &maxPoint);
	m_tracePoints.rightShoulder = Point(
		m_skeletalPoints.rightShoulder_x + 50 - maxPoint.x,
		m_skeletalPoints.rightShoulder_y - 50 + maxPoint.x
	);
	csp = DepthSpaceToColorSpace(m_tracePoints.rightShoulder.x, m_tracePoints.rightShoulder.y);
	offset = GetOffsetForJoint(m_joints[JointType_ShoulderRight]);
	points[3] = Point(csp.X, csp.Y) + offset;
	circle(matDepthColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(matDepth, Point(m_tracePoints.rightShoulder.x, m_tracePoints.rightShoulder.y), 5, BLUE, FILLED, LINE_8);

	// Left Hip
	dsp = JointToDepthSpacePoint(JointType_HipLeft);
	m_skeletalPoints.leftHip_x = (int) dsp.X;
	m_skeletalPoints.leftHip_y = (int) dsp.Y;
	for (int i = m_skeletalPoints.leftHip_x; i >= 0; i--) {
		if (matDepthRaw.at<USHORT>(m_skeletalPoints.leftHip_y, i) == 0) {
			m_tracePoints.leftHip = Point(i, m_skeletalPoints.leftHip_y);
			break;
		}
	}
	csp = DepthSpaceToColorSpace(m_tracePoints.leftHip.x, m_tracePoints.leftHip.y);
	offset = GetOffsetForJoint(m_joints[JointType_HipLeft]);
	points[8] = Point(csp.X, csp.Y) + offset;
	circle(matDepthColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(matDepth, Point(m_tracePoints.leftHip.x, m_tracePoints.leftHip.y), 5, BLUE, FILLED, LINE_8);

	// Right Hip
	dsp = JointToDepthSpacePoint(JointType_HipLeft);
	m_skeletalPoints.rightHip_x = (int) dsp.X;
	m_skeletalPoints.rightHip_y = (int) dsp.Y;
	for (int i = m_skeletalPoints.rightHip_x; i < cDepthWidth; i++) {
		if (matDepthRaw.at<USHORT>(m_skeletalPoints.rightHip_y, i) == 0) {
			m_tracePoints.rightHip = Point(i, m_skeletalPoints.rightHip_y);
			break;
		}
	}
	csp = DepthSpaceToColorSpace(m_tracePoints.rightHip.x, m_tracePoints.rightHip.y);
	offset = GetOffsetForJoint(m_joints[JointType_HipRight]);
	points[9] = Point(csp.X, csp.Y) + offset;
	circle(matDepthColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(matDepth, Point(m_tracePoints.rightHip.x, m_tracePoints.rightHip.y), 5, BLUE, FILLED, LINE_8);

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
	Mat leftHemRoi = matDepthRaw(
		Range(leftBicep.y, leftBicep.y + 1),
		Range(leftBicep.x - 100, leftBicep.x)
	);
	minMaxLoc(leftHemRoi, NULL, NULL, NULL, &maxPoint);
	m_tracePoints.leftOuterHem = Point(leftBicep.x - 100 + maxPoint.x, leftBicep.y);
	m_tracePoints.leftInnerHem = Point(leftBicep.x + 100 - maxPoint.x, leftBicep.y);
	csp = DepthSpaceToColorSpace(m_tracePoints.leftOuterHem.x, m_tracePoints.leftOuterHem.y);
	offset = GetOffsetForJoint(m_joints[JointType_ElbowLeft]);
	points[4] = Point(csp.X, csp.Y) + offset;
	circle(matDepthColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(matDepth, Point(m_tracePoints.leftOuterHem.x, m_tracePoints.leftOuterHem.y), 5, BLUE, FILLED, LINE_8);

	csp = DepthSpaceToColorSpace(m_tracePoints.leftInnerHem.x, m_tracePoints.leftInnerHem.y);
	points[6] = Point(csp.X, csp.Y) + offset;
	circle(matDepthColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(matDepth, Point(m_tracePoints.leftInnerHem.x, m_tracePoints.leftInnerHem.y), 5, BLUE, FILLED, LINE_8);

	// Right Hem
	Point rightBicep = Point(
		(m_skeletalPoints.rightElbow_x + m_skeletalPoints.rightShoulder_x) / 2,
		(m_skeletalPoints.rightElbow_y + m_skeletalPoints.rightShoulder_y) / 2
	);
	Mat rightHemRoi = matDepthRaw(
		Range(rightBicep.y, rightBicep.y + 1),
		Range(rightBicep.x , rightBicep.x + 100)
	);
	minMaxLoc(rightHemRoi, NULL, NULL, &minPoint, NULL);
	m_tracePoints.rightOuterHem = Point(rightBicep.x + minPoint.x, rightBicep.y);
	m_tracePoints.rightInnerHem = Point(rightBicep.x - minPoint.x, rightBicep.y);
	csp = DepthSpaceToColorSpace(m_tracePoints.rightOuterHem.x, m_tracePoints.rightOuterHem.y);
	offset = GetOffsetForJoint(m_joints[JointType_ElbowRight]);
	points[5] = Point(csp.X, csp.Y) + offset;
	circle(matDepthColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(matDepth, Point(m_tracePoints.rightOuterHem.x, m_tracePoints.rightOuterHem.y), 5, BLUE, FILLED, LINE_8);

	csp = DepthSpaceToColorSpace(m_tracePoints.rightInnerHem.x, m_tracePoints.rightInnerHem.y);
	points[7] = Point(csp.X, csp.Y) + offset;
	circle(matDepthColor, Point(csp.X, csp.Y) + offset, 5, BLUE, FILLED, LINE_8);
	circle(matDepth, Point(m_tracePoints.rightInnerHem.x, m_tracePoints.rightInnerHem.y), 5, BLUE, FILLED, LINE_8);

	//namedWindow("depthFile Color", WINDOW_NORMAL);
	//imshow("depthFile Color", matDepthColor);

	//namedWindow("depthFile", WINDOW_NORMAL);
	//imshow("depthFile", matDepth);

	//waitKey(0);

	return points;
}

void CColorBasics::ApplyClothing(vector<Point> personPoints) {
	m_personImage = Mat(cColorHeight, cColorWidth, CV_8UC4, m_colorBuffer);
	//cvtColor(m_personImage, m_personImage, CV_BGRA2BGR);
	m_personImage.convertTo(m_personImage, CV_32FC4, 1.0 / 255.0f);

	int triangles[8][3] = {
		{ 2, 4, 6 },
		{ 2, 0, 6 },
		{ 0, 1, 6 },
		{ 6, 1, 7 },
		{ 1, 3, 7 },
		{ 3, 5, 7 },
		{ 6, 8, 9 },
		{ 6, 7, 9 }
	};

	vector<vector<Point>> not_delauney_triangles;

	for (int i = 0; i < 8; i++) {
		vector<Point> source_t, dest_t;
		for (int j = 0; j < 3; j++) {
			source_t.push_back(m_shirtPoints[triangles[i][j]]);
			dest_t.push_back(personPoints[triangles[i][j]]);
		}

		not_delauney_triangles.push_back(dest_t);
		MapTriangle(source_t, dest_t);

		for (int i = 0; i < not_delauney_triangles.size(); i++) {
			vector<Point> destination_t = not_delauney_triangles[i];

			for (int j = 0; j < destination_t.size(); j++) {
				Point start = destination_t[j];
				Point end = (j == destination_t.size() - 1) ? destination_t[0] : destination_t[j + 1];
				line(m_personImage, start, end, RED, 2);
			}
		}

		namedWindow("person", WINDOW_NORMAL);
		imshow("person", m_personImage);
		waitKey(0);
	}
}

void CColorBasics::UpdateDepth(UINT* capacity, int* width, int* height)
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

void CColorBasics::UpdateBody() 
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
						Joint joint = joints[j];
						m_joints[j] = joints[j];

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
			m_clothingImage = imread("C:\\Users\\Ryan\\~\\code\\fydp-2\\resources\\pink_tshirt_transparent.png", IMREAD_UNCHANGED);
			m_clothingImage.convertTo(m_clothingImage, CV_32F, 1.0/255.0f);
			m_shirtPoints = readClothingPoints("C:\\Users\\Ryan\\~\\code\\fydp-2\\resources\\black_tshirt.jpg.txt");

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

		if (m_bSaveScreenshot) {
		}
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
			// HRESULT hr = SaveBitmapToFile(reinterpret_cast<BYTE*>(m_pDepthRGBX), nWidth, nHeight, sizeof(RGBQUAD) * 8, szScreenshotPath);
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