//------------------------------------------------------------------------------
// <copyright file="ColorBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include "ColorBasics.h"

#define RED_8U Scalar(0, 0, USHRT_MAX)
#define BLUE_8U Scalar(USHRT_MAX, 0, 0)
#define GREEN_8U Scalar(0, USHRT_MAX, 0)
#define RED Scalar(0, 0, 255)
#define BLUE Scalar(255, 0, 0)
#define GREEN Scalar(0, 255, 0)

int directionOfPoint(pair<Point, Point> line, Point c) {
	Point a = line.first;
	Point b = line.second;
	return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x));
}

vector<Point> cornersOfRect(Rect rectangle) {
	vector<Point> corners;
	Point origin(rectangle.x, rectangle.y);
	corners.push_back(origin);
	corners.push_back(origin + Point(rectangle.width - 1, 0));
	corners.push_back(origin + Point(0, rectangle.height - 1));
	corners.push_back(origin + Point(rectangle.width - 1, rectangle.height - 1));
	return corners;
}

bool sameSign(int x, int y) {
	return (y >= 0) ^ (x < 0);
}

void CColorBasics::MapTriangle(
	vector<Point> &source_t,
	vector<Point> &destination_t,
	vector<pair<Point, Point>> cutoffLines,
	Mat clothingImage
) {
	// Find bounding rectangle for each triangle
	Rect source_rect = boundingRect(source_t);
	Rect destination_rect = boundingRect(destination_t);

	// Offset pointsShirt by left top corner of the respective rectangles
	Point2f source_triangle_bounded[3];
	Point2f destination_triangle_bounded[3];
	Point destination_triangle_bounded_int[3];

	for (int i = 0; i < 3; i++) {
		source_triangle_bounded[i] = Point2f(source_t[i].x - source_rect.x, source_t[i].y - source_rect.y);
		destination_triangle_bounded[i] = Point2f(destination_t[i].x - destination_rect.x, destination_t[i].y - destination_rect.y);
		destination_triangle_bounded_int[i] = Point(destination_t[i].x - destination_rect.x, destination_t[i].y - destination_rect.y);
	}

	//	Get mask by filling triangle
	vector<Mat> masks;
	for (int i = 0; i < cutoffLines.size(); i++) {
		pair<Point, Point> cutoffLine = cutoffLines[i];
		Point thirdPoint;

		for (int j = 0; j < destination_t.size(); j++) {
			if (destination_t[j] == cutoffLine.first) continue;
			if (destination_t[j] == cutoffLine.second) continue;
			thirdPoint = destination_t[j];
		}

		int thirdPointDirection = directionOfPoint(cutoffLine, thirdPoint);
		Point maskCorner;
		bool foundCorner = false;
		vector<Point> corners = cornersOfRect(destination_rect);

		for (int j = 0; j < corners.size(); j++) {
			Point corner = corners[j];
			//circle(m_personImage, corner, 5, GREEN_8U, FILLED, LINE_8);

			int cornerDirection = directionOfPoint(cutoffLine, corner);
			if (cornerDirection != 0 && !sameSign(thirdPointDirection, cornerDirection)) {
				foundCorner = true;
				maskCorner = corner;
			}
		}

		if (foundCorner) {
			Mat mask = Mat::zeros(destination_rect.height, destination_rect.width, CV_32FC4);
			Point origin(destination_rect.x, destination_rect.y);
			Point maskTriangle[3] = {
				cutoffLine.first - origin,
				cutoffLine.second - origin,
				maskCorner - origin
			};

			cv::fillConvexPoly(mask, maskTriangle, 3, Scalar(1.0, 1.0, 1.0, 1.0));
			mask = Scalar(1.0, 1.0, 1.0, 1.0) - mask;
			cv::line(mask, cutoffLine.first - origin, cutoffLine.second - origin, Scalar(1.0, 1.0, 1.0, 1.0), 1);
			masks.push_back(mask);
		}
	}

	//	Apply warpImage to small rectangular patches
	Mat source_crop;
	clothingImage(source_rect).copyTo(source_crop);

	Mat dest_crop = Mat::zeros(destination_rect.height, destination_rect.width, source_crop.type());

	// Given a pair of triangles, find the affine transform.
	Mat warp_mat = getAffineTransform(source_triangle_bounded, destination_triangle_bounded);

	// Apply the Affine Transform just found to the src image
	cv::warpAffine(source_crop, dest_crop, warp_mat, dest_crop.size());

	Mat alpha_mask;
	cv::extractChannel(dest_crop, alpha_mask, 3);
	cvtColor(alpha_mask, alpha_mask, COLOR_GRAY2BGRA);

	for (int i = 0; i < masks.size(); i++) {
		cv::multiply(alpha_mask, masks[i], alpha_mask);
	}

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
/// <param name="lpCmdLine">command cutoffLine arguments</param>
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

	file = fopen("./bin_dumps/color.dump", "rb");
	fread(m_colorBuffer, sizeof(RGBQUAD), cColorWidth * cColorHeight, file);
	fclose(file);

	file = fopen("./bin_dumps/depth.dump", "rb");
	fread(m_depthBuffer, sizeof(USHORT), cDepthWidth * cDepthHeight, file);
	fclose(file);

	file = fopen("./bin_dumps/bonez.dump", "rb");
	fread(m_joints, sizeof(Joint), JointType_Count, file);
	fclose(file);
}

void CColorBasics::StoreBinaryData() {
	FILE *file;

	file = fopen("./bin_dumps/color.dump", "wb");
	fwrite(m_colorBuffer, sizeof(RGBQUAD), cColorWidth * cColorHeight, file);
	fclose(file);

	file = fopen("./bin_dumps/depth.dump", "wb");
	fwrite(m_depthBuffer, sizeof(USHORT), cDepthWidth * cDepthHeight, file);
	fclose(file);

	file = fopen("./bin_dumps/bonez.dump", "wb");
	fwrite(m_joints, sizeof(Joint), JointType_Count, file);
	fclose(file);
}

//Read pointsShirt from text file
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

void CColorBasics::WriteLayeredPng(String filename, Mat mat) {
	Mat output;
	vector<int> compressionParams;
	compressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compressionParams.push_back(0);
	mat.convertTo(output, CV_8UC4, 255);
	output.reshape(1, output.rows*output.cols).col(3).setTo(Scalar(255));

	imwrite(filename, output, compressionParams);
}

/// <summary>
/// Main processing function
/// </summary>
void CColorBasics::Update()
{
	UpdateColor();

	bool loadBinaryData = true;
	bool storeBinaryData = false;

	if (loadBinaryData && !m_ranOnceAlready) {
		LoadBinaryData();
	}

	if (m_bSaveScreenshot || (loadBinaryData && !m_ranOnceAlready))
	{
		if (!loadBinaryData) {
			if (!UpdateBody()) {
				Output("Where da bonez at");
				return;
			}
			UpdateDepth();
		}

		if (storeBinaryData) {
			StoreBinaryData();
		}

		DepthSpacePoint dspHipJoint = { 0 };
		m_pCoordinateMapper->MapCameraPointToDepthSpace(m_joints[0].Position, &dspHipJoint);
		if (isinf(dspHipJoint.X) || isinf(dspHipJoint.Y)) {
			Output("Kinect not wok??");
			return;
		}

		ComponentPolarizer componentPolarizer(m_depthBuffer, cDepthHeight, cDepthWidth);
		componentPolarizer.Polarize(dspHipJoint.X, dspHipJoint.Y);

		BodyLandmarkRecognizer bodyLandmarkRecognizer(
			m_depthBuffer,
			cDepthHeight,
			cDepthWidth,
			m_colorBuffer,
			cColorHeight,
			cColorWidth,
			m_joints,
			m_pCoordinateMapper
		);
		vector<Point> upperBodyPoints = bodyLandmarkRecognizer.recognizeFor(ClothingType_Shirt);
		vector<Point> lowerBodyPoints = bodyLandmarkRecognizer.recognizeFor(ClothingType_Shorts);

		// With triangles
		m_personImage = Mat(cColorHeight, cColorWidth, CV_8UC4, m_colorBuffer);
		m_personImage.convertTo(m_personImage, CV_32FC4, 1.0 / 255.0f);
		ApplyClothing(cTrianglesShirt, cNumTrianglesShirt, m_shirtImage, m_shirtPoints, upperBodyPoints, true);
		ApplyClothing(cTrianglesShorts, cNumTrianglesShorts, m_shortsImage, m_shortsPoints, lowerBodyPoints, true);
		namedWindow("Triangles", WINDOW_NORMAL);
		imshow("Triangles", m_personImage);
		WriteLayeredPng("./bin_dumps/triangles.png", m_personImage);

		// Without triangles
		m_personImage = Mat(cColorHeight, cColorWidth, CV_8UC4, m_colorBuffer);
		m_personImage.convertTo(m_personImage, CV_32FC4, 1.0 / 255.0f);
		ApplyClothing(cTrianglesShirt, cNumTrianglesShirt, m_shirtImage, m_shirtPoints, upperBodyPoints, true);
		namedWindow("Shirt Only", WINDOW_NORMAL);
		imshow("Shirt Only", m_personImage);
		WriteLayeredPng("./bin_dumps/shirt_only.png", m_personImage);

		m_personImage = Mat(cColorHeight, cColorWidth, CV_8UC4, m_colorBuffer);
		m_personImage.convertTo(m_personImage, CV_32FC4, 1.0 / 255.0f);
		ApplyClothing(cTrianglesShorts, cNumTrianglesShorts, m_shortsImage, m_shortsPoints, lowerBodyPoints, false);
		namedWindow("Shorts Only", WINDOW_NORMAL);
		imshow("Shorts Only", m_personImage);
		WriteLayeredPng("./bin_dumps/shorts_only.png", m_personImage);

		// Reapply shirt
		ApplyClothing(cTrianglesShirt, cNumTrianglesShirt, m_shirtImage, m_shirtPoints, upperBodyPoints, false);

		WriteLayeredPng("./bin_dumps/result.png", m_personImage);
		namedWindow("Result", WINDOW_NORMAL);
		imshow("Result", m_personImage);

		waitKey(0);
		destroyWindow("Connected Components");
		destroyWindow("Connected Components with Landmarks");
		destroyWindow("Color with Landmarks");
		destroyWindow("Triangles");
		destroyWindow("Shirt Only");
		destroyWindow("Shorts Only");
		destroyWindow("Result");

		ColorSpacePoint csp = { 0 };
		m_pCoordinateMapper->MapCameraPointToColorSpace(m_joints[JointType_SpineBase].Position, &csp);
		circle(m_personImage, Point(csp.X, csp.Y), 3, RED_8U);

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

bool vectorContains(vector<Point> container, Point value) {
	return find(container.begin(), container.end(), value) != container.end();
}

void CColorBasics::ApplyClothing(
	const int triangles[][3],
	int numTriangles,
	Mat matClothing,
	vector<Point> clothingPoints,
	vector<Point> bodyPoints,
	bool drawTriangles
) {
	vector<vector<Point>> sourceTriangles;
	vector<vector<Point>> destinationTriangles;
	for (int i = 0; i < numTriangles; i++) {
		vector<Point> source_t, dest_t;
		for (int j = 0; j < 3; j++) {
			source_t.push_back(clothingPoints[triangles[i][j]]);
			dest_t.push_back(bodyPoints[triangles[i][j]]);
		}

		sourceTriangles.push_back(source_t);
		destinationTriangles.push_back(dest_t);
	}

	vector<pair<Point, Point>> allCutoffLines;
	for (int i = 0; i < numTriangles; i++) {
		vector<Point> source_t = sourceTriangles[i];
		vector<Point> destination_t = destinationTriangles[i];
		vector<pair<Point, Point>> cutoffLines;

		for (int j = 0; j < numTriangles; j++) {
			if (i == j) continue;
			vector<Point> other_t = destinationTriangles[j];

			for (int k = 0; k < destination_t.size(); k++) {
				Point start = destination_t[k];
				Point end = destination_t[(k == destination_t.size() - 1) ? 0 : k + 1];
				if (vectorContains(other_t, start) && vectorContains(other_t, end)) {
					cutoffLines.push_back(pair<Point, Point>(start, end));
					allCutoffLines.push_back(pair<Point, Point>(start, end));
				}
			}
		}

		MapTriangle(source_t, destination_t, cutoffLines, matClothing);
	}



	// Draw Triangles
	if (!drawTriangles) return;

	for (int i = 0; i < destinationTriangles.size(); i++) {
		vector<Point> currentTriangle = destinationTriangles[i];
		for (int j = 0; j < currentTriangle.size(); j++) {
			Point start = currentTriangle[j];
			Point end = (j == currentTriangle.size() - 1) ? currentTriangle[0] : currentTriangle[j + 1];
			line(m_personImage, start, end, RED_8U, 2);
		}
	}

	// Draw cutoff lines
	for (int j = 0; j < allCutoffLines.size(); j++) {
		pair<Point, Point> cutoffLine = allCutoffLines[j];
		line(m_personImage, cutoffLine.first, cutoffLine.second, GREEN_8U, 2);
	}
}

void CColorBasics::UpdateDepth()
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
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
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

			// Note:  If you wish to filter by reliable depth distance, uncomment the following cutoffLine.
            hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
        }

        if (SUCCEEDED(hr))
        {
			UINT16* underlyingBuffer = NULL;
            hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &underlyingBuffer);            

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

bool CColorBasics::UpdateBody() 
{
    IBodyFrame* pBodyFrame = NULL;
    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
	bool atLeastOneBodyCaptured = false;

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
						m_joints[j] = joints[j];
						atLeastOneBodyCaptured = true;
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
	return atLeastOneBodyCaptured;
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
			m_shirtImage = imread("./resources/superman_tshirt_transparent.png", IMREAD_UNCHANGED);
			m_shirtImage.convertTo(m_shirtImage, CV_32F, 1.0/255.0f);
			m_shirtPoints = readClothingPoints("./resources/superman_tshirt.jpg.txt");

			m_shortsImage = imread("./resources/red_shorts_transparent.png", IMREAD_UNCHANGED);
			m_shortsImage.convertTo(m_shortsImage, CV_32F, 1.0 / 255.0f);
			m_shortsPoints = readClothingPoints("./resources/red_shorts.jpg.txt");

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