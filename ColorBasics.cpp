//------------------------------------------------------------------------------
// <copyright file="ColorBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include "ColorBasics.h"

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
				SetStatusMessage("Where da bonez at", 5000, true);
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
			SetStatusMessage("Kinect not wok??", 5000, true);
			return;
		}

		// Cutoff for polarization
		DepthSpacePoint dspLowestAnkle = { 0 };
		if (m_joints[JointType_AnkleLeft].Position.Y < m_joints[JointType_AnkleRight].Position.Y) {
			m_pCoordinateMapper->MapCameraPointToDepthSpace(m_joints[JointType_AnkleLeft].Position, &dspLowestAnkle);
		}
		else {
			m_pCoordinateMapper->MapCameraPointToDepthSpace(m_joints[JointType_AnkleRight].Position, &dspLowestAnkle);
		}

		// Edge Detection
		ComponentPolarizer componentPolarizer(m_depthBuffer, cDepthHeight, cDepthWidth);
		componentPolarizer.Polarize(dspHipJoint.X, dspHipJoint.Y, (int) dspLowestAnkle.Y + 2);

		// Landmark Recognition
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
		bodyLandmarkRecognizer.buildTracePoints();

		// vector<Point> upperBodyPoints = bodyLandmarkRecognizer.returnPointsFor(ClothingType_Shirt);
		// vector<Point> lowerBodyPoints = bodyLandmarkRecognizer.returnPointsFor(ClothingType_Shorts);
		// vector<Point> pantsBodyTracePoints = bodyLandmarkRecognizer.returnPointsFor(ClothingType_Pants);
		// vector<Point> sweaterBodyTracePoints = bodyLandmarkRecognizer.returnPointsFor(ClothingType_Sweater);
		vector<Point> fullBodyTracePoints = bodyLandmarkRecognizer.returnPointsFor(ClothingType_FullBody);

		m_personImage = Mat(cColorHeight, cColorWidth, CV_8UC4, m_colorBuffer);
		m_personImage.convertTo(m_personImage, CV_32FC4, 1.0 / 255.0f);
		ClothingMapper clothingMapper(&m_personImage);
		// clothingMapper.ApplyClothing(ClothingType_Shorts, m_shortsImage, m_shortsPoints, lowerBodyPoints, true);
		// clothingMapper.ApplyClothing(ClothingType_Pants, m_pantsImage, m_pantsPoints, pantsBodyTracePoints, false);
		// clothingMapper.ApplyClothing(ClothingType_Shirt, m_shirtImage, m_shirtPoints, upperBodyPoints, true);
		// clothingMapper.ApplyClothing(ClothingType_Sweater, m_sweaterImage, m_sweaterPoints, sweaterBodyTracePoints, false);
		clothingMapper.ApplyClothing(ClothingType_FullBody, m_fullBodyClothingImage, m_fullBodyPoints, fullBodyTracePoints, false);
		//for (int i = 0; i < JointType_Count; i++) {
		//	Joint joint = m_joints[i];
		//	ColorSpacePoint csp = { 0 };
		//	m_pCoordinateMapper->MapCameraPointToColorSpace(joint.Position, &csp);
		//	circle(m_personImage, Point((int)csp.X, (int)csp.Y), 10, GREEN_8U, FILLED, LINE_8);
		//}

		namedWindow("Result", WINDOW_NORMAL);
		imshow("Result", m_personImage);

		waitKey(0);
		destroyWindow("Result");

		// Write to file
		time_t timestamp = time(nullptr);
		sprintf(filepath, "%s%d%s", OUTPUT_DIRECTORY.c_str(), (int)timestamp, FILE_EXTENSION.c_str());
		WriteLayeredPng(filepath, m_personImage);

		// Update status message
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
                hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(m_colorBuffer), ColorImageFormat_Bgra);}
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

			vector<vector<Joint>> allJoints;

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

				Joint joints[JointType_Count];
				vector<Joint> currentJoints(JointType_Count);

				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr))
				{
					for (int j = 0; j < _countof(joints); ++j)
					{
						currentJoints[j] = joints[j];
						atLeastOneBodyCaptured = true;
					}
				}

				allJoints.push_back(currentJoints);
            }

			// Select the body closest to the center of the kinect
			float bestX = 999999999.9f;
			int bestBodyIndex = 0;
			for (int i = 0; i < allJoints.size(); i++) {
				Joint currentSpineBase = allJoints[i][JointType_SpineBase];
				if (abs(currentSpineBase.Position.X) < bestX) {
					// Nice
					bestX = abs(currentSpineBase.Position.X);
					bestBodyIndex = i;
				}
			}
			// m_joints = allJoints[bestBodyIndex].data();
			for (int j = 0; j < JointType_Count; ++j)
			{
				m_joints[j] = allJoints[bestBodyIndex][j];
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
			m_shirtImage = imread("./resources/upper/superman_tshirt.png", IMREAD_UNCHANGED);
			resize(m_shirtImage, m_clothingPreview, Size(400, (400 * m_shirtImage.size().height) / m_shirtImage.size().width));
			m_shirtImage.convertTo(m_shirtImage, CV_32F, 1.0/255.0f);
			m_shirtPoints = readClothingPoints("./resources/upper/superman_tshirt.png.txt");

			m_shortsImage = imread("./resources/lower/blue_shorts.png", IMREAD_UNCHANGED);
			m_shortsImage.convertTo(m_shortsImage, CV_32F, 1.0 / 255.0f);
			m_shortsPoints = readClothingPoints("./resources/lower/blue_shorts.png.txt");

			m_sweaterImage = imread("./resources/upper/grey_sweater.png", IMREAD_UNCHANGED);
			m_sweaterImage.convertTo(m_sweaterImage, CV_32F, 1.0 / 255.0f);
			m_sweaterPoints = readClothingPoints("./resources/upper/grey_sweater.png.txt");

			m_pantsImage = imread("./resources/lower/navy_pants.png", IMREAD_UNCHANGED);
			m_pantsImage.convertTo(m_pantsImage, CV_32F, 1.0 / 255.0f);
			m_pantsPoints = readClothingPoints("./resources/lower/navy_pants.png.txt");

			m_fullBodyClothingImage = imread("./resources/ironman.png", IMREAD_UNCHANGED);
			m_fullBodyClothingImage.convertTo(m_fullBodyClothingImage, CV_32F, 1.0 / 255.0f);
			m_fullBodyPoints = readClothingPoints("./resources/ironman.png.txt");

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
            // Button clicked
			if(BN_CLICKED == HIWORD(wParam)){
				switch (LOWORD(wParam)){
					// If it was for the screenshot control and a button clicked event, save a screenshot next frame
					case IDC_BUTTON_SCREENSHOT:
						m_bSaveScreenshot = true;
						break;
					case IDC_BUTTON_SHIRT_PREV:
						SetStatusMessage(L"Clicked previous shirt button", 5000, true);
						break;
					case IDC_BUTTON_SHIRT_NEXT:
						SetStatusMessage(L"Clicked next shirt button", 5000, true);
						break;
					case IDC_BUTTON_SWEATER_PREV:
						SetStatusMessage(L"Clicked previous sweater button", 5000, true);
						break;
					case IDC_BUTTON_SWEATER_NEXT:
						SetStatusMessage(L"Clicked next sweater button", 5000, true);
						break;
					case IDC_BUTTON_SHORTS_PREV:
						SetStatusMessage(L"Clicked previous shorts button", 5000, true);
						break;
					case IDC_BUTTON_SHORTS_NEXT:
						SetStatusMessage(L"Clicked next shorts button", 5000, true);
						break;
				}
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
		m_personImage = Mat(cColorHeight, cColorWidth, CV_8UC4, pBuffer);
		m_clothingPreview.copyTo(m_personImage(Rect(0, 0, m_clothingPreview.size().width, m_clothingPreview.size().height)));

        // Draw the data with Direct2D
        //m_pDrawColor->Draw(reinterpret_cast<BYTE*>(pBuffer), cColorWidth * cColorHeight * sizeof(RGBQUAD));
        m_pDrawColor->Draw(reinterpret_cast<BYTE*>(m_personImage.data), cColorWidth * cColorHeight * sizeof(RGBQUAD));

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
