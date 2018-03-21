//------------------------------------------------------------------------------
// <copyright file="ColorBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include <fstream>
#include <ctime>
#include <unordered_map>
#include <iostream>

#include "resource.h"
#include "ImageRenderer.h"
#include "DisjointSet.h"
#include "ComponentPolarizer.h"
#include "BodyLandmarkRecognizer.h"
#include "ClothingMapper.h"

using namespace cv;
using namespace std;

class CColorBasics
{
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;

	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

public:
	/// <summary>
	/// Constructor
	/// </summary>
	CColorBasics();

	/// <summary>
	/// Destructor
	/// </summary>
	~CColorBasics();

	/// <summary>
	/// Handles window messages, passes most to the class instance to handle
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Handle windows messages for a class instance
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Creates the main window and begins processing
	/// </summary>
	/// <param name="hInstance"></param>
	/// <param name="nCmdShow"></param>
	int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
	string 					OUTPUT_DIRECTORY = "./output/";
	string 					FILE_EXTENSION = ".png";
	HWND                    m_hWnd;
	INT64                   m_nStartTime;
	INT64                   m_nLastCounter;
	double                  m_fFreq;
	INT64                   m_nNextStatusTime;
	DWORD                   m_nFramesSinceUpdate;
	bool                    m_bSaveScreenshot;
	bool					m_ranOnceAlready = false;
	char 					filepath[255];

	vector<Point>			m_shirtPoints;
	vector<Point>			m_shortsPoints;
	vector<Point>			m_sweaterPoints;
	vector<Point>			m_pantsPoints;
	vector<Point>			m_fullBodyPoints;

	UINT16*					m_depthBuffer;
	RGBQUAD*				m_colorBuffer;
	Joint					m_joints[JointType_Count];

	Mat						m_personImage;
	Mat						m_shirtImage;
	Mat						m_shortsImage;
	Mat						m_sweaterImage;
	Mat						m_pantsImage;
	Mat						m_fullBodyClothingImage;
	Mat						m_clothingPreview;

	// Current Kinect
	IKinectSensor*          m_pKinectSensor;
	ICoordinateMapper*		m_pCoordinateMapper;

	// Stream Readers
	IColorFrameReader*      m_pColorFrameReader;
	IDepthFrameReader*      m_pDepthFrameReader;
	IBodyFrameReader*       m_pBodyFrameReader;

	// Direct2D
	ImageRenderer*          m_pDrawColor;
	ID2D1Factory*           m_pD2DFactory;
	RGBQUAD*                m_pColorRGBX;
	RGBQUAD*                m_pDepthRGBX;

	/// <summary>
	/// Main processing function
	/// </summary>
	void					CopyClothingToBuffer();
	void					LoadBinaryData();
	void					StoreBinaryData();
	void                    Update();
	void				    UpdateColor();
	void					UpdateDepth();
	bool					UpdateBody();
	void					MapTriangle(vector<Point> &source_t, vector<Point> &destination_t, vector<pair<Point, Point>> cutoffLines, Mat clothingImage);
	void					ApplyClothing(const int triangles[][3], int numTriangles, Mat clothingPiece, vector<Point> clothingPoints, vector<Point> bodyPoints, bool drawTriangles);
	void					WriteLayeredPng(String filename, Mat mat);
	vector<Point>			readClothingPoints(string filename);

	void Output(const char* szFormat, ...);

	/// <summary>
	/// Initializes the default Kinect sensor
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                 InitializeDefaultSensor();

	/// <summary>
	/// Handle new color data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="pBuffer">pointer to frame data</param>
	/// <param name="nWidth">width (in pixels) of input image data</param>
	/// <param name="nHeight">height (in pixels) of input image data</param>
	/// </summary>
	void                    ProcessColor(INT64 nTime, RGBQUAD* pBuffer, int nWidth, int nHeight);

	/// <summary>
	/// Handle new depth data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="pBuffer">pointer to frame data</param>
	/// <param name="nWidth">width (in pixels) of input image data</param>
	/// <param name="nHeight">height (in pixels) of input image data</param>
	/// <param name="nMinDepth">minimum reliable depth</param>
	/// <param name="nMaxDepth">maximum reliable depth</param>
	/// </summary>
	void                    ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth, USHORT nMinDepth, USHORT nMaxDepth);

	/// <summary>
	/// Set the status bar message
	/// </summary>
	/// <param name="szMessage">message to display</param>
	/// <param name="nShowTimeMsec">time in milliseconds for which to ignore future status messages</param>
	/// <param name="bForce">force status update</param>
	bool                    SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

	/// <summary>
	/// Get the name of the file where screenshot will be stored.
	/// </summary>
	/// <param name="lpszFilePath">string buffer that will receive screenshot file name.</param>
	/// <param name="nFilePathSize">number of characters in lpszFilePath string buffer.</param>
	/// <returns>
	/// S_OK on success, otherwise failure code.
	/// </returns>
	HRESULT                 GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize);

	/// <summary>
	/// Save passed in image data to disk as a bitmap
	/// </summary>
	/// <param name="pBitmapBits">image data to save</param>
	/// <param name="lWidth">width (in pixels) of input image data</param>
	/// <param name="lHeight">height (in pixels) of input image data</param>
	/// <param name="wBitsPerPixel">bits per pixel of image data</param>
	/// <param name="lpszFilePath">full file path to output bitmap to</param>
	/// <returns>indicates success or failure</returns>
	HRESULT                 SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath);
};
