//------------------------------------------------------------------------------
// <copyright file="stdafx.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// include file for standard system and project includes

#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#endif

#define RED_16U Scalar(0, 0, USHRT_MAX)
#define BLUE_16U Scalar(USHRT_MAX, 0, 0)
#define GREEN_16U Scalar(0, USHRT_MAX, 0)
#define RED_8U Scalar(0, 0, 255)
#define BLUE_8U Scalar(255, 0, 0)
#define GREEN_8U Scalar(0, 255, 0)

// Windows Header Files
#include <windows.h>
#include <Shlobj.h>

// Direct2D Header Files
#include <d2d1.h>

// Kinect Header files
#include <Kinect.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Types
#include "basetsd.h"

// Common std
#include <strsafe.h>
#include <vector>
#include <stdexcept>

#pragma comment (lib, "d2d1.lib")

#ifdef _UNICODE
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif

typedef enum _ClothingType {
	ClothingType_Shirt,
	ClothingType_Shorts,
	ClothingType_Sweater,
	ClothingType_Pants,
	ClothingType_FullBody
} ClothingType;

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}
