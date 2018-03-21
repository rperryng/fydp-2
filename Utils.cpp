#include "stdafx.h"
#include "Utils.h"

namespace Utils {
	void Output(const char* szFormat, ...) {
		char szBuff[1024];
		va_list arg;
		va_start(arg, szFormat);
		_vsnprintf(szBuff, sizeof(szBuff), szFormat, arg);
		va_end(arg);

		OutputDebugStringA(szBuff);
		OutputDebugStringA("\n");
	}
}
