/*===================================================================================
                                    PaddleCurves
                          Copyright Kerry R. Loux 2019

                   This code is licensed under the GPLv2 License
                     (http://opensource.org/licenses/GPL-2.0).

===================================================================================*/

// File:  plotterApp.cpp
// Date:  3/26/2019
// Auth:  K. Loux
// Desc:  The application class.

// wxWidgets headers
#include <wx/wx.h>

// Local headers
#include "paddleCurvesApp.h"
#include "mainFrame.h"

// Implement the application (have wxWidgets set up the appropriate entry points, etc.)
IMPLEMENT_APP(PaddleCurvesApp);

//==========================================================================
// Class:			PaddleCurvesApp
// Function:		Constant Declarations
//
// Description:		Constant declarations for the PaddleCurvesApp class.
//
// Input Arguments:
//		None
//
// Output Arguments:
//		None
//
// Return Value:
//		None
//
//==========================================================================
const wxString PaddleCurvesApp::appTitle = _T("Paddle Curves");
const wxString PaddleCurvesApp::appName = _T("PaddleCurvesApplication");
const wxString PaddleCurvesApp::creator = _T("Kerry Loux");
// gitHash and versionString are defined in gitHash.cpp, which is automatically generated during the build

//==========================================================================
// Class:			PaddleCurvesApp
// Function:		OnInit
//
// Description:		Initializes the application window.
//
// Input Arguments:
//		None
//
// Output Arguments:
//		None
//
// Return Value:
//		bool = true for successful window initialization, false for error
//
//==========================================================================
bool PaddleCurvesApp::OnInit()
{
	SetAppName(appName);
	SetVendorName(creator);

	mainFrame = new MainFrame();

	if (!mainFrame)
		return false;

	mainFrame->Show(true);

	return true;
}
