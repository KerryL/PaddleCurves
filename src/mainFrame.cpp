/*===================================================================================
                                    PaddleCurves
                          Copyright Kerry R. Loux 2019

                   This code is licensed under the GPLv2 License
                     (http://opensource.org/licenses/GPL-2.0).

===================================================================================*/

// File:  mainFrame.cpp
// Date:  3/26/2019
// Auth:  K. Loux
// Desc:  The main GUI class for the application.

// Standard C++ headers
#include <algorithm>

// Local headers
#include "mainFrame.h"
#include "paddleCurvesApp.h"

// LibPlot2D headers
#include <lp2d/renderer/plotRenderer.h>
#include <lp2d/utilities/guiUtilities.h>
#include <lp2d/libPlot2D.h>

// wxWidgets headers
#include <wx/valnum.h>

//==========================================================================
// Class:			MainFrame
// Function:		Constant declarations
//
// Description:		Constant declarations for MainFrame class.
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
const unsigned long long MainFrame::mHighQualityCurvePointLimit(10000);

//==========================================================================
// Class:			MainFrame
// Function:		MainFrame
//
// Description:		Constructor for MainFrame class.  Initializes the form
//					and creates the controls, etc.
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
MainFrame::MainFrame() : wxFrame(nullptr, wxID_ANY, wxEmptyString,
	wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE), mPlotInterface(this)
{
	geometryInfo.splineInfo = geometryInfo.BuildDefaultSplineInfo();
	CreateControls();
	SetProperties();
	initialized = true;
	
	TransferDataFromWindow();
	UpdateCurveDataAndCalculations();
}

//==========================================================================
// Class:			MainFrame
// Function:		CreateControls
//
// Description:		Creates sizers and controls and lays them out in the window.
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
void MainFrame::CreateControls()
{
	wxSizer *topSizer(new wxBoxSizer(wxHORIZONTAL));
	wxPanel *panel(new wxPanel(this));
	topSizer->Add(panel, wxSizerFlags().Expand());

	wxSizer *mainSizer(new wxBoxSizer(wxHORIZONTAL));
	panel->SetSizer(mainSizer);

	wxBoxSizer *sizer(new wxBoxSizer(wxVERTICAL));
	sizer->Add(CreateButtons(panel), wxSizerFlags().Border(wxALL, 5).Expand());
	sizer->Add(CreateTextInputs(panel), wxSizerFlags().Border(wxALL, 5).Expand());
	sizer->Add(CreateTextOutputs(panel), wxSizerFlags().Border(wxALL, 5).Expand());
	mainSizer->Add(sizer, wxSizerFlags().Expand().Border(wxALL, 5));

	topSizer->Add(CreatePlotArea(this), wxSizerFlags().Expand().Proportion(1));

	SetSizerAndFit(topSizer);
	TransferDataToWindow();
}

//==========================================================================
// Class:			MainFrame
// Function:		CreatePlotArea
//
// Description:		Creates the main plot control.
//
// Input Arguments:
//		parent	= wxWindow*
//
// Output Arguments:
//		None
//
// Return Value:
//		LibPlot2D::PlotRenderer* pointing to plotArea
//
//==========================================================================
LibPlot2D::PlotRenderer* MainFrame::CreatePlotArea(wxWindow *parent)
{
	wxGLAttributes displayAttributes;
	displayAttributes.PlatformDefaults().RGBA().DoubleBuffer().SampleBuffers(1).Samplers(4).Stencil(1).EndList();
	assert(wxGLCanvas::IsDisplaySupported(displayAttributes));
	mPlotArea = new LibPlot2D::PlotRenderer(mPlotInterface, *parent, wxID_ANY, displayAttributes);

	mPlotArea->SetMinSize(wxSize(650, 320));
	mPlotArea->SetMajorGridOn();
	mPlotArea->SetCurveQuality(LibPlot2D::PlotRenderer::CurveQuality::HighWrite);

	return mPlotArea;
}

//==========================================================================
// Class:			MainFrame
// Function:		CreateButtons
//
// Description:		Creates the buttons and returns the sizer pointer.
//
// Input Arguments:
//		parent	= wxWindow*
//
// Output Arguments:
//		None
//
// Return Value:
//		wxSizer*
//
//==========================================================================
wxSizer* MainFrame::CreateButtons(wxWindow* parent)
{
	wxBoxSizer *buttonSizer = new wxBoxSizer(wxHORIZONTAL);

	wxBoxSizer *topSizer = new wxBoxSizer(wxVERTICAL);
	buttonSizer->Add(topSizer);

	topSizer->Add(new wxButton(parent, idButtonOpen, _T("&Open")), wxSizerFlags().Expand().Proportion(1));
	topSizer->Add(new wxButton(parent, idButtonSave, _T("&Save")), wxSizerFlags().Expand().Proportion(1));
	topSizer->Add(new wxButton(parent, idButtonAutoScale, _T("&Auto Scale")), wxSizerFlags().Expand().Proportion(1));

	buttonSizer->Add(CreateVersionText(parent), wxSizerFlags().Border(wxALL, 5));

	return buttonSizer;
}

//==========================================================================
// Class:			MainFrame
// Function:		CreateTextInputs
//
// Description:		Creates the input text controls.
//
// Input Arguments:
//		parent	= wxWindow*
//
// Output Arguments:
//		None
//
// Return Value:
//		wxSizer*
//
//==========================================================================
wxSizer* MainFrame::CreateTextInputs(wxWindow* parent)
{
	wxStaticBoxSizer* sizer(new wxStaticBoxSizer(wxVERTICAL, parent, _T("Inputs")));
	wxSizer* subSizer(new wxFlexGridSizer(3, 5, 5));
	sizer->Add(subSizer);

	shaftWidthText = new wxTextCtrl(sizer->GetStaticBox(), wxID_ANY);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("Shaft Width")));
	subSizer->Add(shaftWidthText);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("(in)")));
	shaftWidthText->SetValidator(wxFloatingPointValidator<double>(3, &geometryInfo.shaftWidth, wxNUM_VAL_NO_TRAILING_ZEROES));

	referenceWidthText = new wxTextCtrl(sizer->GetStaticBox(), wxID_ANY);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("Reference Width")));
	subSizer->Add(referenceWidthText);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("(in)")));
	referenceWidthText->SetValidator(wxFloatingPointValidator<double>(3, &geometryInfo.referenceWidth, wxNUM_VAL_NO_TRAILING_ZEROES));

	referenceLengthText = new wxTextCtrl(sizer->GetStaticBox(), wxID_ANY);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("Reference Length")));
	subSizer->Add(referenceLengthText);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("(in)")));
	referenceLengthText->SetValidator(wxFloatingPointValidator<double>(3, &geometryInfo.referenceLength, wxNUM_VAL_NO_TRAILING_ZEROES));

	showReferenceRectangleCheckBox = new wxCheckBox(sizer->GetStaticBox(), wxID_ANY, _T("Show Reference Rectangle"));
	sizer->Add(showReferenceRectangleCheckBox);

	return sizer;
}

//==========================================================================
// Class:			MainFrame
// Function:		CreateTextOutputs
//
// Description:		Creates the output text controls.
//
// Input Arguments:
//		parent	= wxWindow*
//
// Output Arguments:
//		None
//
// Return Value:
//		wxSizer*
//
//==========================================================================
wxSizer* MainFrame::CreateTextOutputs(wxWindow* parent)
{
	wxStaticBoxSizer* sizer(new wxStaticBoxSizer(wxVERTICAL, parent, _T("Outputs")));
	wxSizer* subSizer(new wxFlexGridSizer(3, 5, 5));
	sizer->Add(subSizer);

	const wxString dummyQuantity(_T("9000.000"));
	topHalfAreaText = new wxStaticText(sizer->GetStaticBox(), wxID_ANY, dummyQuantity);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("Top Area")));
	subSizer->Add(topHalfAreaText);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("(in\u00b2)")));

	bottomHalfAreaText = new wxStaticText(sizer->GetStaticBox(), wxID_ANY, dummyQuantity);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("Bottom Area")));
	subSizer->Add(bottomHalfAreaText);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("(in\u00b2)")));

	areaMismatchText = new wxStaticText(sizer->GetStaticBox(), wxID_ANY, dummyQuantity);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("Area Mismatch")));
	subSizer->Add(areaMismatchText);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("(in\u00b2)")));

	centroidMismatchText = new wxStaticText(sizer->GetStaticBox(), wxID_ANY, dummyQuantity);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("Centroid Distance Mismatch")));
	subSizer->Add(centroidMismatchText);
	subSizer->Add(new wxStaticText(sizer->GetStaticBox(), wxID_ANY, _T("(in)")));

	return sizer;
}

//==========================================================================
// Class:			MainFrame
// Function:		CreateVersionText
//
// Description:		Builds a static text control containing version
//					information.
//
// Input Arguments:
//		parent	= wxWindow*
//
// Output Arguments:
//		None
//
// Return Value:
//		wxWindow*
//
//==========================================================================
wxWindow* MainFrame::CreateVersionText(wxWindow *parent)
{
	wxString paddleCurvesVersionString(PaddleCurvesApp::versionString
		+ _T(" (") + PaddleCurvesApp::gitHash + _T(")"));
	wxString lp2dVersionString(LibPlot2D::versionString
		+ _T(" (") + LibPlot2D::gitHash + _T(")"));

	wxStaticText* versionInfo(new wxStaticText(parent, wxID_ANY,
		paddleCurvesVersionString));
	versionInfo->SetToolTip(_T("PaddleCurves ") + paddleCurvesVersionString
		+ _T("\nLibPlot2D ") + lp2dVersionString);

	return versionInfo;
}

//==========================================================================
// Class:			MainFrame
// Function:		SetProperties
//
// Description:		Sets the window properties for this window.  Includes
//					title, frame size, and default fonts.
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
void MainFrame::SetProperties()
{
	SetTitle(PaddleCurvesApp::appTitle);
	SetName(PaddleCurvesApp::appName);
	mPlotInterface.SetApplicationTitle(PaddleCurvesApp::appTitle);
	Center();

/*#ifdef __WXMSW__
	SetIcon(wxIcon(_T("ICON_ID_MAIN"), wxBITMAP_TYPE_ICO_RESOURCE));
#elif __WXGTK__
	SetIcon(wxIcon(plots16_xpm));
	SetIcon(wxIcon(plots24_xpm));
	SetIcon(wxIcon(plots32_xpm));
	SetIcon(wxIcon(plots48_xpm));
	SetIcon(wxIcon(plots64_xpm));
	SetIcon(wxIcon(plots128_xpm));
#endif*/

	const int entryCount(3);
	wxAcceleratorEntry entries[entryCount];
	entries[0].Set(wxACCEL_CTRL, static_cast<int>('o'), idButtonOpen);
	entries[1].Set(wxACCEL_CTRL, static_cast<int>('s'), idButtonSave);
	entries[2].Set(wxACCEL_CTRL, static_cast<int>('a'), idButtonAutoScale);
	wxAcceleratorTable accel(entryCount, entries);
	SetAcceleratorTable(accel);
}

//==========================================================================
// Class:			MainFrame
// Function:		Event Table
//
// Description:		Links GUI events with event handler functions.
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
BEGIN_EVENT_TABLE(MainFrame, wxFrame)
	EVT_BUTTON(idButtonOpen,			MainFrame::ButtonOpenClickedEvent)
	EVT_BUTTON(idButtonSave,			MainFrame::ButtonSaveClickedEvent)
	EVT_BUTTON(idButtonAutoScale,		MainFrame::ButtonAutoScaleClickedEvent)
	EVT_TEXT(wxID_ANY,					MainFrame::TextChangedEvent)
	EVT_CHECKBOX(wxID_ANY,				MainFrame::TextChangedEvent)
END_EVENT_TABLE();

//==========================================================================
// Class:			MainFrame
// Function:		ButtonOpenClickedEvent
//
// Description:		Displays a dialog asking the user to specify the file to
//					read from.
//
// Input Arguments:
//		event	= wxCommandEvent&
//
// Output Arguments:
//		None
//
// Return Value:
//		None
//
//==========================================================================
void MainFrame::ButtonOpenClickedEvent(wxCommandEvent& WXUNUSED(event))
{
	// Set up the wildcard specifications (done here for readability)
	wxString wildcard("All files (*)|*");
	wildcard.append("|Comma Separated (*.csv)|*.csv");
	wildcard.append("|Tab Delimited (*.txt)|*.txt");

	wxArrayString fileList = LibPlot2D::GuiUtilities::GetFileNameFromUser(this,
		_T("Open Data File"), wxEmptyString, wxEmptyString, wildcard,
		wxFD_OPEN | wxFD_FILE_MUST_EXIST);

	if (fileList.GetCount() == 0)
		return;

	if (!ReadGeometryInfo(fileList.front(), geometryInfo))
		return;

	TransferDataToWindow();
	UpdateCurveDataAndCalculations();
}

//==========================================================================
// Class:			MainFrame
// Function:		ButtonSaveClickedEvent
//
// Description:		Displays a dialog asking the user to specify the file to
//					write to.
//
// Input Arguments:
//		event	= wxCommandEvent&
//
// Output Arguments:
//		None
//
// Return Value:
//		None
//
//==========================================================================
void MainFrame::ButtonSaveClickedEvent(wxCommandEvent& WXUNUSED(event))
{
	// Set up the wildcard specifications (done here for readability)
	wxString wildcard("All files (*)|*");
	wildcard.append("|Comma Separated (*.csv)|*.csv");
	wildcard.append("|Tab Delimited (*.txt)|*.txt");

	wxArrayString fileList = LibPlot2D::GuiUtilities::GetFileNameFromUser(this,
		_T("Save Data File"), wxEmptyString, wxEmptyString, wildcard,wxFD_SAVE);

	if (fileList.GetCount() == 0)
		return;

	WriteGeometryInfo(fileList.front(), geometryInfo);
}

//==========================================================================
// Class:			MainFrame
// Function:		ButtonAutoScaleClickedEvent
//
// Description:		Event fires when user clicks "AutoScale" button.
//
// Input Arguments:
//		event	= &wxCommandEvent
//
// Output Arguments:
//		None
//
// Return Value:
//		None
//
//==========================================================================
void MainFrame::ButtonAutoScaleClickedEvent(wxCommandEvent& WXUNUSED(event))
{
	mPlotArea->AutoScale();
}

//==========================================================================
// Class:			MainFrame
// Function:		TextChangedEvent
//
// Description:		Event fires when user changes text control inputs.
//
// Input Arguments:
//		event	= &wxCommandEvent
//
// Output Arguments:
//		None
//
// Return Value:
//		None
//
//==========================================================================
void MainFrame::TextChangedEvent(wxCommandEvent& WXUNUSED(event))
{
	TransferDataFromWindow();
	UpdateCurveDataAndCalculations();
}

//==========================================================================
// Class:			MainFrame
// Function:		SetTitleFromFileName
//
// Description:		Sets the frame's title according to the specified file name.
//
// Input Arguments:
//		pathAndFileName	= wxString
//
// Output Arguments:
//		None
//
// Return Value:
//		None
//
//==========================================================================
void MainFrame::SetTitleFromFileName(wxString pathAndFileName)
{
	wxString fileName(LibPlot2D::GuiUtilities::ExtractFileNameFromPath(pathAndFileName));
	unsigned int end(fileName.find_last_of(_T(".")));
	SetTitle(fileName.Mid(0, end) + _T(" - ") + PaddleCurvesApp::appTitle);
}

MainFrame::GeometryInfo::SplineInfo MainFrame::GeometryInfo::BuildDefaultSplineInfo() const
{
	SplineInfo info;
	info.push_back(SplinePoint(0.0, 0.5 * shaftWidth, -1.0, -1.0));
	info.back().drag = GeometryInfo::SplinePoint::DragConstraint::FixedXFixedY;

	info.push_back(SplinePoint(0.25 * referenceLength, 0.5 * referenceWidth, -1.0, 0.0));
	info.back().drag = GeometryInfo::SplinePoint::DragConstraint::FreeXFreeY;

	info.push_back(SplinePoint(0.75 * referenceLength, 0.5 * referenceWidth, 1.0, 0.0));
	info.back().drag = GeometryInfo::SplinePoint::DragConstraint::FreeXFreeY;

	info.push_back(SplinePoint(referenceLength, 0.25 * referenceWidth, 0.0, 1.0));
	info.back().drag = GeometryInfo::SplinePoint::DragConstraint::FixedXFreeY;

	info.push_back(SplinePoint(0.5 * referenceLength, -0.5 * referenceWidth, 1.0, 0.0));
	info.back().drag = GeometryInfo::SplinePoint::DragConstraint::FreeXFreeY;

	info.push_back(SplinePoint(0.0, -0.5 * shaftWidth, -1.0, 1.0));
	info.back().drag = GeometryInfo::SplinePoint::DragConstraint::FixedXFixedY;

	return info;
}

void MainFrame::UpdateCurveDataAndCalculations()
{
	if (!initialized)
		return;

	auto data(ComputeCurveData());
	double topArea, bottomArea, topCentroid, bottomCentroid;
	CalculateAreasAndCentroids(*data, topArea, bottomArea, topCentroid, bottomCentroid);
	topHalfAreaText->SetLabel(wxString::Format(_T("%0.3f"), topArea));
	bottomHalfAreaText->SetLabel(wxString::Format(_T("%0.3f"), bottomArea));
	areaMismatchText->SetLabel(wxString::Format(_T("%0.3f"), topArea + bottomArea));
	centroidMismatchText->SetLabel(wxString::Format(_T("%0.3f"), topCentroid - bottomCentroid));

	mPlotInterface.ClearAllCurves();

	std::unique_ptr<LibPlot2D::Dataset2D> centerline(std::make_unique<LibPlot2D::Dataset2D>(2));
	centerline->GetX()[0] = 0.0;
	centerline->GetY()[0] = 0.0;
	centerline->GetX()[1] = geometryInfo.referenceLength;
	centerline->GetY()[1] = 0.0;
	mPlotInterface.AddCurve(std::move(centerline), wxString());

	std::unique_ptr<LibPlot2D::Dataset2D> shaft(std::make_unique<LibPlot2D::Dataset2D>(4));
	const double showShaftLength(3.0);
	shaft->GetX()[0] = -showShaftLength;
	shaft->GetY()[0] = 0.5 * geometryInfo.shaftWidth;
	shaft->GetX()[1] = 0.0;
	shaft->GetY()[1] = 0.5 * geometryInfo.shaftWidth;
	shaft->GetX()[2] = 0.0;
	shaft->GetY()[2] = -0.5 * geometryInfo.shaftWidth;
	shaft->GetX()[3] = -showShaftLength;
	shaft->GetY()[3] = -0.5 * geometryInfo.shaftWidth;
	mPlotInterface.AddCurve(std::move(shaft), wxString());

	mPlotInterface.AddCurve(std::move(data), wxString());

	if (showReferenceRectangleCheckBox->GetValue())
	{
		std::unique_ptr<LibPlot2D::Dataset2D> referenceRect(std::make_unique<LibPlot2D::Dataset2D>(5));
		referenceRect->GetX()[0] = 0.0;
		referenceRect->GetY()[0] = 0.5 * geometryInfo.referenceWidth;
		referenceRect->GetX()[1] = geometryInfo.referenceLength;
		referenceRect->GetY()[1] = 0.5 * geometryInfo.referenceWidth;
		referenceRect->GetX()[2] = geometryInfo.referenceLength;
		referenceRect->GetY()[2] = -0.5 * geometryInfo.referenceWidth;
		referenceRect->GetX()[3] = 0.0;
		referenceRect->GetY()[3] = -0.5 * geometryInfo.referenceWidth;
		referenceRect->GetX()[4] = referenceRect->GetX()[0];
		referenceRect->GetY()[4] = referenceRect->GetY()[0];
		mPlotInterface.AddCurve(std::move(referenceRect), wxString());
	}

	mPlotInterface.ForceEqualAxisScaling();
}

std::unique_ptr<LibPlot2D::Dataset2D> MainFrame::ComputeCurveData() const
{
	constexpr unsigned int constraintsPerSegment(4);// at each end of each segment:  point + slope + curvature
	const unsigned int constraints((geometryInfo.splineInfo.size() - 1) * constraintsPerSegment);
	Eigen::MatrixXd a(constraints, constraints);
	Eigen::VectorXd bx(constraints), by(constraints);
		
	constexpr unsigned int pointsPerSegment(100);
	std::unique_ptr<LibPlot2D::Dataset2D> ds(
		std::make_unique<LibPlot2D::Dataset2D>(pointsPerSegment * (geometryInfo.splineInfo.size() - 1) + 1));

	for (unsigned int i = 1; i < geometryInfo.splineInfo.size(); ++i)
	{
		const double deltaX(geometryInfo.splineInfo[i].x - geometryInfo.splineInfo[i - 1].x);
		const double deltaY(geometryInfo.splineInfo[i].y - geometryInfo.splineInfo[i - 1].y);

		double xSlopeBefore, xSlopeAfter, ySlopeBefore, ySlopeAfter;
		if (fabs(geometryInfo.splineInfo[i - 1].xSlope) > fabs(geometryInfo.splineInfo[i - 1].ySlope))
		{
			xSlopeBefore = deltaX;
			ySlopeBefore = geometryInfo.splineInfo[i - 1].ySlope / geometryInfo.splineInfo[i - 1].xSlope * deltaX;
		}
		else
		{
			xSlopeBefore = geometryInfo.splineInfo[i - 1].xSlope / geometryInfo.splineInfo[i - 1].ySlope * deltaY;
			ySlopeBefore = deltaY;
		}

		if (fabs(geometryInfo.splineInfo[i].xSlope) > fabs(geometryInfo.splineInfo[i].ySlope))
		{
			xSlopeAfter = deltaX;
			ySlopeAfter = geometryInfo.splineInfo[i].ySlope / geometryInfo.splineInfo[i].xSlope * deltaX;
		}
		else
		{
			xSlopeAfter = geometryInfo.splineInfo[i].xSlope / geometryInfo.splineInfo[i].ySlope * deltaY;
			ySlopeAfter = deltaY;
		}

		const unsigned int offset(constraintsPerSegment * (i - 1));
		a.row(offset).setZero();
		a(offset, offset) = 1.0;
		bx(offset) = geometryInfo.splineInfo[i - 1].x;
		by(offset) = geometryInfo.splineInfo[i - 1].y;
		
		a.row(offset + 1).setZero();
		a.block<1, constraintsPerSegment>(offset + 1, offset).setOnes();
		bx(offset + 1) = geometryInfo.splineInfo[i].x;
		by(offset + 1) = geometryInfo.splineInfo[i].y;

		a.row(offset + 2).setZero();
		a(offset + 2, offset + 1) = 1.0;
		bx(offset + 2) = xSlopeBefore;
		by(offset + 2) = ySlopeBefore;
		
		a.row(offset + 3).setZero();
		for (unsigned int k = 1; k < constraintsPerSegment; ++k)
			a(offset + 3, offset + k) = k;
		bx(offset + 3) = xSlopeAfter;
		by(offset + 3) = ySlopeAfter;
	}
	
	const Eigen::VectorXd xCoef(a.fullPivLu().solve(bx));
	const Eigen::VectorXd yCoef(a.fullPivLu().solve(by));

	for (unsigned int i = 1; i < geometryInfo.splineInfo.size(); ++i)
	{
		const unsigned int offset(constraintsPerSegment * (i - 1));
		for (unsigned int j = 0; j <= pointsPerSegment; ++j)
		{
			constexpr unsigned int order(constraintsPerSegment - 1);
			const double u(static_cast<double>(j) / pointsPerSegment);
			double x(xCoef(offset));
			double y(yCoef(offset));
			for (unsigned int k = 1; k <= order; ++k)
			{
				const double uPow(pow(u, k));
				x += xCoef(offset + k) * uPow;
				y += yCoef(offset + k) * uPow;
			}

			const unsigned int index((i - 1) * pointsPerSegment + j);
			ds->GetX()[index] = x;
			ds->GetY()[index] = y;
		}
	}

	return std::move(ds);
}

void MainFrame::CalculateAreasAndCentroids(const LibPlot2D::Dataset2D& ds,
	double& topArea, double& bottomArea, double& topCentroidArm, double& bottomCentroidArm)
{
	topArea = 0.0;
	bottomArea = 0.0;
	topCentroidArm = 0.0;
	bottomCentroidArm = 0.0;

	unsigned int topCount(0);
	unsigned int bottomCount(0);

	for (unsigned int i = 1; i < ds.GetNumberOfPoints(); ++i)
	{
		const double x(ds.GetX()[i]);
		const double y(ds.GetY()[i]);
		const double lastX(ds.GetX()[i - 1]);
		const double lastY(ds.GetY()[i - 1]);

		const double sliceArea((x - lastX) * 0.5 * (y + lastY));
		const double averageArm(0.25 * (y + lastY));// TODO:  Verify centroid calcs
		if (y >= 0.0 && lastY >= 0.0)
		{
			if (x > lastX)
			{
				topArea += sliceArea;
				topCentroidArm += averageArm;
			}
			else
			{
				topArea -= sliceArea;
				topCentroidArm -= averageArm;
			}
			++topCount;
		}
		else if (y <= 0.0 && lastY <= 0.0)
		{
			if (x > lastX)
			{
				bottomArea += sliceArea;
				bottomCentroidArm += averageArm;
			}
			else
			{
				bottomArea -= sliceArea;
				bottomCentroidArm -= averageArm;
			}
			++bottomCount;
		}
	}

	topCentroidArm /= topCount;
	bottomCentroidArm /= bottomCount;
}

bool MainFrame::ReadGeometryInfo(const wxString& fileName, GeometryInfo& g)
{
	// TODO:  Implement
	return false;
}

void MainFrame::WriteGeometryInfo(const wxString& fileName, const GeometryInfo& g)
{
	// TODO:  Implement
}
