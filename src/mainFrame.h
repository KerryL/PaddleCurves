/*===================================================================================
                                    PaddleCurves
                          Copyright Kerry R. Loux 2019

                   This code is licensed under the GPLv2 License
                     (http://opensource.org/licenses/GPL-2.0).

===================================================================================*/

// File:  mainFrame.h
// Date:  3/26/2019
// Auth:  K. Loux
// Desc:  The main GUI class for the application.

#ifndef MAIN_FRAME_H_
#define MAIN_FRAME_H_

// LibPlot2D headers
#include <lp2d/gui/guiInterface.h>

// wxWidgets headers
#include <wx/wx.h>

// Standard C++ headers
#include <vector>

// LibPlot2D forward declarations
namespace LibPlot2D
{
class PlotRenderer;
}

// The main frame class
class MainFrame : public wxFrame
{
public:
	MainFrame();
	~MainFrame() = default;

private:
	static const unsigned long long mHighQualityCurvePointLimit;

	LibPlot2D::GuiInterface mPlotInterface;

	// Functions that do some of the frame initialization and control positioning
	void CreateControls();
	void SetProperties();

	LibPlot2D::PlotRenderer* CreatePlotArea(wxWindow* parent);
	wxSizer* CreateButtons(wxWindow* parent);
	wxWindow* CreateVersionText(wxWindow* parent);
	wxSizer* CreateTextInputs(wxWindow* parent);
	wxSizer* CreateTextOutputs(wxWindow* parent);

	// Controls
	wxCheckBox* showReferenceRectangleCheckBox;
	wxTextCtrl* shaftWidthText;
	wxTextCtrl* referenceWidthText;
	wxTextCtrl* referenceLengthText;

	wxStaticText* topHalfAreaText;
	wxStaticText* bottomHalfAreaText;
	wxStaticText* areaMismatchText;
	wxStaticText* centroidMismatchText;

	LibPlot2D::PlotRenderer *mPlotArea;

	// The event IDs
	enum MainFrameEventID
	{
		idButtonOpen = wxID_HIGHEST + 500,
		idButtonSave,
		idButtonAutoScale
	};

	// Button events
	void ButtonOpenClickedEvent(wxCommandEvent &event);
	void ButtonAutoScaleClickedEvent(wxCommandEvent &event);
	void ButtonSaveClickedEvent(wxCommandEvent &event);

	void TextChangedEvent(wxCommandEvent &event);

	void SetTitleFromFileName(wxString pathAndFileName);

	struct GeometryInfo
	{
		double shaftWidth = 1.125;// [in]
		double referenceWidth = 5.0;// [in]
		double referenceLength = 16.0;// [in]

		struct SplinePoint
		{
			SplinePoint(const double& x, const double& y, const double& xSlope, const double& ySlope)
				: x(x), y(y), xSlope(xSlope), ySlope(ySlope) {}
			double x;
			double y;
			double xSlope;
			double ySlope;

			enum class DragConstraint
			{
				FreeXFreeY,
				FreeXFixedY,
				FixedXFreeY,
				FixedXFixedY
			};

			DragConstraint drag = DragConstraint::FreeXFreeY;
		};

		typedef std::vector<SplinePoint> SplineInfo;
		SplineInfo splineInfo;

		SplineInfo BuildDefaultSplineInfo() const;
	} geometryInfo;

	void UpdateCurveDataAndCalculations();
	static void ComputeSegmentSlopes(const GeometryInfo::SplinePoint& s,
		const double& deltaX, const double& deltaY, double& xSlope, double& ySlope);
	std::unique_ptr<LibPlot2D::Dataset2D> BuildSplineCurve(
		const unsigned int& constraintsPerSegment, const Eigen::VectorXd& xCoef, const Eigen::VectorXd& yCoef) const;
	static void CalculateAreasAndCentroids(const LibPlot2D::Dataset2D& ds,
		double& topArea, double& bottomArea, double& topCentroidArm, double& bottomCentroidArm);
	std::unique_ptr<LibPlot2D::Dataset2D> ComputeCurveData() const;

	static bool ReadGeometryInfo(const wxString& fileName, GeometryInfo& g);
	static void WriteGeometryInfo(const wxString& fileName, const GeometryInfo& g);

	bool initialized = false;

	void MainFrame::WriteRefs() const;// For debugging

	DECLARE_EVENT_TABLE();
};

#endif// MAIN_FRAME_H_
