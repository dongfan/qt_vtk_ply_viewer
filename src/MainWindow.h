#pragma once

#include <QMainWindow>
#include <QObject>
#include <QAtomicInt>
#include <QString>
#include <QHash>
#include <QVector>

class QDockWidget;
class QLabel;
class XyzPlotWidget;
#include <QPointer>
#include <QTextBrowser>

// -------------------- QA criteria help linkage --------------------
enum class QaFailReason
{
    None = 0,
    NoCloud,
    TooFewPoints,
    PlaneUnstable,
    PlaneNoisy,
    ExcessiveNoise,
    Unknown
};


#include <vtkSmartPointer.h>

class QVTKOpenGLNativeWidget;
class QLineEdit;
class QComboBox;
class QSlider;
class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;
class QPushButton;
class QStackedWidget;
class QLabel;

class vtkRenderer;
class vtkPolyDataMapper;
class vtkActor;
class vtkPolyData;
class vtkAxesActor;
class vtkOrientationMarkerWidget;
class vtkCubeAxesActor;
class vtkLookupTable;
class vtkScalarBarActor;
class vtkElevationFilter;

// -------------------- PLY Loader Worker --------------------
class PlyLoadWorker : public QObject
{
    Q_OBJECT
public:
    QString path;
    QAtomicInt* cancelFlag = nullptr;

signals:
    void progress(int percent);
    void finished(vtkPolyData* polyRaw);
    void failed(QString msg);

public slots:
    void run();
};

// -------------------- Scan QA Worker --------------------
class ScanQaWorker : public QObject
{
    Q_OBJECT
public:
    vtkSmartPointer<vtkPolyData> input;

    // Voxel
    bool voxelEnabled = true;
    double voxelMm = 1.0;

    // Outlier (Radius)
    bool outlierEnabled = true;
    double outlierRadiusMm = 3.0;
    int outlierMinNeighbors = 5;

    // Plane (RANSAC)
    bool planeEnabled = true;
    double planeThresholdMm = 2.0;
    int planeIterations = 200;
    bool planeRemoveEnabled = true; // plane  ON/OFF

signals:
    void progress(int percent);

    // removedOutlier: Outlier ŵ  
    // removedPlane: Plane ŵ  (planeRemoveEnabled  ǹ)
    // planeInlierRatio: 0~1 (planeEnabled  ǹ)
    // planeRmseMm: mm (planeEnabled  ǹ)
    void finished(vtkPolyData* outRaw,
        int removedOutlier,
        int removedPlane,
        double planeInlierRatio,
        double planeRmseMm);

    void failed(QString msg);

public slots:
    void run();
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

protected:
    void dragEnterEvent(QDragEnterEvent* e) override;
    void dropEvent(QDropEvent* e) override;

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

private:
    void CreateDockUI();

    // Data / actions
    void LoadPLYAsync(const QString& path);
    void ApplyRenderOptions();
    void UpdateRender();

    void SetWorkspaceIndex(int idx);

    // Scan QA
    void ApplyScanQaAsync();
    void SetViewRaw(bool useRaw);
    void UpdateQaMetricsUI();

    void installHelp(QObject* w, const QString& text);

    // QA criteria help linkage
    void InstallQaCriteriaHelp();
    void ShowQaCriteriaHelp();
    QString BuildQaCriteriaHelpHtml(QaFailReason reason, const QString& statusText) const;
    void FlashWidget(QWidget* w, int ms = 650);

    // XYZ graph dock
    void InstallXyzGraph();
    void UpdateXyzGraph();

    // Coordinate axes + color mapping (VTK)
    void InstallAxesAndColor();
    void UpdateAxesAndColor();
    void ApplyColorMappingForCurrentView();

private:
    // VTK view
    QVTKOpenGLNativeWidget* vtkWidget_ = nullptr;

    // Common controls
    QLineEdit* filePathEdit_ = nullptr;
    QComboBox* modeCombo_ = nullptr;
    QSlider* pointSizeSlider_ = nullptr;
    QSlider* lineWidthSlider_ = nullptr;

    // Coordinate / color controls
    QCheckBox* showAxesCheck_ = nullptr;        // corner triad
    QCheckBox* showCubeAxesCheck_ = nullptr;    // bounds axes with labels
    QComboBox* colorModeCombo_ = nullptr;       // Solid / Z-Height / PLY RGB
    QCheckBox* showScalarBarCheck_ = nullptr;   // color legend

    // VTK pipeline
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkPolyDataMapper> mapper_;
    vtkSmartPointer<vtkActor> actor_;

    // VTK: coordinate axes + color legend
    vtkSmartPointer<vtkAxesActor> axesActor_;
    vtkSmartPointer<vtkOrientationMarkerWidget> axesWidget_;
    vtkSmartPointer<vtkCubeAxesActor> cubeAxes_;
    vtkSmartPointer<vtkLookupTable> lutJet_;
    vtkSmartPointer<vtkScalarBarActor> scalarBar_;
    vtkSmartPointer<vtkElevationFilter> elevationFilter_;

    // --- Workspace UI ---
    QComboBox* workspaceCombo_ = nullptr;
    QStackedWidget* workspaceStack_ = nullptr;

    // --- Scan QA UI controls ---
    QComboBox* viewCombo_ = nullptr;          // Raw / Processed
    QCheckBox* voxelEnable_ = nullptr;
    QDoubleSpinBox* voxelMmSpin_ = nullptr;

    QCheckBox* outlierEnable_ = nullptr;
    QDoubleSpinBox* outlierRadiusSpin_ = nullptr;
    QSpinBox* outlierMinNbSpin_ = nullptr;

    QCheckBox* planeEnable_ = nullptr;
    QDoubleSpinBox* planeThresholdSpin_ = nullptr;
    QSpinBox* planeIterSpin_ = nullptr;
    QCheckBox* planeRemoveEnable_ = nullptr;

    QPushButton* applyQaBtn_ = nullptr;

    // --- Scan QA metrics UI ---
    QLabel* qaPointCountLabel_ = nullptr;
    QLabel* qaBoundsLabel_ = nullptr;
    QLabel* qaStatusLabel_ = nullptr;

    QLabel* qaRemovedOutlierLabel_ = nullptr;
    QLabel* qaRemovedPlaneLabel_ = nullptr;
    QLabel* qaPlaneInlierLabel_ = nullptr;
    QLabel* qaPlaneRmseLabel_ = nullptr;

    // --- State ---
    QString currentPath_;
    bool loading_ = false;
    bool processing_ = false;

    // --- DataModel ---
    vtkSmartPointer<vtkPolyData> rawCloud_;
    vtkSmartPointer<vtkPolyData> processedCloud_;

    int lastRemovedOutlier_ = 0;
    int lastRemovedPlane_ = 0;
    double lastPlaneInlierRatio_ = 0.0;
    double lastPlaneRmseMm_ = 0.0;

    // QA status (criteria linkage)
    QaFailReason lastQaReason_ = QaFailReason::None;
    double lastOutlierRatio_ = 0.0;
    QString lastQaStatusText_;

    // Help UI
    QTextBrowser* helpLabel_ = nullptr;

    // QA help pinning: when QA criteria help is shown, keep it visible while tweaking controls.
    bool qaHelpPinned_ = false;
    QString qaPinnedHtml_;

    // XYZ graph UI
    QDockWidget* xyzDock_ = nullptr;
    QLabel* xyzStatsLabel_ = nullptr;
    XyzPlotWidget* xyzPlot_ = nullptr;

    // help map (widget -> text)
    QHash<const QObject*, QString> helpMap_;
};
