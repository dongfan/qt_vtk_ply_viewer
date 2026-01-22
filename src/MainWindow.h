#pragma once

#include <QMainWindow>
#include <QObject>
#include <QAtomicInt>
#include <QString>

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
    bool planeRemoveEnabled = true; // plane 제거 ON/OFF

signals:
    void progress(int percent);

    // removedOutlier: Outlier 제거된 점 개수
    // removedPlane: Plane 제거된 점 개수(planeRemoveEnabled일 때만 의미)
    // planeInlierRatio: 0~1 (planeEnabled일 때만 의미)
    // planeRmseMm: mm (planeEnabled일 때만 의미)
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

private:
    // VTK view
    QVTKOpenGLNativeWidget* vtkWidget_ = nullptr;

    // Common controls
    QLineEdit* filePathEdit_ = nullptr;
    QComboBox* modeCombo_ = nullptr;
    QSlider* pointSizeSlider_ = nullptr;
    QSlider* lineWidthSlider_ = nullptr;

    // VTK pipeline
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkPolyDataMapper> mapper_;
    vtkSmartPointer<vtkActor> actor_;

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
};
