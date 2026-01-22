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

// Worker (moc 대상: 헤더에 둬서 자동 moc 되게 함)
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

// -------------------- ScanQA Process Worker --------------------
class ScanQaWorker : public QObject
{
    Q_OBJECT
public:
    vtkSmartPointer<vtkPolyData> input;

    // params
    bool voxelEnabled = true;
    double voxelMm = 1.0;

    bool outlierEnabled = true;
    double outlierRadiusMm = 3.0;
    int outlierMinNeighbors = 5;

signals:
    void progress(int percent);
    void finished(vtkPolyData* outRaw, int removedPoints);
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
    void UpdateQaMetricsUI();

    void SetViewRaw(bool useRaw);

private:
    QVTKOpenGLNativeWidget* vtkWidget_ = nullptr;

    QLineEdit* filePathEdit_ = nullptr;
    QComboBox* modeCombo_ = nullptr;
    QSlider* pointSizeSlider_ = nullptr;
    QSlider* lineWidthSlider_ = nullptr;

    // Scan QA controls
    QComboBox* viewCombo_ = nullptr;          // Raw / Processed
    QCheckBox* voxelEnable_ = nullptr;
    QDoubleSpinBox* voxelMmSpin_ = nullptr;

    QCheckBox* outlierEnable_ = nullptr;
    QDoubleSpinBox* outlierRadiusSpin_ = nullptr;
    QSpinBox* outlierMinNbSpin_ = nullptr;

    QPushButton* applyQaBtn_ = nullptr;

    QString currentPath_;
    bool loading_ = false;
    bool processing_ = false;

    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkPolyDataMapper> mapper_;
    vtkSmartPointer<vtkActor> actor_;

    // --- Workspace UI ---
    QComboBox* workspaceCombo_ = nullptr;
    QStackedWidget* workspaceStack_ = nullptr;

    // --- Scan QA metrics UI (일단 3개만) ---
    QLabel* qaPointCountLabel_ = nullptr;
    QLabel* qaBoundsLabel_ = nullptr;
    QLabel* qaStatusLabel_ = nullptr;
    QLabel* qaRemovedLabel_ = nullptr;

    // --- DataModel 최소 (원본/처리본) ---
    vtkSmartPointer<vtkPolyData> rawCloud_;
    vtkSmartPointer<vtkPolyData> processedCloud_;
    int lastRemovedPoints_ = 0;

};
