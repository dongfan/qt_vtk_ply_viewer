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

class vtkRenderer;
class vtkPolyDataMapper;
class vtkActor;
class vtkPolyData;

class QStackedWidget;
class QLabel;

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
    void LoadPLYAsync(const QString& path);
    void ApplyRenderOptions();
    void UpdateRender();

    void SetWorkspaceIndex(int idx);
    void UpdateQaMetricsUI();

private:
    QVTKOpenGLNativeWidget* vtkWidget_ = nullptr;

    QLineEdit* filePathEdit_ = nullptr;
    QComboBox* modeCombo_ = nullptr;
    QSlider* pointSizeSlider_ = nullptr;
    QSlider* lineWidthSlider_ = nullptr;

    QString currentPath_;
    bool loading_ = false;

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

    // --- DataModel 최소 (원본/처리본) ---
    vtkSmartPointer<vtkPolyData> rawCloud_;
    vtkSmartPointer<vtkPolyData> processedCloud_;

};
