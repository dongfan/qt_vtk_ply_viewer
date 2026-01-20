#pragma once

#include <QMainWindow>

class QVTKOpenGLNativeWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onOpenPly();

private:
    void setupUi();
    void setupVtk();

    QVTKOpenGLNativeWidget* m_vtkWidget = nullptr;

    class vtkGenericOpenGLRenderWindow* m_renderWindowRaw = nullptr;
    class vtkRenderer* m_rendererRaw = nullptr;
};
