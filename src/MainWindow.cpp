#include "MainWindow.h"

#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>

#include <QVTKOpenGLNativeWidget.h>

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <vtkPLYReader.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setupUi();
    setupVtk();
}

void MainWindow::setupUi()
{
    auto* fileMenu = menuBar()->addMenu(tr("File"));
    auto* openAct = fileMenu->addAction(tr("Open PLY..."));
    connect(openAct, &QAction::triggered, this, &MainWindow::onOpenPly);

    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
    setCentralWidget(m_vtkWidget);
}

void MainWindow::setupVtk()
{
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_renderWindowRaw = renderWindow;

    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    m_rendererRaw = renderer;

    renderWindow->AddRenderer(renderer);
    m_vtkWidget->setRenderWindow(renderWindow);

    auto style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    m_vtkWidget->renderWindow()->GetInteractor()->SetInteractorStyle(style);

    renderer->SetBackground(0.12, 0.12, 0.14);
    m_vtkWidget->renderWindow()->Render();
}

void MainWindow::onOpenPly()
{
    const QString path = QFileDialog::getOpenFileName(
        this,
        tr("Open PLY"),
        QString(),
        tr("PLY Files (*.ply)")
    );
    if (path.isEmpty())
        return;

    auto reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(path.toStdString().c_str());
    reader->Update();

    vtkPolyData* poly = reader->GetOutput();
    if (!poly || poly->GetNumberOfPoints() == 0) {
        QMessageBox::warning(this, tr("Load Failed"),
                             tr("PLY에 포인트가 없거나 읽기 실패했습니다."));
        return;
    }

    auto glyph = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyph->SetInputData(poly);
    glyph->Update();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyph->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(2.0);

    m_rendererRaw->RemoveAllViewProps();
    m_rendererRaw->AddActor(actor);
    m_rendererRaw->ResetCamera();

    statusBar()->showMessage(tr("Loaded: %1 points").arg(poly->GetNumberOfPoints()));
    m_vtkWidget->renderWindow()->Render();
}
