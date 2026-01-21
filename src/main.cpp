#include <QApplication>
#include <QSurfaceFormat>

#include "MainWindow.h"

static void SetupDefaultOpenGLFormat()
{
    QSurfaceFormat fmt;
    fmt.setRenderableType(QSurfaceFormat::OpenGL);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setVersion(3, 2);
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setSamples(0);
    QSurfaceFormat::setDefaultFormat(fmt);
}

int main(int argc, char* argv[])
{
    SetupDefaultOpenGLFormat();

    QApplication app(argc, argv);

    MainWindow w;
    w.show();

    return app.exec();
}
