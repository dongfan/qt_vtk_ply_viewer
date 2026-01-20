#include <QApplication>
#include <QMainWindow>
#include <QSurfaceFormat>
#include <QVBoxLayout>
#include <QWidget>
#include <QDockWidget>
#include <QPushButton>
#include <QFileDialog>
#include <QLineEdit>
#include <QLabel>
#include <QCheckBox>
#include <QSlider>
#include <QComboBox>
#include <QMessageBox>
#include <QMimeData>
#include <QDragEnterEvent>
#include <QDropEvent>

#include <QThread>
#include <QProgressDialog>
#include <QAtomicInt>
#include <QPointer>

#include <functional>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <cstring>

#include <QVTKOpenGLNativeWidget.h>

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkIdTypeArray.h>

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

// -------------------- Safe PLY Loader (PCL binary_little_endian) --------------------
static vtkSmartPointer<vtkPolyData> LoadPlySafe_PCLBinary_Progress(
    const std::string& path,
    const std::function<void(int /*percent*/)>& onProgress,
    const std::function<bool(void)>& isCanceled
)
{
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) throw std::runtime_error("Cannot open PLY: " + path);

    std::string line;
    bool headerEnded = false;
    bool isBinaryLE = false;

    long long vertexCount = -1;
    bool inVertexElement = false;

    struct Prop { std::string type; std::string name; };
    std::vector<Prop> vertexProps;

    auto trim = [](std::string s) {
        while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
        size_t i = 0; while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) i++;
        return s.substr(i);
        };

    // --- parse header ---
    while (std::getline(ifs, line))
    {
        if (isCanceled()) throw std::runtime_error("Canceled");

        line = trim(line);
        if (line.rfind("format ", 0) == 0) {
            if (line.find("binary_little_endian") != std::string::npos) isBinaryLE = true;
        }
        else if (line.rfind("element vertex ", 0) == 0) {
            vertexCount = std::stoll(line.substr(std::string("element vertex ").size()));
            inVertexElement = true;
            vertexProps.clear();
        }
        else if (line.rfind("element ", 0) == 0) {
            if (line.rfind("element vertex ", 0) != 0) inVertexElement = false;
        }
        else if (inVertexElement && line.rfind("property ", 0) == 0) {
            std::istringstream iss(line);
            std::string tok, type, name;
            iss >> tok >> type >> name;
            if (!type.empty() && !name.empty()) vertexProps.push_back({ type, name });
        }
        else if (line == "end_header") {
            headerEnded = true;
            break;
        }
    }

    if (!headerEnded) throw std::runtime_error("PLY header end_header missing");
    if (!isBinaryLE) throw std::runtime_error("Only binary_little_endian supported by this safe loader");
    if (vertexCount <= 0) throw std::runtime_error("Invalid vertex count");
    if (vertexProps.empty()) throw std::runtime_error("No vertex properties in header");

    auto typeSize = [](const std::string& t)->size_t {
        if (t == "char" || t == "int8") return 1;
        if (t == "uchar" || t == "uint8") return 1;
        if (t == "short" || t == "int16") return 2;
        if (t == "ushort" || t == "uint16") return 2;
        if (t == "int" || t == "int32") return 4;
        if (t == "uint" || t == "uint32") return 4;
        if (t == "float" || t == "float32") return 4;
        if (t == "double" || t == "float64") return 8;
        return 0;
        };

    std::unordered_map<std::string, size_t> offset;
    std::unordered_map<std::string, std::string> ptype;
    size_t stride = 0;

    for (auto& p : vertexProps) {
        size_t sz = typeSize(p.type);
        if (sz == 0) throw std::runtime_error("Unsupported PLY property type: " + p.type);
        offset[p.name] = stride;
        ptype[p.name] = p.type;
        stride += sz;
    }

    if (!offset.count("x") || !offset.count("y") || !offset.count("z"))
        throw std::runtime_error("PLY must contain x/y/z properties");

    std::vector<unsigned char> buf(stride);

    vtkNew<vtkPoints> points;
    points->SetDataTypeToFloat();
    points->SetNumberOfPoints(static_cast<vtkIdType>(vertexCount));

    auto readFloatAt = [&](const std::string& name)->float {
        const auto& t = ptype[name];
        const size_t off = offset[name];

        if (t == "float" || t == "float32") {
            float v; std::memcpy(&v, buf.data() + off, sizeof(float));
            return v;
        }
        if (t == "double" || t == "float64") {
            double v; std::memcpy(&v, buf.data() + off, sizeof(double));
            return static_cast<float>(v);
        }
        if (t == "int" || t == "int32") {
            int32_t v; std::memcpy(&v, buf.data() + off, 4);
            return static_cast<float>(v);
        }
        if (t == "uint" || t == "uint32") {
            uint32_t v; std::memcpy(&v, buf.data() + off, 4);
            return static_cast<float>(v);
        }
        if (t == "short" || t == "int16") {
            int16_t v; std::memcpy(&v, buf.data() + off, 2);
            return static_cast<float>(v);
        }
        if (t == "ushort" || t == "uint16") {
            uint16_t v; std::memcpy(&v, buf.data() + off, 2);
            return static_cast<float>(v);
        }
        if (t == "char" || t == "int8") {
            int8_t v; std::memcpy(&v, buf.data() + off, 1);
            return static_cast<float>(v);
        }
        if (t == "uchar" || t == "uint8") {
            uint8_t v; std::memcpy(&v, buf.data() + off, 1);
            return static_cast<float>(v);
        }
        throw std::runtime_error("Unsupported numeric type for " + name + ": " + t);
        };

    // ---- read vertices (0~90%) ----
    for (long long i = 0; i < vertexCount; ++i) {
        if (isCanceled()) throw std::runtime_error("Canceled");

        ifs.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(stride));
        if (!ifs) throw std::runtime_error("Unexpected EOF while reading vertices");

        const float x = readFloatAt("x");
        const float y = readFloatAt("y");
        const float z = readFloatAt("z");
        points->SetPoint(static_cast<vtkIdType>(i), x, y, z);

        if ((i & 0x3FFF) == 0) { // 16384 포인트마다 갱신
            int p = static_cast<int>((i * 90) / vertexCount);
            onProgress(p);
        }
    }
    onProgress(90);

    // ---- build verts connectivity (90~99%) ----
    // vtkCellArray를 빠르게 만들기 위해 connectivity 배열(2N: [1,0, 1,1, ...]) 구성
    vtkNew<vtkIdTypeArray> conn;
    conn->SetNumberOfValues(static_cast<vtkIdType>(vertexCount * 2));

    vtkIdType* ptr = conn->GetPointer(0);
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(vertexCount); ++i) {
        if (isCanceled()) throw std::runtime_error("Canceled");

        ptr[2 * i] = 1;
        ptr[2 * i + 1] = i;

        if ((i & 0x3FFF) == 0) {
            int p = 90 + static_cast<int>((i * 9) / vertexCount);
            onProgress(p);
        }
    }

    vtkNew<vtkCellArray> verts;
    verts->SetData(1, conn); // legacy-style: (numIds, id...) 반복

    vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(points);
    poly->SetVerts(verts);
    onProgress(99);
    return poly;
}

// -------------------- Worker (must be top-level for moc) --------------------
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
    void run()
    {
        try {
            auto poly = LoadPlySafe_PCLBinary_Progress(
                path.toStdString(),
                [&](int p) { emit progress(p); },
                [&]() { return cancelFlag && cancelFlag->loadRelaxed() != 0; }
            );

            if (!poly || poly->GetNumberOfPoints() == 0)
                throw std::runtime_error("No points found");

            // raw 전달 (refcount 1 보장)
            vtkPolyData* raw = poly.GetPointer();
            raw->Register(nullptr);
            poly = nullptr;
            emit finished(raw);
        }
        catch (const std::exception& e) {
            emit failed(e.what());
        }
        catch (...) {
            emit failed("Unknown error");
        }
    }
};

// -------------------- MainWindow --------------------
class MainWindow : public QMainWindow
{
public:
    MainWindow()
    {
        setWindowTitle("Qt + VTK PLY Viewer");
        resize(1200, 850);
        setAcceptDrops(true);

        auto* central = new QWidget(this);
        auto* layout = new QVBoxLayout(central);
        layout->setContentsMargins(0, 0, 0, 0);

        vtkWidget_ = new QVTKOpenGLNativeWidget(central);
        layout->addWidget(vtkWidget_);
        setCentralWidget(central);

        vtkNew<vtkGenericOpenGLRenderWindow> rw;
        vtkWidget_->setRenderWindow(rw);

        renderer_ = vtkSmartPointer<vtkRenderer>::New();
        renderer_->SetBackground(0.15, 0.15, 0.18);
        vtkWidget_->renderWindow()->AddRenderer(renderer_);

        vtkNew<vtkInteractorStyleTrackballCamera> style;
        vtkWidget_->renderWindow()->GetInteractor()->SetInteractorStyle(style);

        mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
        actor_ = vtkSmartPointer<vtkActor>::New();
        actor_->SetMapper(mapper_);
        actor_->GetProperty()->SetInterpolationToPhong();
        actor_->GetProperty()->SetColor(0.9, 0.9, 0.9);

        renderer_->AddActor(actor_);

        CreateDockUI();
        ApplyRenderOptions();
        UpdateRender();
    }

protected:
    void dragEnterEvent(QDragEnterEvent* e) override
    {
        if (e->mimeData()->hasUrls())
            e->acceptProposedAction();
    }

    void dropEvent(QDropEvent* e) override
    {
        const auto urls = e->mimeData()->urls();
        if (urls.isEmpty()) return;

        const QString path = urls.first().toLocalFile();
        if (path.endsWith(".ply", Qt::CaseInsensitive))
        {
            LoadPLYAsync(path);
            e->acceptProposedAction();
        }
    }

private:
    void CreateDockUI()
    {
        auto* dock = new QDockWidget("Controls", this);
        dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

        auto* panel = new QWidget(dock);
        auto* v = new QVBoxLayout(panel);

        v->addWidget(new QLabel("PLY File:", panel));

        filePathEdit_ = new QLineEdit(panel);
        filePathEdit_->setReadOnly(true);
        v->addWidget(filePathEdit_);

        auto* openBtn = new QPushButton("Open PLY...", panel);
        v->addWidget(openBtn);
        connect(openBtn, &QPushButton::clicked, this, [this]() {
            const QString path = QFileDialog::getOpenFileName(this, "Open PLY", QString(), "PLY files (*.ply)");
            if (!path.isEmpty())
                LoadPLYAsync(path);
            });

        auto* resetBtn = new QPushButton("Reset Camera", panel);
        v->addWidget(resetBtn);
        connect(resetBtn, &QPushButton::clicked, this, [this]() {
            renderer_->ResetCamera();
            UpdateRender();
            });

        v->addSpacing(10);

        v->addWidget(new QLabel("Display Mode:", panel));
        modeCombo_ = new QComboBox(panel);
        modeCombo_->addItem("Surface (Default)");
        modeCombo_->addItem("Wireframe");
        modeCombo_->addItem("Points");
        modeCombo_->setCurrentIndex(2); // 포인트클라우드는 기본 Points가 안전
        v->addWidget(modeCombo_);
        connect(modeCombo_, &QComboBox::currentIndexChanged, this, [this](int) {
            ApplyRenderOptions();
            });

        v->addWidget(new QLabel("Point Size:", panel));
        pointSizeSlider_ = new QSlider(Qt::Horizontal, panel);
        pointSizeSlider_->setRange(1, 15);
        pointSizeSlider_->setValue(3);
        v->addWidget(pointSizeSlider_);
        connect(pointSizeSlider_, &QSlider::valueChanged, this, [this](int) {
            ApplyRenderOptions();
            });

        v->addWidget(new QLabel("Line Width:", panel));
        lineWidthSlider_ = new QSlider(Qt::Horizontal, panel);
        lineWidthSlider_->setRange(1, 10);
        lineWidthSlider_->setValue(1);
        v->addWidget(lineWidthSlider_);
        connect(lineWidthSlider_, &QSlider::valueChanged, this, [this](int) {
            ApplyRenderOptions();
            });

        // ⚠️ 포인트클라우드에서는 이 두 옵션을 당분간 “없애는” 게 맞습니다.
        // 멈춤/크래시의 80%가 여기서 납니다.
        // (나중에 "mesh ply" 들어왔을 때만 활성화하는 구조로 확장하면 됩니다.)
        v->addWidget(new QLabel("Note: Triangulate/Normals are disabled for point clouds (to avoid freeze).", panel));

        v->addStretch(1);
        panel->setLayout(v);
        dock->setWidget(panel);
        addDockWidget(Qt::LeftDockWidgetArea, dock);
    }

    void LoadPLYAsync(const QString& path)
    {
        currentPath_ = path;
        filePathEdit_->setText(path);

        // 이미 로딩 중이면 막기
        if (loading_) {
            QMessageBox::information(this, "Loading", "Already loading. Please wait or cancel.");
            return;
        }
        loading_ = true;

        // Progress dialog
        QPointer<QProgressDialog> dlg = new QProgressDialog("Loading PLY...", "Cancel", 0, 100, this);
        dlg->setWindowModality(Qt::ApplicationModal);
        dlg->setMinimumDuration(0);
        dlg->setValue(0);
        dlg->show();

        auto cancelFlag = new QAtomicInt(0);

        QThread* th = new QThread(this);
        PlyLoadWorker* worker = new PlyLoadWorker();
        worker->path = path;
        worker->cancelFlag = cancelFlag;
        worker->moveToThread(th);

        connect(dlg, &QProgressDialog::canceled, this, [=]() {
            cancelFlag->storeRelease(1);
            dlg->setLabelText("Canceling...");
            th->requestInterruption();
            });

        connect(worker, &PlyLoadWorker::progress, this, [=](int p) {
            if (dlg) dlg->setValue(p);
            }, Qt::QueuedConnection);

        connect(worker, &PlyLoadWorker::finished, this, [=](vtkPolyData* raw) {
            // close dialog
            if (dlg) { dlg->setValue(100); dlg->close(); dlg->deleteLater(); }

            vtkSmartPointer<vtkPolyData> poly;
            poly.TakeReference(raw);

            // ✅ UI 스레드에서는 "바로 붙이고 렌더"만 한다 (여기서 무거운 필터 금지)
            mapper_->SetInputData(poly);
            mapper_->ScalarVisibilityOff();

            renderer_->ResetCamera();
            ApplyRenderOptions();
            UpdateRender();

            // cleanup
            loading_ = false;
            worker->deleteLater();
            th->quit();
            th->deleteLater();
            delete cancelFlag;
            }, Qt::QueuedConnection);

        connect(worker, &PlyLoadWorker::failed, this, [=](const QString& msg) {
            if (dlg) { dlg->close(); dlg->deleteLater(); }
            loading_ = false;

            // cancel이면 조용히 종료
            if (msg.contains("Canceled", Qt::CaseInsensitive)) {
                // no popup
            }
            else {
                QMessageBox::critical(this, "PLY Load Failed", msg);
            }

            worker->deleteLater();
            th->quit();
            th->deleteLater();
            delete cancelFlag;
            }, Qt::QueuedConnection);

        connect(th, &QThread::started, worker, &PlyLoadWorker::run);
        th->start();
    }

    void ApplyRenderOptions()
    {
        const int mode = modeCombo_->currentIndex();
        auto* prop = actor_->GetProperty();

        prop->SetPointSize(pointSizeSlider_->value());
        prop->SetLineWidth(lineWidthSlider_->value());

        if (mode == 0) prop->SetRepresentationToSurface();
        else if (mode == 1) prop->SetRepresentationToWireframe();
        else prop->SetRepresentationToPoints();

        UpdateRender();
    }

    void UpdateRender()
    {
        if (vtkWidget_ && vtkWidget_->renderWindow())
            vtkWidget_->renderWindow()->Render();
    }

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
};

int main(int argc, char* argv[])
{
    SetupDefaultOpenGLFormat();
    QApplication app(argc, argv);

    MainWindow w;
    w.show();

    return app.exec();
}

#include "main.moc"
