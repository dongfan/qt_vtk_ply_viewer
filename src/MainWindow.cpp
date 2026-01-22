#include "MainWindow.h"

#include <QVBoxLayout>
#include <QWidget>
#include <QDockWidget>
#include <QPushButton>
#include <QFileDialog>
#include <QLineEdit>
#include <QLabel>
#include <QSlider>
#include <QComboBox>
#include <QMessageBox>
#include <QMimeData>
#include <QDragEnterEvent>
#include <QDropEvent>

#include <QThread>
#include <QProgressDialog>
#include <QPointer>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QGroupBox>
#include <QFormLayout>
#include <QHBoxLayout>

#include <vtkKdTreePointLocator.h>
#include <vtkIdList.h>

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

#include <vtkPLYReader.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkIdTypeArray.h>

#include <QStackedWidget>
#include <QGroupBox>
#include <QFormLayout>

namespace {

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

            if ((i & 0x3FFF) == 0) {
                int p = static_cast<int>((i * 90) / vertexCount);
                onProgress(p);
            }
        }

        // vertices cell array (legacy style)
        vtkNew<vtkIdTypeArray> conn;
        conn->SetNumberOfComponents(1);
        conn->SetNumberOfTuples(static_cast<vtkIdType>(vertexCount) * 2);

        auto* ptr = conn->WritePointer(0, static_cast<vtkIdType>(vertexCount) * 2);
        for (long long i = 0; i < vertexCount; ++i) {
            if (isCanceled()) throw std::runtime_error("Canceled");
            ptr[2 * i] = 1;
            ptr[2 * i + 1] = static_cast<vtkIdType>(i);

            if ((i & 0x3FFF) == 0) {
                int p = 90 + static_cast<int>((i * 9) / vertexCount);
                onProgress(p);
            }
        }

        vtkNew<vtkCellArray> verts;
        verts->SetData(1, conn);

        vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
        poly->SetPoints(points);
        poly->SetVerts(verts);

        onProgress(99);
        return poly;
    }

} // namespace

// -------------------- PlyLoadWorker --------------------
void PlyLoadWorker::run()
{
    try {
        auto poly = LoadPlySafe_PCLBinary_Progress(
            path.toStdString(),
            [&](int p) { emit progress(p); },
            [&]() { return cancelFlag && cancelFlag->loadRelaxed() != 0; }
        );

        if (!poly || poly->GetNumberOfPoints() == 0)
            throw std::runtime_error("No points found");

        vtkPolyData* raw = poly.GetPointer();
        raw->Register(nullptr);      // refcount 1 보장해서 넘김
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

namespace {

    // 간단 voxel downsample (격자 버킷팅: 각 voxel당 첫 점만 유지)
    static vtkSmartPointer<vtkPolyData> VoxelDownsample(vtkPolyData* in, double voxelMm)
    {
        if (!in || !in->GetPoints() || voxelMm <= 0.0) return in;

        const double inv = 1.0 / voxelMm;

        struct Key {
            int x, y, z;
            bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
        };
        struct KeyHash {
            size_t operator()(const Key& k) const noexcept {
                // 3D int hash
                size_t h = 1469598103934665603ull;
                auto mix = [&](int v) {
                    h ^= (size_t)v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
                    };
                mix(k.x); mix(k.y); mix(k.z);
                return h;
            }
        };

        std::unordered_map<Key, vtkIdType, KeyHash> seen;
        seen.reserve((size_t)in->GetNumberOfPoints() / 4 + 1);

        vtkNew<vtkPoints> pts;
        pts->SetDataTypeToFloat();

        // verts cell array (point cloud)
        vtkNew<vtkIdTypeArray> conn;
        conn->SetNumberOfComponents(1);

        vtkIdType outCount = 0;
        double p[3];

        for (vtkIdType i = 0; i < in->GetNumberOfPoints(); ++i) {
            in->GetPoint(i, p);
            Key k{
                (int)std::floor(p[0] * inv),
                (int)std::floor(p[1] * inv),
                (int)std::floor(p[2] * inv)
            };
            if (seen.find(k) != seen.end()) continue;
            seen.emplace(k, i);

            pts->InsertNextPoint(p);
            outCount++;
        }

        conn->SetNumberOfTuples(outCount * 2);
        auto* ptr = conn->WritePointer(0, outCount * 2);
        for (vtkIdType i = 0; i < outCount; ++i) {
            ptr[2 * i] = 1;
            ptr[2 * i + 1] = i;
        }
        vtkNew<vtkCellArray> verts;
        verts->SetData(1, conn);

        auto out = vtkSmartPointer<vtkPolyData>::New();
        out->SetPoints(pts);
        out->SetVerts(verts);
        return out;
    }

    // Radius outlier removal (KDTree로 반경 내 이웃 수가 minNeighbors 미만이면 제거)
    static vtkSmartPointer<vtkPolyData> RadiusOutlierRemoval(vtkPolyData* in, double radiusMm, int minNeighbors)
    {
        if (!in || !in->GetPoints() || radiusMm <= 0.0) return in;
        if (minNeighbors <= 0) return in;

        vtkNew<vtkKdTreePointLocator> kdtree;
        kdtree->SetDataSet(in);
        kdtree->BuildLocator();

        vtkNew<vtkPoints> pts;
        pts->SetDataTypeToFloat();

        vtkNew<vtkIdList> ids;

        double p[3];
        for (vtkIdType i = 0; i < in->GetNumberOfPoints(); ++i) {
            in->GetPoint(i, p);

            ids->Reset();
            kdtree->FindPointsWithinRadius(radiusMm, p, ids);

            // ids에는 자기 자신 포함될 수 있음. minNeighbors는 "자기 포함" 기준으로 잡는게 UI에서 직관적.
            if (ids->GetNumberOfIds() >= minNeighbors) {
                pts->InsertNextPoint(p);
            }
        }

        const vtkIdType outCount = pts->GetNumberOfPoints();

        vtkNew<vtkIdTypeArray> conn;
        conn->SetNumberOfComponents(1);
        conn->SetNumberOfTuples(outCount * 2);

        auto* ptr = conn->WritePointer(0, outCount * 2);
        for (vtkIdType i = 0; i < outCount; ++i) {
            ptr[2 * i] = 1;
            ptr[2 * i + 1] = i;
        }

        vtkNew<vtkCellArray> verts;
        verts->SetData(1, conn);

        auto out = vtkSmartPointer<vtkPolyData>::New();
        out->SetPoints(pts);
        out->SetVerts(verts);
        return out;
    }

} // namespace

void ScanQaWorker::run()
{
    try {
        if (!input || input->GetNumberOfPoints() <= 0)
            throw std::runtime_error("No cloud loaded");

        emit progress(5);

        vtkSmartPointer<vtkPolyData> cur = input;

        // 1) voxel
        if (voxelEnabled) {
            cur = VoxelDownsample(cur, voxelMm);
        }
        emit progress(55);

        // 2) outlier
        vtkIdType before = cur->GetNumberOfPoints();
        if (outlierEnabled) {
            cur = RadiusOutlierRemoval(cur, outlierRadiusMm, outlierMinNeighbors);
        }
        vtkIdType after = cur->GetNumberOfPoints();
        int removed = (int)std::max<vtkIdType>(0, before - after);

        emit progress(95);

        vtkPolyData* raw = cur.GetPointer();
        raw->Register(nullptr); // refcount 확보
        emit finished(raw, removed);

        emit progress(100);
    }
    catch (const std::exception& e) {
        emit failed(e.what());
    }
    catch (...) {
        emit failed("Unknown error");
    }
}

// -------------------- MainWindow --------------------
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
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

void MainWindow::dragEnterEvent(QDragEnterEvent* e)
{
    if (e->mimeData()->hasUrls())
        e->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent* e)
{
    const auto urls = e->mimeData()->urls();
    if (urls.isEmpty()) return;

    const QString path = urls.first().toLocalFile();
    if (path.endsWith(".ply", Qt::CaseInsensitive)) {
        LoadPLYAsync(path);
        e->acceptProposedAction();
    }
}

void MainWindow::CreateDockUI()
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

    // -------------------- Scan QA Panel --------------------
    {
        auto* qaGroup = new QGroupBox("Scan QA (Preprocess)", panel);
        auto* form = new QFormLayout(qaGroup);

        // View: Raw / Processed
        viewCombo_ = new QComboBox(panel);
        viewCombo_->addItem("Raw");
        viewCombo_->addItem("Processed");
        viewCombo_->setCurrentIndex(0);
        form->addRow("View:", viewCombo_);

        connect(viewCombo_, &QComboBox::currentIndexChanged, this, [this](int idx) {
            SetViewRaw(idx == 0);
            });

        // Voxel
        voxelEnable_ = new QCheckBox("Enable", panel);
        voxelEnable_->setChecked(true);

        voxelMmSpin_ = new QDoubleSpinBox(panel);
        voxelMmSpin_->setRange(0.1, 50.0);
        voxelMmSpin_->setValue(1.0);
        voxelMmSpin_->setSingleStep(0.1);
        voxelMmSpin_->setSuffix(" mm");

        auto* voxelRow = new QWidget(panel);
        auto* voxelH = new QHBoxLayout(voxelRow);
        voxelH->setContentsMargins(0, 0, 0, 0);
        voxelH->addWidget(voxelEnable_);
        voxelH->addWidget(voxelMmSpin_, 1);
        form->addRow("Voxel:", voxelRow);

        // Outlier (Radius)
        outlierEnable_ = new QCheckBox("Enable", panel);
        outlierEnable_->setChecked(true);

        outlierRadiusSpin_ = new QDoubleSpinBox(panel);
        outlierRadiusSpin_->setRange(0.1, 200.0);
        outlierRadiusSpin_->setValue(3.0);
        outlierRadiusSpin_->setSingleStep(0.1);
        outlierRadiusSpin_->setSuffix(" mm");

        outlierMinNbSpin_ = new QSpinBox(panel);
        outlierMinNbSpin_->setRange(1, 200);
        outlierMinNbSpin_->setValue(5);

        auto* outRow = new QWidget(panel);
        auto* outH = new QHBoxLayout(outRow);
        outH->setContentsMargins(0, 0, 0, 0);
        outH->addWidget(outlierEnable_);
        outH->addWidget(new QLabel("Radius:", panel));
        outH->addWidget(outlierRadiusSpin_, 1);
        outH->addWidget(new QLabel("MinN:", panel));
        outH->addWidget(outlierMinNbSpin_);
        form->addRow("Outlier:", outRow);

        applyQaBtn_ = new QPushButton("Apply Scan QA", panel);
        form->addRow(applyQaBtn_);

        connect(applyQaBtn_, &QPushButton::clicked, this, [this]() {
            ApplyScanQaAsync();
            });

        // Metrics
        qaPointCountLabel_ = new QLabel("-", panel);
        qaBoundsLabel_ = new QLabel("-", panel);
        qaRemovedLabel_ = new QLabel("-", panel);

        form->addRow("Points:", qaPointCountLabel_);
        form->addRow("Bounds (X/Y/Z):", qaBoundsLabel_);
        form->addRow("Removed (last):", qaRemovedLabel_);

        qaGroup->setLayout(form);
        v->addWidget(qaGroup);
    }

    v->addSpacing(10);


    v->addWidget(new QLabel("Display Mode:", panel));
    modeCombo_ = new QComboBox(panel);
    modeCombo_->addItem("Surface (Default)");
    modeCombo_->addItem("Wireframe");
    modeCombo_->addItem("Points");
    modeCombo_->setCurrentIndex(2);
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

    v->addWidget(new QLabel("Note: Triangulate/Normals are disabled for point clouds (to avoid freeze).", panel));
    v->addStretch(1);

    // -------------------- Workspace selector --------------------
    v->addWidget(new QLabel("Workspace:", panel));

    workspaceCombo_ = new QComboBox(panel);
    workspaceCombo_->addItem("Scan QA");
    workspaceCombo_->addItem("Inspection");
    workspaceCombo_->addItem("Robot Pose");
    workspaceCombo_->setCurrentIndex(0);
    v->addWidget(workspaceCombo_);

    // workspace-specific panels container
    workspaceStack_ = new QStackedWidget(panel);
    v->addWidget(workspaceStack_);

    connect(workspaceCombo_, &QComboBox::currentIndexChanged, this, [this](int idx) {
        SetWorkspaceIndex(idx);
        });

    // -------------------- Page 0: Scan QA --------------------
    {
        auto* page = new QWidget(panel);
        auto* pv = new QVBoxLayout(page);

        auto* box = new QGroupBox("Scan QA Metrics", page);
        auto* form = new QFormLayout(box);

        qaPointCountLabel_ = new QLabel("-", box);
        qaBoundsLabel_ = new QLabel("-", box);
        qaStatusLabel_ = new QLabel("Load a PLY to see metrics.", box);

        form->addRow("Points:", qaPointCountLabel_);
        form->addRow("Bounds (X/Y/Z mm):", qaBoundsLabel_);
        form->addRow("Status:", qaStatusLabel_);

        box->setLayout(form);
        pv->addWidget(box);
        pv->addStretch(1);

        workspaceStack_->addWidget(page);
    }

    // -------------------- Page 1: Inspection (placeholder) --------------------
    {
        auto* page = new QWidget(panel);
        auto* pv = new QVBoxLayout(page);
        pv->addWidget(new QLabel("Inspection workspace: (next step) Align + Measure tools", page));
        pv->addStretch(1);
        workspaceStack_->addWidget(page);
    }

    // -------------------- Page 2: Robot Pose (placeholder) --------------------
    {
        auto* page = new QWidget(panel);
        auto* pv = new QVBoxLayout(page);
        pv->addWidget(new QLabel("Robot Pose workspace: (later) ROI + Normal + Pose + Export", page));
        pv->addStretch(1);
        workspaceStack_->addWidget(page);
    }

    // initial workspace
    SetWorkspaceIndex(0);

    // divider spacing
    v->addSpacing(10);

    panel->setLayout(v);
    dock->setWidget(panel);
    addDockWidget(Qt::LeftDockWidgetArea, dock);
}

void MainWindow::LoadPLYAsync(const QString& path)
{
    currentPath_ = path;
    filePathEdit_->setText(path);

    if (loading_) {
        QMessageBox::information(this, "Loading", "Already loading. Please wait or cancel.");
        return;
    }
    loading_ = true;

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
        if (dlg) { dlg->setValue(100); dlg->close(); dlg->deleteLater(); }

        vtkSmartPointer<vtkPolyData> poly;
        poly.TakeReference(raw);

        rawCloud_ = poly;
        processedCloud_ = poly;   // 지금은 전처리 전이라 동일
        UpdateQaMetricsUI();

        rawCloud_ = poly;
        processedCloud_ = poly;
        lastRemovedPoints_ = 0;

        mapper_->SetInputData(rawCloud_);
        mapper_->ScalarVisibilityOff();

        UpdateQaMetricsUI();
        if (viewCombo_) viewCombo_->setCurrentIndex(0); // Raw로 보기

        renderer_->ResetCamera();
        ApplyRenderOptions();
        UpdateRender();

        loading_ = false;
        worker->deleteLater();
        th->quit();
        th->deleteLater();
        delete cancelFlag;
        }, Qt::QueuedConnection);

    connect(worker, &PlyLoadWorker::failed, this, [=](const QString& msg) {
        if (dlg) { dlg->close(); dlg->deleteLater(); }
        loading_ = false;

        if (!msg.contains("Canceled", Qt::CaseInsensitive)) {
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

void MainWindow::ApplyRenderOptions()
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

void MainWindow::UpdateRender()
{
    if (vtkWidget_ && vtkWidget_->renderWindow())
        vtkWidget_->renderWindow()->Render();
}

void MainWindow::SetWorkspaceIndex(int idx)
{
    if (workspaceStack_) workspaceStack_->setCurrentIndex(idx);
}

void MainWindow::SetViewRaw(bool useRaw)
{
    if (!mapper_) return;
    if (useRaw) {
        if (rawCloud_) mapper_->SetInputData(rawCloud_);
    }
    else {
        if (processedCloud_) mapper_->SetInputData(processedCloud_);
    }
    UpdateRender();
}

void MainWindow::ApplyScanQaAsync()
{
    if (processing_) return;
    if (!rawCloud_ || rawCloud_->GetNumberOfPoints() <= 0) {
        QMessageBox::information(this, "Scan QA", "Load a PLY first.");
        return;
    }

    processing_ = true;

    QPointer<QProgressDialog> dlg = new QProgressDialog("Applying Scan QA...", "Cancel", 0, 100, this);
    dlg->setWindowModality(Qt::ApplicationModal);
    dlg->setMinimumDuration(0);
    dlg->setValue(0);
    dlg->show();

    auto cancelFlag = new QAtomicInt(0);

    QThread* th = new QThread(this);
    auto* worker = new ScanQaWorker();
    worker->input = rawCloud_;

    worker->voxelEnabled = voxelEnable_ && voxelEnable_->isChecked();
    worker->voxelMm = voxelMmSpin_ ? voxelMmSpin_->value() : 1.0;

    worker->outlierEnabled = outlierEnable_ && outlierEnable_->isChecked();
    worker->outlierRadiusMm = outlierRadiusSpin_ ? outlierRadiusSpin_->value() : 3.0;
    worker->outlierMinNeighbors = outlierMinNbSpin_ ? outlierMinNbSpin_->value() : 5;

    worker->moveToThread(th);

    connect(dlg, &QProgressDialog::canceled, this, [=]() {
        cancelFlag->storeRelease(1); // (현재 worker는 cancelFlag를 사용하지 않지만, 확장 대비)
        dlg->setLabelText("Canceling...");
        th->requestInterruption();
        });

    connect(worker, &ScanQaWorker::progress, this, [=](int p) {
        if (dlg) dlg->setValue(p);
        }, Qt::QueuedConnection);

    connect(worker, &ScanQaWorker::finished, this, [=](vtkPolyData* outRaw, int removed) {
        if (dlg) { dlg->setValue(100); dlg->close(); dlg->deleteLater(); }

        vtkSmartPointer<vtkPolyData> out;
        out.TakeReference(outRaw);

        processedCloud_ = out;
        lastRemovedPoints_ = removed;

        // Processed로 보기 전환
        if (viewCombo_) viewCombo_->setCurrentIndex(1);
        mapper_->SetInputData(processedCloud_);
        mapper_->ScalarVisibilityOff();

        UpdateQaMetricsUI();
        UpdateRender();

        processing_ = false;
        worker->deleteLater();
        th->quit();
        th->deleteLater();
        delete cancelFlag;
        }, Qt::QueuedConnection);

    connect(worker, &ScanQaWorker::failed, this, [=](const QString& msg) {
        if (dlg) { dlg->close(); dlg->deleteLater(); }
        QMessageBox::critical(this, "Scan QA Failed", msg);

        processing_ = false;
        worker->deleteLater();
        th->quit();
        th->deleteLater();
        delete cancelFlag;
        }, Qt::QueuedConnection);

    connect(th, &QThread::started, worker, &ScanQaWorker::run);
    th->start();
}

void MainWindow::UpdateQaMetricsUI()
{
    if (!qaPointCountLabel_ || !qaBoundsLabel_ || !qaRemovedLabel_) return;

    vtkPolyData* cloud = (viewCombo_ && viewCombo_->currentIndex() == 0) ? rawCloud_.GetPointer() : processedCloud_.GetPointer();
    if (!cloud || cloud->GetNumberOfPoints() <= 0) {
        qaPointCountLabel_->setText("-");
        qaBoundsLabel_->setText("-");
        qaRemovedLabel_->setText("-");
        return;
    }

    const vtkIdType n = cloud->GetNumberOfPoints();
    double b[6]; cloud->GetBounds(b);
    const double sx = b[1] - b[0], sy = b[3] - b[2], sz = b[5] - b[4];

    qaPointCountLabel_->setText(QString::number((long long)n));
    qaBoundsLabel_->setText(QString("%1 / %2 / %3 mm")
        .arg(sx, 0, 'f', 2)
        .arg(sy, 0, 'f', 2)
        .arg(sz, 0, 'f', 2));

    qaRemovedLabel_->setText(QString::number(lastRemovedPoints_));
}
