#include "MainWindow.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
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
#include <QProgressDialog>
#include <QPointer>
#include <QThread>
#include <QGroupBox>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QStackedWidget>

#include <functional>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <random>
#include <cmath>
#include <algorithm>

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

#include <vtkKdTreePointLocator.h>
#include <vtkIdList.h>

namespace {

    // -------------------- Safe PLY Loader (binary_little_endian; point cloud) --------------------
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
        if (!isBinaryLE) throw std::runtime_error("Only binary_little_endian supported by this loader");
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

    // -------------------- Make point-cloud poly from vtkPoints --------------------
    static vtkSmartPointer<vtkPolyData> MakePointCloudPoly(vtkPoints* pts)
    {
        vtkSmartPointer<vtkPolyData> out = vtkSmartPointer<vtkPolyData>::New();
        out->SetPoints(pts);

        const vtkIdType n = pts ? pts->GetNumberOfPoints() : 0;

        vtkNew<vtkIdTypeArray> conn;
        conn->SetNumberOfComponents(1);
        conn->SetNumberOfTuples(n * 2);

        auto* ptr = conn->WritePointer(0, n * 2);
        for (vtkIdType i = 0; i < n; ++i) {
            ptr[2 * i] = 1;
            ptr[2 * i + 1] = i;
        }

        vtkNew<vtkCellArray> verts;
        verts->SetData(1, conn);
        out->SetVerts(verts);

        return out;
    }

    // -------------------- VOXEL downsample (hash grid; each voxel keeps 1 point) --------------------
    static vtkSmartPointer<vtkPolyData> VoxelDownsample(vtkPolyData* in, double voxelMm)
    {
        if (!in || !in->GetPoints() || voxelMm <= 0.0) return in;

        const double inv = 1.0 / voxelMm;

        struct Key { int x, y, z; };
        struct KeyHash {
            size_t operator()(const Key& k) const noexcept {
                size_t h = 1469598103934665603ull;
                auto mix = [&](int v) {
                    h ^= (size_t)v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
                    };
                mix(k.x); mix(k.y); mix(k.z);
                return h;
            }
        };
        struct KeyEq {
            bool operator()(const Key& a, const Key& b) const noexcept {
                return a.x == b.x && a.y == b.y && a.z == b.z;
            }
        };

        std::unordered_map<Key, char, KeyHash, KeyEq> seen;
        seen.reserve((size_t)in->GetNumberOfPoints() / 4 + 1);

        vtkNew<vtkPoints> pts;
        pts->SetDataTypeToFloat();

        double p[3];
        for (vtkIdType i = 0; i < in->GetNumberOfPoints(); ++i) {
            in->GetPoint(i, p);
            Key k{
                (int)std::floor(p[0] * inv),
                (int)std::floor(p[1] * inv),
                (int)std::floor(p[2] * inv)
            };
            if (seen.find(k) != seen.end()) continue;
            seen.emplace(k, 1);
            pts->InsertNextPoint(p);
        }

        return MakePointCloudPoly(pts);
    }

    // -------------------- OUTLIER (Radius neighbor count using KDTree) --------------------
    static vtkSmartPointer<vtkPolyData> RadiusOutlierRemoval(vtkPolyData* in, double radiusMm, int minNeighbors)
    {
        if (!in || !in->GetPoints() || radiusMm <= 0.0 || minNeighbors <= 0) return in;

        vtkNew<vtkKdTreePointLocator> kdtree;
        kdtree->SetDataSet(in);
        kdtree->BuildLocator();

        vtkNew<vtkIdList> ids;
        vtkNew<vtkPoints> pts;
        pts->SetDataTypeToFloat();

        double p[3];
        for (vtkIdType i = 0; i < in->GetNumberOfPoints(); ++i) {
            in->GetPoint(i, p);

            ids->Reset();
            kdtree->FindPointsWithinRadius(radiusMm, p, ids);

            // self 포함 기준(현업 UI에서 직관적)
            if ((int)ids->GetNumberOfIds() >= minNeighbors) {
                pts->InsertNextPoint(p);
            }
        }

        return MakePointCloudPoly(pts);
    }

    // -------------------- PLANE RANSAC (manual) --------------------
    static void PlaneRansac(vtkPolyData* in,
        double thresholdMm,
        int iterations,
        std::vector<vtkIdType>& bestInliers,
        double& outNx, double& outNy, double& outNz, double& outD)
    {
        bestInliers.clear();
        outNx = 0; outNy = 0; outNz = 1; outD = 0;

        if (!in || !in->GetPoints()) return;
        const vtkIdType N = in->GetNumberOfPoints();
        if (N < 3) return;

        std::mt19937 rng((unsigned)std::random_device{}());
        std::uniform_int_distribution<vtkIdType> dist(0, N - 1);

        double p1[3], p2[3], p3[3];

        auto norm3 = [](double x, double y, double z) {
            return std::sqrt(x * x + y * y + z * z);
            };

        const double thr = std::max(0.0001, thresholdMm);

        for (int it = 0; it < iterations; ++it) {
            vtkIdType i1 = dist(rng), i2 = dist(rng), i3 = dist(rng);
            if (i1 == i2 || i2 == i3 || i1 == i3) continue;

            in->GetPoint(i1, p1);
            in->GetPoint(i2, p2);
            in->GetPoint(i3, p3);

            const double v1x = p2[0] - p1[0], v1y = p2[1] - p1[1], v1z = p2[2] - p1[2];
            const double v2x = p3[0] - p1[0], v2y = p3[1] - p1[1], v2z = p3[2] - p1[2];

            // n = v1 x v2
            double nx = v1y * v2z - v1z * v2y;
            double ny = v1z * v2x - v1x * v2z;
            double nz = v1x * v2y - v1y * v2x;

            const double nlen = norm3(nx, ny, nz);
            if (nlen < 1e-9) continue;

            nx /= nlen; ny /= nlen; nz /= nlen;
            const double d = -(nx * p1[0] + ny * p1[1] + nz * p1[2]);

            std::vector<vtkIdType> inliers;
            inliers.reserve((size_t)N / 2);

            double p[3];
            for (vtkIdType i = 0; i < N; ++i) {
                in->GetPoint(i, p);
                const double distp = std::abs(nx * p[0] + ny * p[1] + nz * p[2] + d); // ||n||=1
                if (distp <= thr) inliers.push_back(i);
            }

            if (inliers.size() > bestInliers.size()) {
                bestInliers.swap(inliers);
                outNx = nx; outNy = ny; outNz = nz; outD = d;

                // early stop (plane dominates)
                if (bestInliers.size() > (size_t)(N * 0.85))
                    break;
            }
        }
    }

    static double PlaneRmse(vtkPolyData* in,
        const std::vector<vtkIdType>& inliers,
        double nx, double ny, double nz, double d)
    {
        if (!in || inliers.empty()) return 0.0;

        double p[3];
        double sum2 = 0.0;
        for (auto id : inliers) {
            in->GetPoint(id, p);
            const double distp = (nx * p[0] + ny * p[1] + nz * p[2] + d); // signed
            sum2 += distp * distp;
        }
        return std::sqrt(sum2 / (double)inliers.size());
    }

    static vtkSmartPointer<vtkPolyData> RemoveInliers(vtkPolyData* in, const std::vector<vtkIdType>& inliers)
    {
        if (!in || !in->GetPoints() || inliers.empty()) return in;

        const vtkIdType N = in->GetNumberOfPoints();
        std::vector<char> mask((size_t)N, 0);
        for (auto id : inliers) if (id >= 0 && id < N) mask[(size_t)id] = 1;

        vtkNew<vtkPoints> pts;
        pts->SetDataTypeToFloat();

        double p[3];
        for (vtkIdType i = 0; i < N; ++i) {
            if (mask[(size_t)i]) continue;
            in->GetPoint(i, p);
            pts->InsertNextPoint(p);
        }

        return MakePointCloudPoly(pts);
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
        raw->Register(nullptr); // refcount 1 보장해서 넘김
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

// -------------------- ScanQaWorker --------------------
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
        emit progress(35);

        // 2) outlier
        int removedOutlier = 0;
        {
            const vtkIdType before = cur->GetNumberOfPoints();
            if (outlierEnabled) {
                cur = RadiusOutlierRemoval(cur, outlierRadiusMm, outlierMinNeighbors);
            }
            const vtkIdType after = cur->GetNumberOfPoints();
            removedOutlier = (int)std::max<vtkIdType>(0, before - after);
        }
        emit progress(65);

        // 3) plane
        int removedPlane = 0;
        double planeInlierRatio = 0.0;
        double planeRmseMm = 0.0;

        if (planeEnabled && cur && cur->GetNumberOfPoints() >= 3) {
            std::vector<vtkIdType> inliers;
            double nx, ny, nz, d;
            PlaneRansac(cur, planeThresholdMm, planeIterations, inliers, nx, ny, nz, d);

            const vtkIdType N = cur->GetNumberOfPoints();
            planeInlierRatio = (N > 0) ? ((double)inliers.size() / (double)N) : 0.0;
            planeRmseMm = PlaneRmse(cur, inliers, nx, ny, nz, d);

            if (planeRemoveEnabled && !inliers.empty()) {
                const vtkIdType before = cur->GetNumberOfPoints();
                cur = RemoveInliers(cur, inliers);
                const vtkIdType after = cur->GetNumberOfPoints();
                removedPlane = (int)std::max<vtkIdType>(0, before - after);
            }
        }

        emit progress(95);

        vtkPolyData* raw = cur.GetPointer();
        raw->Register(nullptr);
        emit finished(raw, removedOutlier, removedPlane, planeInlierRatio, planeRmseMm);

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

    // -------------------- Workspace selector --------------------
    v->addWidget(new QLabel("Workspace:", panel));

    workspaceCombo_ = new QComboBox(panel);
    workspaceCombo_->addItem("Scan QA");
    workspaceCombo_->addItem("Inspection");
    workspaceCombo_->addItem("Robot Pose");
    workspaceCombo_->setCurrentIndex(0);
    v->addWidget(workspaceCombo_);

    workspaceStack_ = new QStackedWidget(panel);
    v->addWidget(workspaceStack_);

    connect(workspaceCombo_, &QComboBox::currentIndexChanged, this, [this](int idx) {
        SetWorkspaceIndex(idx);
        });

    // -------------------- Page 0: Scan QA --------------------
    {
        auto* page = new QWidget(panel);
        auto* pv = new QVBoxLayout(page);

        // ---- Scan QA Panel (Preprocess) ----
        auto* qaGroup = new QGroupBox("Scan QA (Preprocess)", page);
        auto* form = new QFormLayout(qaGroup);

        // View: Raw / Processed
        viewCombo_ = new QComboBox(page);
        viewCombo_->addItem("Raw");
        viewCombo_->addItem("Processed");
        viewCombo_->setCurrentIndex(0);
        form->addRow("View:", viewCombo_);
        connect(viewCombo_, &QComboBox::currentIndexChanged, this, [this](int idx) {
            SetViewRaw(idx == 0);
            });

        // Voxel
        voxelEnable_ = new QCheckBox("Enable", page);
        voxelEnable_->setChecked(true);

        voxelMmSpin_ = new QDoubleSpinBox(page);
        voxelMmSpin_->setRange(0.1, 50.0);
        voxelMmSpin_->setValue(1.0);
        voxelMmSpin_->setSingleStep(0.1);
        voxelMmSpin_->setSuffix(" mm");

        auto* voxelRow = new QWidget(page);
        auto* voxelH = new QHBoxLayout(voxelRow);
        voxelH->setContentsMargins(0, 0, 0, 0);
        voxelH->addWidget(voxelEnable_);
        voxelH->addWidget(voxelMmSpin_, 1);
        form->addRow("Voxel:", voxelRow);

        // Outlier
        outlierEnable_ = new QCheckBox("Enable", page);
        outlierEnable_->setChecked(true);

        outlierRadiusSpin_ = new QDoubleSpinBox(page);
        outlierRadiusSpin_->setRange(0.1, 200.0);
        outlierRadiusSpin_->setValue(3.0);
        outlierRadiusSpin_->setSingleStep(0.1);
        outlierRadiusSpin_->setSuffix(" mm");

        outlierMinNbSpin_ = new QSpinBox(page);
        outlierMinNbSpin_->setRange(1, 200);
        outlierMinNbSpin_->setValue(5);

        auto* outRow = new QWidget(page);
        auto* outH = new QHBoxLayout(outRow);
        outH->setContentsMargins(0, 0, 0, 0);
        outH->addWidget(outlierEnable_);
        outH->addWidget(new QLabel("R:", page));
        outH->addWidget(outlierRadiusSpin_, 1);
        outH->addWidget(new QLabel("MinN:", page));
        outH->addWidget(outlierMinNbSpin_);
        form->addRow("Outlier:", outRow);

        // Plane
        planeEnable_ = new QCheckBox("Enable", page);
        planeEnable_->setChecked(true);

        planeThresholdSpin_ = new QDoubleSpinBox(page);
        planeThresholdSpin_->setRange(0.1, 50.0);
        planeThresholdSpin_->setValue(2.0);
        planeThresholdSpin_->setSingleStep(0.1);
        planeThresholdSpin_->setSuffix(" mm");

        planeIterSpin_ = new QSpinBox(page);
        planeIterSpin_->setRange(50, 5000);
        planeIterSpin_->setValue(200);

        planeRemoveEnable_ = new QCheckBox("Remove plane", page);
        planeRemoveEnable_->setChecked(true);

        auto* planeRow = new QWidget(page);
        auto* planeH = new QHBoxLayout(planeRow);
        planeH->setContentsMargins(0, 0, 0, 0);
        planeH->addWidget(planeEnable_);
        planeH->addWidget(new QLabel("Thr:", page));
        planeH->addWidget(planeThresholdSpin_, 1);
        planeH->addWidget(new QLabel("Iter:", page));
        planeH->addWidget(planeIterSpin_);
        planeH->addWidget(planeRemoveEnable_);
        form->addRow("Plane:", planeRow);

        // Apply
        applyQaBtn_ = new QPushButton("Apply Scan QA", page);
        form->addRow(applyQaBtn_);
        connect(applyQaBtn_, &QPushButton::clicked, this, [this]() {
            ApplyScanQaAsync();
            });

        // Metrics
        qaPointCountLabel_ = new QLabel("-", page);
        qaBoundsLabel_ = new QLabel("-", page);
        qaStatusLabel_ = new QLabel("Load a PLY to see metrics.", page);

        qaRemovedOutlierLabel_ = new QLabel("-", page);
        qaRemovedPlaneLabel_ = new QLabel("-", page);
        qaPlaneInlierLabel_ = new QLabel("-", page);
        qaPlaneRmseLabel_ = new QLabel("-", page);

        form->addRow("Points:", qaPointCountLabel_);
        form->addRow("Bounds (X/Y/Z):", qaBoundsLabel_);
        form->addRow("Status:", qaStatusLabel_);
        form->addRow("Removed Outlier:", qaRemovedOutlierLabel_);
        form->addRow("Removed Plane:", qaRemovedPlaneLabel_);
        form->addRow("Plane Inlier(%):", qaPlaneInlierLabel_);
        form->addRow("Plane RMSE(mm):", qaPlaneRmseLabel_);

        qaGroup->setLayout(form);
        pv->addWidget(qaGroup);
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

    SetWorkspaceIndex(0);

    // -------------------- Render controls (common) --------------------
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

    v->addWidget(new QLabel("Note: current loader supports binary_little_endian PLY only.", panel));

    v->addStretch(1);

    panel->setLayout(v);
    dock->setWidget(panel);
    addDockWidget(Qt::LeftDockWidgetArea, dock);
}

void MainWindow::SetWorkspaceIndex(int idx)
{
    if (workspaceStack_) workspaceStack_->setCurrentIndex(idx);
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
        processedCloud_ = poly;

        lastRemovedOutlier_ = 0;
        lastRemovedPlane_ = 0;
        lastPlaneInlierRatio_ = 0.0;
        lastPlaneRmseMm_ = 0.0;

        if (viewCombo_) viewCombo_->setCurrentIndex(0); // Raw
        mapper_->SetInputData(rawCloud_);
        mapper_->ScalarVisibilityOff();

        renderer_->ResetCamera();
        ApplyRenderOptions();
        UpdateQaMetricsUI();
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
    const int mode = modeCombo_ ? modeCombo_->currentIndex() : 2;
    auto* prop = actor_->GetProperty();

    if (pointSizeSlider_) prop->SetPointSize(pointSizeSlider_->value());
    if (lineWidthSlider_) prop->SetLineWidth(lineWidthSlider_->value());

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

    QThread* th = new QThread(this);
    auto* worker = new ScanQaWorker();
    worker->input = rawCloud_;

    worker->voxelEnabled = voxelEnable_ && voxelEnable_->isChecked();
    worker->voxelMm = voxelMmSpin_ ? voxelMmSpin_->value() : 1.0;

    worker->outlierEnabled = outlierEnable_ && outlierEnable_->isChecked();
    worker->outlierRadiusMm = outlierRadiusSpin_ ? outlierRadiusSpin_->value() : 3.0;
    worker->outlierMinNeighbors = outlierMinNbSpin_ ? outlierMinNbSpin_->value() : 5;

    worker->planeEnabled = planeEnable_ && planeEnable_->isChecked();
    worker->planeThresholdMm = planeThresholdSpin_ ? planeThresholdSpin_->value() : 2.0;
    worker->planeIterations = planeIterSpin_ ? planeIterSpin_->value() : 200;
    worker->planeRemoveEnabled = planeRemoveEnable_ && planeRemoveEnable_->isChecked();

    worker->moveToThread(th);

    connect(dlg, &QProgressDialog::canceled, this, [=]() {
        dlg->setLabelText("Cancel requested (will finish current step)...");
        th->requestInterruption();
        });

    connect(worker, &ScanQaWorker::progress, this, [=](int p) {
        if (dlg) dlg->setValue(p);
        }, Qt::QueuedConnection);

    connect(worker, &ScanQaWorker::finished, this,
        [=](vtkPolyData* outRaw, int removedOut, int removedPlane, double inlierRatio, double rmseMm)
        {
            if (dlg) { dlg->setValue(100); dlg->close(); dlg->deleteLater(); }

            vtkSmartPointer<vtkPolyData> out;
            out.TakeReference(outRaw);

            processedCloud_ = out;
            lastRemovedOutlier_ = removedOut;
            lastRemovedPlane_ = removedPlane;
            lastPlaneInlierRatio_ = inlierRatio;
            lastPlaneRmseMm_ = rmseMm;

            if (viewCombo_) viewCombo_->setCurrentIndex(1); // Processed
            mapper_->SetInputData(processedCloud_);
            mapper_->ScalarVisibilityOff();

            UpdateQaMetricsUI();
            UpdateRender();

            processing_ = false;
            worker->deleteLater();
            th->quit();
            th->deleteLater();
        }, Qt::QueuedConnection);

    connect(worker, &ScanQaWorker::failed, this, [=](const QString& msg) {
        if (dlg) { dlg->close(); dlg->deleteLater(); }
        QMessageBox::critical(this, "Scan QA Failed", msg);

        processing_ = false;
        worker->deleteLater();
        th->quit();
        th->deleteLater();
        }, Qt::QueuedConnection);

    connect(th, &QThread::started, worker, &ScanQaWorker::run);
    th->start();
}

void MainWindow::UpdateQaMetricsUI()
{
    if (!qaPointCountLabel_ || !qaBoundsLabel_ || !qaStatusLabel_) return;

    vtkPolyData* cloud = (viewCombo_ && viewCombo_->currentIndex() == 0)
        ? rawCloud_.GetPointer()
        : processedCloud_.GetPointer();

    if (!cloud || cloud->GetNumberOfPoints() <= 0) {
        qaPointCountLabel_->setText("-");
        qaBoundsLabel_->setText("-");
        qaStatusLabel_->setText("No valid cloud loaded.");

        if (qaRemovedOutlierLabel_) qaRemovedOutlierLabel_->setText("-");
        if (qaRemovedPlaneLabel_) qaRemovedPlaneLabel_->setText("-");
        if (qaPlaneInlierLabel_) qaPlaneInlierLabel_->setText("-");
        if (qaPlaneRmseLabel_) qaPlaneRmseLabel_->setText("-");
        return;
    }

    const vtkIdType n = cloud->GetNumberOfPoints();
    double b[6]; cloud->GetBounds(b);
    const double sx = b[1] - b[0];
    const double sy = b[3] - b[2];
    const double sz = b[5] - b[4];

    qaPointCountLabel_->setText(QString::number((long long)n));
    qaBoundsLabel_->setText(QString("%1 / %2 / %3 mm")
        .arg(sx, 0, 'f', 2)
        .arg(sy, 0, 'f', 2)
        .arg(sz, 0, 'f', 2));

    // status (simple)
    if (n < 5000) qaStatusLabel_->setText("Too few points (likely not usable).");
    else qaStatusLabel_->setText("OK (basic).");

    if (qaRemovedOutlierLabel_) qaRemovedOutlierLabel_->setText(QString::number(lastRemovedOutlier_));
    if (qaRemovedPlaneLabel_) qaRemovedPlaneLabel_->setText(QString::number(lastRemovedPlane_));
    if (qaPlaneInlierLabel_) qaPlaneInlierLabel_->setText(QString("%1 %").arg(lastPlaneInlierRatio_ * 100.0, 0, 'f', 1));
    if (qaPlaneRmseLabel_) qaPlaneRmseLabel_->setText(QString::number(lastPlaneRmseMm_, 'f', 3));
}
