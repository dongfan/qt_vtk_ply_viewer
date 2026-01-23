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
#include <QPainter>
#include <QFontMetrics>
#include <QtMath>
#include <algorithm>
#include <numeric>

#include <QGroupBox>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QStackedWidget>
#include <QEvent>
#include <QSplitter>
#include <QTimer>
#include <QRegularExpression>
#include <QTextDocument>
#include <QToolTip>
#include <QCursor>

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

// ----------------- XYZ plot widget (no QtCharts dependency) -----------------
class XyzPlotWidget : public QWidget {
public:
    explicit XyzPlotWidget(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumHeight(220);
        setMouseTracking(true);
    }

    struct AxisStats {
        double minV = 0.0;
        double maxV = 0.0;
        double mean = 0.0;
        double stdev = 0.0;
    };

    void setData(const QVector<int>& hx,
                 const QVector<int>& hy,
                 const QVector<int>& hz,
                 const AxisStats& sx,
                 const AxisStats& sy,
                 const AxisStats& sz)
    {
        hx_ = hx; hy_ = hy; hz_ = hz;
        sx_ = sx; sy_ = sy; sz_ = sz;
        update();
    }

    void clear() {
        hx_.clear(); hy_.clear(); hz_.clear();
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        // background
        p.fillRect(rect(), QColor(25, 25, 30));

        const int pad = 10;
        const int gap = 10;
        QRect r = rect().adjusted(pad, pad, -pad, -pad);

        const int hEach = (r.height() - 2 * gap) / 3;
        QRect rx(r.left(), r.top(), r.width(), hEach);
        QRect ry(r.left(), r.top() + hEach + gap, r.width(), hEach);
        QRect rz(r.left(), r.top() + 2 * (hEach + gap), r.width(), hEach);

        drawHist(p, rx, "X", hx_, sx_);
        drawHist(p, ry, "Y", hy_, sy_);
        drawHist(p, rz, "Z", hz_, sz_);
    }

private:
    static double safeMaxCount(const QVector<int>& h) {
        int mx = 0;
        for (int v : h) mx = std::max(mx, v);
        return mx > 0 ? double(mx) : 1.0;
    }

    void drawHist(QPainter& p, const QRect& r, const QString& name,
                  const QVector<int>& h, const AxisStats& s)
    {
        // frame
        p.setPen(QColor(90, 90, 110));
        p.drawRoundedRect(r, 6, 6);

        // title + stats
        p.setPen(QColor(230, 230, 240));
        const QString title = QString("%1  min:%2  max:%3  mean:%4  σ:%5")
            .arg(name)
            .arg(s.minV, 0, 'f', 2)
            .arg(s.maxV, 0, 'f', 2)
            .arg(s.mean, 0, 'f', 2)
            .arg(s.stdev, 0, 'f', 2);

        const int textH = 18;
        p.drawText(r.adjusted(8, 2, -8, 0), Qt::AlignLeft | Qt::AlignTop, title);

        QRect plot = r.adjusted(8, textH + 4, -8, -10);

        if (h.isEmpty()) {
            p.setPen(QColor(150, 150, 170));
            p.drawText(plot, Qt::AlignCenter, "No data");
            return;
        }

        const double mx = safeMaxCount(h);
        const int n = h.size();

        // axes baseline
        p.setPen(QColor(120, 120, 140));
        p.drawLine(plot.bottomLeft(), plot.bottomRight());

        // bars
        const double barW = double(plot.width()) / double(n);
        for (int i = 0; i < n; ++i) {
            const double frac = double(h[i]) / mx;
            const int bh = int(frac * double(plot.height()));
            QRectF br(plot.left() + i * barW,
                      plot.bottom() - bh,
                      std::max(1.0, barW - 1.0),
                      bh);
            p.fillRect(br, QColor(120, 200, 255, 170));
        }

        // min/max labels
        p.setPen(QColor(200, 200, 210));
        p.drawText(r.adjusted(8, 0, -8, -2), Qt::AlignLeft | Qt::AlignBottom,
                   QString::number(s.minV, 'f', 2));
        p.drawText(r.adjusted(8, 0, -8, -2), Qt::AlignRight | Qt::AlignBottom,
                   QString::number(s.maxV, 'f', 2));
    }

    QVector<int> hx_, hy_, hz_;
    AxisStats sx_, sy_, sz_;
};

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkCubeAxesActor.h>
#include <vtkTextProperty.h>
#include <vtkLookupTable.h>
#include <vtkScalarBarActor.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkElevationFilter.h>
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

static QString HtmlToPlainText(const QString& html)
{
    QTextDocument doc;
    doc.setHtml(html);
    return doc.toPlainText();
}

// -------------------- MainWindow --------------------
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("Qt + VTK PLY Viewer");
    resize(1200, 850);
    setAcceptDrops(true);

    auto* central = new QWidget(this);

    // ✅ Splitter로 뷰어/도움말을 위아래로 분리 (사용자 드래그로 높이 조절 가능)
    auto* splitter = new QSplitter(Qt::Vertical, central);

    // VTK Viewer (위)
    vtkWidget_ = new QVTKOpenGLNativeWidget(splitter);
    splitter->addWidget(vtkWidget_);

    // Help panel (아래)
    helpLabel_ = new QTextBrowser(splitter);
    helpLabel_->setText("Tip: Hover controls to see help.");
    helpLabel_->setReadOnly(true);
    helpLabel_->setOpenExternalLinks(false); // 나중에 true로 가능
    helpLabel_->setStyleSheet(
        "padding:6px;"
        "background:#1f2329;"
        "color:#c9d1d9;"
        "border-top:1px solid #333;"
    );
    splitter->addWidget(helpLabel_);

    // Help 패널 클릭 시, QA 도움말 고정(pin) 해제용
    helpLabel_->installEventFilter(this);

    // 초기 사이즈 비율: Viewer 크게 / Help 작게
    splitter->setStretchFactor(0, 10); // VTK
    splitter->setStretchFactor(1, 1);  // Help
    splitter->setSizes({ 800, 80 });     // 초기 높이(원하는 값으로 조절 가능)

    // 최소 높이 (완전히 안 보이게 접히는 걸 막고 싶으면 값 올리세요)
    vtkWidget_->setMinimumHeight(200);
    helpLabel_->setMinimumHeight(24);

    // central 레이아웃에는 splitter만 넣는다
    auto* layout = new QVBoxLayout(central);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(splitter);

    setCentralWidget(central);

    // ---- VTK setup ----
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
    InstallAxesAndColor();
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

        installHelp(voxelMmSpin_,
            "<b>Voxel (mm) – Downsample Grid Size</b><br>"
            "• Reduces point density by keeping one point per voxel.<br><br>"
            "• <b>Smaller value</b>: more detail, slower processing<br>"
            "• <b>Larger value</b>: fewer points, faster processing<br><br>"
            "<i>Typical:</i> 0.3 – 2.0 mm<br>"
            "<i>Note:</i> Too large voxels may reduce plane inlier ratio."
        );

        installHelp(outlierRadiusSpin_,
            "<b>R (mm) – Outlier Search Radius</b><br>"
            "• Counts neighboring points within radius <b>R</b>.<br>"
            "• A point is kept only if neighbors ≥ <b>MinN</b>.<br><br>"
            "• <b>Smaller R</b>: stricter, removes more sparse points<br>"
            "• <b>Larger R</b>: keeps more points, may preserve noise<br><br>"
            "<i>Typical:</i> 2 – 10 mm"
        );

        installHelp(outlierMinNbSpin_,
            "<b>MinN – Minimum Neighbor Count</b><br>"
            "• Minimum number of points required within radius <b>R</b>.<br><br>"
            "• <b>Higher value</b>: aggressive noise removal<br>"
            "• <b>Lower value</b>: preserves sparse regions<br><br>"
            "<i>Typical:</i> 3 – 15<br>"
            "<i>Tip:</i> Increase MinN if floating noise remains."
        );

        installHelp(planeThresholdSpin_,
            "<b>Thr (mm) – Plane Inlier Distance Threshold</b><br>"
            "• A point is considered a plane inlier if:<br>"
            "&nbsp;&nbsp;&nbsp;&nbsp;<b>distance to plane ≤ Thr</b><br><br>"
            "• <b>Larger Thr</b>: higher plane inlier (%)<br>"
            "• <b>Smaller Thr</b>: stricter plane definition<br><br>"
            "<i>Typical:</i> 1 – 8 mm<br>"
            "<i>Note:</i> Too small values may underestimate plane coverage."
        );

        installHelp(planeIterSpin_,
            "<b>Iter – RANSAC Iterations</b><br>"
            "• Number of random plane hypotheses tested.<br><br>"
            "• <b>Higher value</b>: better chance to find dominant plane<br>"
            "• <b>Lower value</b>: faster, but may miss correct plane<br><br>"
            "<i>Typical:</i> 200 – 2000<br>"
            "<i>Tip:</i> Increase for complex scenes or low inlier ratios."
        );

        installHelp(planeRemoveEnable_,
            "<b>Remove Plane</b><br>"
            "• Removes detected plane inlier points (e.g., table or floor).<br><br>"
            "• <b>ON</b>: isolate objects for picking / inspection<br>"
            "• <b>OFF</b>: visualize plane quality without removal<br><br>"
            "<i>Recommended:</i> ON for picking, OFF for QA tuning"
        );

        // Metrics
        qaPointCountLabel_ = new QLabel("-", page);
        qaBoundsLabel_ = new QLabel("-", page);
        qaStatusLabel_ = new QLabel("Load a PLY to see metrics.", page);

        qaRemovedOutlierLabel_ = new QLabel("-", page);
        qaRemovedPlaneLabel_ = new QLabel("-", page);
        qaPlaneInlierLabel_ = new QLabel("-", page);
        qaPlaneRmseLabel_ = new QLabel("-", page);
        // QA 기준 Help 연동(FAIL/PASS 클릭)
        InstallQaCriteriaHelp();


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

    // -------------------- Coordinate & Color --------------------
    v->addSpacing(10);
    v->addWidget(new QLabel("Coordinate & Color:", panel));

    showAxesCheck_ = new QCheckBox("Show corner axes", panel);
    showAxesCheck_->setChecked(true);
    v->addWidget(showAxesCheck_);

    showCubeAxesCheck_ = new QCheckBox("Show bounds axes (X/Y/Z)", panel);
    showCubeAxesCheck_->setChecked(true);
    v->addWidget(showCubeAxesCheck_);

    v->addWidget(new QLabel("Color:", panel));
    colorModeCombo_ = new QComboBox(panel);
    colorModeCombo_->addItem("Solid");
    colorModeCombo_->addItem("Height (Z) Jet");
    colorModeCombo_->addItem("PLY RGB (if available)");
    colorModeCombo_->setCurrentIndex(0);
    v->addWidget(colorModeCombo_);

    showScalarBarCheck_ = new QCheckBox("Show color legend", panel);
    showScalarBarCheck_->setChecked(true);
    v->addWidget(showScalarBarCheck_);

    // update hooks
    auto refreshAxesAndColor = [this]() {
        ApplyColorMappingForCurrentView();
        UpdateAxesAndColor();
        ApplyRenderOptions();
        UpdateRender();
    };
    connect(showAxesCheck_, &QCheckBox::toggled, this, [refreshAxesAndColor](bool) { refreshAxesAndColor(); });
    connect(showCubeAxesCheck_, &QCheckBox::toggled, this, [refreshAxesAndColor](bool) { refreshAxesAndColor(); });
    connect(showScalarBarCheck_, &QCheckBox::toggled, this, [refreshAxesAndColor](bool) { refreshAxesAndColor(); });
    connect(colorModeCombo_, &QComboBox::currentIndexChanged, this, [refreshAxesAndColor](int) { refreshAxesAndColor(); });

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
        ApplyColorMappingForCurrentView();
        UpdateAxesAndColor();

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

// -------------------- Coordinate axes + color mapping --------------------
void MainWindow::InstallAxesAndColor()
{
    // Corner triad (orientation marker)
    axesActor_ = vtkSmartPointer<vtkAxesActor>::New();
    axesActor_->SetTotalLength(60.0, 60.0, 60.0);
    axesActor_->SetShaftTypeToCylinder();
    axesActor_->SetCylinderRadius(0.03);

    axesWidget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    axesWidget_->SetOrientationMarker(axesActor_);
    axesWidget_->SetViewport(0.0, 0.0, 0.18, 0.18);
    if (vtkWidget_ && vtkWidget_->renderWindow() && vtkWidget_->renderWindow()->GetInteractor()) {
        axesWidget_->SetInteractor(vtkWidget_->renderWindow()->GetInteractor());
    }
    axesWidget_->SetEnabled(1);
    axesWidget_->InteractiveOff();

    // Bounds axes with labels (CubeAxes)
    cubeAxes_ = vtkSmartPointer<vtkCubeAxesActor>::New();
    cubeAxes_->SetFlyModeToOuterEdges();
    cubeAxes_->SetGridLineLocation(vtkCubeAxesActor::VTK_GRID_LINES_FURTHEST);
    cubeAxes_->SetCamera(renderer_->GetActiveCamera());
    cubeAxes_->SetXTitle("X (mm)");
    cubeAxes_->SetYTitle("Y (mm)");
    cubeAxes_->SetZTitle("Z (mm)");
    cubeAxes_->SetXAxisVisibility(1);
    cubeAxes_->SetYAxisVisibility(1);
    cubeAxes_->SetZAxisVisibility(1);

    // title/label colors: X red, Y green, Z blue
    auto setAxisText = [](vtkTextProperty* tp, double r, double g, double b) {
        if (!tp) return;
        tp->SetColor(r, g, b);
        tp->SetFontSize(12);
        tp->BoldOn();
        tp->ShadowOff();
    };
    //setAxisText(cubeAxes_->GetXTitleTextProperty(0), 1.0, 0.2, 0.2);
    //setAxisText(cubeAxes_->GetXLabelTextProperty(0), 0.95, 0.75, 0.75);
    //setAxisText(cubeAxes_->GetYTitleTextProperty(0), 0.2, 1.0, 0.2);
    //setAxisText(cubeAxes_->GetYLabelTextProperty(0), 0.75, 0.95, 0.75);
    //setAxisText(cubeAxes_->GetZTitleTextProperty(0), 0.2, 0.4, 1.0);
    //setAxisText(cubeAxes_->GetZLabelTextProperty(0), 0.75, 0.82, 0.95);

    renderer_->AddActor(cubeAxes_);

    // Jet-like LUT for height coloring
    lutJet_ = vtkSmartPointer<vtkLookupTable>::New();
    lutJet_->SetNumberOfTableValues(256);
    // HSV hue sweep blue(0.667) -> red(0.0) resembles Jet for many uses
    lutJet_->SetHueRange(0.667, 0.0);
    lutJet_->SetSaturationRange(1.0, 1.0);
    lutJet_->SetValueRange(1.0, 1.0);
    lutJet_->Build();

    scalarBar_ = vtkSmartPointer<vtkScalarBarActor>::New();
    scalarBar_->SetLookupTable(lutJet_);
    scalarBar_->SetTitle("Z (mm)");
    scalarBar_->SetNumberOfLabels(5);
    scalarBar_->SetMaximumWidthInPixels(90);
    scalarBar_->SetMaximumHeightInPixels(280);
    renderer_->AddActor2D(scalarBar_);

    elevationFilter_ = vtkSmartPointer<vtkElevationFilter>::New();

    // Initial visibility
    UpdateAxesAndColor();
    ApplyColorMappingForCurrentView();
}

void MainWindow::UpdateAxesAndColor()
{
    // Toggle visibility according to UI
    const bool showCorner = (showAxesCheck_ ? showAxesCheck_->isChecked() : true);
    const bool showBounds = (showCubeAxesCheck_ ? showCubeAxesCheck_->isChecked() : true);

    if (axesWidget_) axesWidget_->SetEnabled(showCorner ? 1 : 0);
    if (cubeAxes_) cubeAxes_->SetVisibility(showBounds ? 1 : 0);

    // Update bounds from current mapper input
    vtkPolyData* cloud = nullptr;
    const bool wantProcessed = (viewCombo_ && viewCombo_->currentIndex() == 1);
    if (wantProcessed && processedCloud_ && processedCloud_->GetNumberOfPoints() > 0) cloud = processedCloud_;
    else if (rawCloud_ && rawCloud_->GetNumberOfPoints() > 0) cloud = rawCloud_;
    if (cloud && cubeAxes_) {
        double b[6] = {0,0,0,0,0,0};
        cloud->GetBounds(b);
        cubeAxes_->SetBounds(b);
        cubeAxes_->SetCamera(renderer_->GetActiveCamera());
    }
}

void MainWindow::ApplyColorMappingForCurrentView()
{
    if (!mapper_) return;

    vtkPolyData* cloud = nullptr;
    const bool wantProcessed = (viewCombo_ && viewCombo_->currentIndex() == 1);
    if (wantProcessed && processedCloud_ && processedCloud_->GetNumberOfPoints() > 0) cloud = processedCloud_;
    else if (rawCloud_ && rawCloud_->GetNumberOfPoints() > 0) cloud = rawCloud_;
    if (!cloud) {
        mapper_->ScalarVisibilityOff();
        if (scalarBar_) scalarBar_->SetVisibility(0);
        return;
    }

    const int mode = (colorModeCombo_ ? colorModeCombo_->currentIndex() : 0);
    const bool showBar = (showScalarBarCheck_ ? showScalarBarCheck_->isChecked() : true);

    // 0 Solid
    if (mode == 0) {
        mapper_->SetInputData(cloud);
        mapper_->ScalarVisibilityOff();
        actor_->GetProperty()->SetColor(0.9, 0.9, 0.9);
        if (scalarBar_) scalarBar_->SetVisibility(0);
        return;
    }

    // 1 Height (Z) Jet
    if (mode == 1) {
        double b[6] = {0,0,0,0,0,0};
        cloud->GetBounds(b);

        // Elevation filter generates a scalar array based on Z
        elevationFilter_->SetInputData(cloud);
        elevationFilter_->SetLowPoint(0.0, 0.0, b[4]);
        elevationFilter_->SetHighPoint(0.0, 0.0, b[5]);
        elevationFilter_->Update();

        mapper_->SetInputConnection(elevationFilter_->GetOutputPort());
        mapper_->SetLookupTable(lutJet_);
        mapper_->SetScalarRange(b[4], b[5]);
        mapper_->ScalarVisibilityOn();
        actor_->GetProperty()->SetColor(1.0, 1.0, 1.0);

        if (scalarBar_) {
            scalarBar_->SetLookupTable(lutJet_);
            scalarBar_->SetTitle("Z (mm)");
            scalarBar_->SetVisibility(showBar ? 1 : 0);
        }
        return;
    }

    // 2 PLY RGB (if available)
    if (mode == 2) {
        mapper_->SetInputData(cloud);

        auto* scalars = cloud->GetPointData() ? cloud->GetPointData()->GetScalars() : nullptr;
        const bool hasRgb = (scalars && (scalars->GetNumberOfComponents() == 3 || scalars->GetNumberOfComponents() == 4));

        if (hasRgb) {
            mapper_->ScalarVisibilityOn();
            mapper_->SetColorModeToDirectScalars();
            mapper_->SetScalarModeToUsePointData();
            actor_->GetProperty()->SetColor(1.0, 1.0, 1.0);
        } else {
            // fallback
            mapper_->ScalarVisibilityOff();
            actor_->GetProperty()->SetColor(0.9, 0.9, 0.9);
        }
        if (scalarBar_) scalarBar_->SetVisibility(0);
        return;
    }
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
    ApplyColorMappingForCurrentView();
    UpdateAxesAndColor();
    ApplyColorMappingForCurrentView();
    UpdateAxesAndColor();
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
            ApplyColorMappingForCurrentView();
            UpdateAxesAndColor();

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

struct QaThresholds
{
    int minPoints = 30000;
    double minPlaneInlierRatio = 0.25; // 0~1
    double maxPlaneRmseMm = 3.0;
    double maxOutlierRatio = 0.20; // 0~1
};


struct ParamRecD { double min=0.0; double max=0.0; QString unit; };
struct ParamRecI { int min=0; int max=0; QString unit; };

struct QaParamRecs
{
    ParamRecD voxelMm;
    ParamRecD outlierR;
    ParamRecI outlierMinN;
    ParamRecD planeThr;
    ParamRecI planeIter;
};

// Recommended ranges (tuning guidance). Adjust freely per your domain.
static QaParamRecs GetParamRecs(bool inspectionMode)
{
    QaParamRecs r;
    if (inspectionMode) {
        r.voxelMm      = {0.30, 1.50, "mm"};
        r.outlierR     = {0.50, 3.00, "mm"};
        r.outlierMinN  = {8, 20, ""};
        r.planeThr     = {0.50, 3.00, "mm"};
        r.planeIter    = {300, 1500, ""};
    } else {
        r.voxelMm      = {0.50, 3.00, "mm"};
        r.outlierR     = {1.00, 8.00, "mm"};
        r.outlierMinN  = {4, 12, ""};
        r.planeThr     = {1.00, 8.00, "mm"};
        r.planeIter    = {150, 800, ""};
    }
    return r;
}

static QString HtmlValueBadge(const QString& text, const QString& bg, const QString& fg = "white")
{
    return QString("<span style='background:%1;color:%3;padding:1px 6px;border-radius:6px;font-weight:600;'>%2</span>")
        .arg(bg, text.toHtmlEscaped(), fg);
}

static QString HtmlCurrentWithRec(double cur, const ParamRecD& rec, int prec = 2)
{
    const bool out = (cur < rec.min || cur > rec.max);
    const QString curTxt = QString::number(cur, 'f', prec) + (rec.unit.isEmpty() ? "" : (" " + rec.unit));
    const QString recTxt = QString("Recommended: %1–%2 %3")
        .arg(rec.min, 0, 'f', prec)
        .arg(rec.max, 0, 'f', prec)
        .arg(rec.unit);
    if (out) {
        return QString("<span style='font-weight:700;color:#e74c3c;'>%1</span> <span style='opacity:0.85;'>(%2)</span>")
            .arg(curTxt.toHtmlEscaped(), recTxt.toHtmlEscaped());
    }
    return QString("<span style='font-weight:700;'>%1</span> <span style='opacity:0.75;'>(%2)</span>")
        .arg(curTxt.toHtmlEscaped(), recTxt.toHtmlEscaped());
}

static QString HtmlCurrentWithRecInt(int cur, const ParamRecI& rec)
{
    const bool out = (cur < rec.min || cur > rec.max);
    const QString curTxt = QString::number(cur);
    const QString recTxt = QString("Recommended: %1–%2").arg(rec.min).arg(rec.max);
    if (out) {
        return QString("<span style='font-weight:700;color:#e74c3c;'>%1</span> <span style='opacity:0.85;'>(%2)</span>")
            .arg(curTxt.toHtmlEscaped(), recTxt.toHtmlEscaped());
    }
    return QString("<span style='font-weight:700;'>%1</span> <span style='opacity:0.75;'>(%2)</span>")
        .arg(curTxt.toHtmlEscaped(), recTxt.toHtmlEscaped());
}

// Try to parse numeric hints from status text (fallback if last* metrics are unavailable)
static void ParseMetricsFromStatusText(const QString& statusText, double& rmseMm, double& inlierRatio01, double& outlierRatio01)
{
    // RMSE: "RMSE 1.23 mm" or "RMSE = 1.23 mm"
    QRegularExpression rxRmse(R"(RMSE\s*=?\s*([0-9]+(?:\.[0-9]+)?)\s*mm)", QRegularExpression::CaseInsensitiveOption);
    auto m1 = rxRmse.match(statusText);
    if (m1.hasMatch()) rmseMm = m1.captured(1).toDouble();

    // inlier: "inlier 34.5%" or "inlier 34.5 %"
    QRegularExpression rxInlier(R"(inlier\s*([0-9]+(?:\.[0-9]+)?)\s*%?)", QRegularExpression::CaseInsensitiveOption);
    auto m2 = rxInlier.match(statusText);
    if (m2.hasMatch()) inlierRatio01 = m2.captured(1).toDouble() / 100.0;

    // outliers: "(12.3% outliers)" or "12.3% outliers"
    QRegularExpression rxOut(R"(([0-9]+(?:\.[0-9]+)?)\s*%\s*outliers?)", QRegularExpression::CaseInsensitiveOption);
    auto m3 = rxOut.match(statusText);
    if (m3.hasMatch()) outlierRatio01 = m3.captured(1).toDouble() / 100.0;
}


struct QaEvalResult
{
    QString statusText;
    QaFailReason reason = QaFailReason::None;
    double outlierRatio = 0.0; // 0~1
};

static QaThresholds GetThresholds(bool inspectionMode)
{
    QaThresholds t;
    if (inspectionMode) {
        // Inspection(측정) 모드: 더 엄격한 기준 (초기값)
        t.minPoints = 60000;
        t.minPlaneInlierRatio = 0.35;
        t.maxPlaneRmseMm = 1.50;
        t.maxOutlierRatio = 0.10;
    }
    return t;
}

static QaEvalResult EvaluateScanQaDetailed(
    vtkPolyData* cloud,
    int removedOutlier,
    int /*removedPlane*/,
    double planeInlierRatio,
    double planeRmseMm,
    bool inspectionMode
)
{
    QaEvalResult r;
    const QaThresholds t = GetThresholds(inspectionMode);

    if (!cloud) {
        r.statusText = "FAIL: No cloud";
        r.reason = QaFailReason::NoCloud;
        return r;
    }

    const vtkIdType pointCount = cloud->GetNumberOfPoints();
    if (pointCount < t.minPoints) {
        r.statusText = "FAIL: Too few points";
        r.reason = QaFailReason::TooFewPoints;
        return r;
    }

    if (planeInlierRatio < t.minPlaneInlierRatio) {
        r.statusText = QString("FAIL: Plane unstable (inlier %1%)")
            .arg(planeInlierRatio * 100.0, 0, 'f', 1);
        r.reason = QaFailReason::PlaneUnstable;
        return r;
    }

    if (planeRmseMm > t.maxPlaneRmseMm) {
        r.statusText = QString("FAIL: Plane noisy (RMSE %1 mm)")
            .arg(planeRmseMm, 0, 'f', 2);
        r.reason = QaFailReason::PlaneNoisy;
        return r;
    }

    // NOTE: removedOutlier는 processed에서 제거된 포인트 수.
    // outlierRatio는 (제거된 수 / (원래 수))에 가까운 근사치로 계산.
    r.outlierRatio = (double)removedOutlier / (double)(pointCount + removedOutlier);

    if (r.outlierRatio > t.maxOutlierRatio) {
        r.statusText = QString("FAIL: Excessive noise (%1% outliers)")
            .arg(r.outlierRatio * 100.0, 0, 'f', 1);
        r.reason = QaFailReason::ExcessiveNoise;
        return r;
    }

    r.statusText = "PASS: Scan usable for next step";
    r.reason = QaFailReason::None;
    return r;
}

static QString EvaluateScanQa(
    vtkPolyData* cloud,
    int removedOutlier,
    int removedPlane,
    double planeInlierRatio,
    double planeRmseMm,
    bool inspectionMode
)
{
    return EvaluateScanQaDetailed(cloud, removedOutlier, removedPlane, planeInlierRatio, planeRmseMm, inspectionMode).statusText;
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

    // --- Quality Judgment ---
    bool inspectionMode = (workspaceCombo_ && workspaceCombo_->currentIndex() == 1);

    QaEvalResult eval = EvaluateScanQaDetailed(
        cloud,
        lastRemovedOutlier_,
        lastRemovedPlane_,
        lastPlaneInlierRatio_,
        lastPlaneRmseMm_,
        inspectionMode
    );

    lastQaReason_ = eval.reason;
    lastOutlierRatio_ = eval.outlierRatio;
    lastQaStatusText_ = eval.statusText;

    qaStatusLabel_->setText(eval.statusText);

    // 색상으로 직관적 표시
    if (eval.statusText.startsWith("PASS")) {
        qaStatusLabel_->setStyleSheet("color: #2ecc71; font-weight: bold;");
    }
    else {
        qaStatusLabel_->setStyleSheet("color: #e74c3c; font-weight: bold;");
    }
}


void MainWindow::installHelp(QObject* w, const QString& text)
{
    if (!w) return;
    helpMap_[w] = text;

    // Tooltip도 같이 달기(짧게/핵심만)
    if (auto* qw = qobject_cast<QWidget*>(w)) {
        //qw->setToolTip(text);
        qw->installEventFilter(this);
    }
}

static inline double stdevFromSumSq(int n, double sum, double sumSq)
{
    if (n <= 1) return 0.0;
    const double mean = sum / double(n);
    const double var = std::max(0.0, (sumSq / double(n)) - mean * mean);
    return std::sqrt(var);
}

void MainWindow::InstallQaCriteriaHelp()
{
    if (!qaStatusLabel_) return;

    // Hover 시에는 간단한 안내, 클릭 시에는 QA 기준/원인/권장 튜닝을 상세 표시
    helpMap_[qaStatusLabel_] =
        "<b>QA Status</b><br>"
        "• Shows PASS/FAIL based on Scan QA criteria.<br>"
        "• <b>Click</b> to open detailed criteria + tuning guide.";

    qaStatusLabel_->setCursor(Qt::PointingHandCursor);
    qaStatusLabel_->installEventFilter(this);
}

void MainWindow::FlashWidget(QWidget* w, int ms)
{
    if (!w) return;

    const QString old = w->styleSheet();
    w->setStyleSheet(old + "border: 2px solid #ffcc00; border-radius: 4px;");
    QTimer::singleShot(ms, w, [w, old]() {
        if (w) w->setStyleSheet(old);
    });
}

QString MainWindow::BuildQaCriteriaHelpHtml(QaFailReason reason, const QString& statusText) const
{
    const bool inspectionMode = (workspaceCombo_ && workspaceCombo_->currentIndex() == 1);
    const QaThresholds t = GetThresholds(inspectionMode);

    auto fmtPct = [](double v01) { return QString("%1 %").arg(v01 * 100.0, 0, 'f', 1); };
    auto badge = [](const QString& txt, const QString& bg) {
        return QString("<span style='background:%1;color:white;padding:2px 6px;border-radius:6px;font-weight:600;'>%2</span>").arg(bg, txt);
    };

    const QString modeName = inspectionMode ? "Inspection(측정)" : "Pick/Scan QA";

    const QString statusBadge =
        statusText.startsWith("PASS") ? badge("PASS", "#2ecc71") :
        statusText.startsWith("FAIL") ? badge("FAIL", "#e74c3c") :
        badge("INFO", "#3498db");

    
// 현재 측정값 (우선 last* 값 사용, 없으면 statusText에서 보조 파싱)
const QString curPoints = qaPointCountLabel_ ? qaPointCountLabel_->text() : "-";

double curInlier = lastPlaneInlierRatio_;
double curRmse = lastPlaneRmseMm_;
double curOutlier = lastOutlierRatio_;

// fallback: statusText 안에 수치가 들어있는 경우(예: "RMSE 1.2 mm", "inlier 30%")
if ((curRmse <= 0.0 || curInlier <= 0.0 || curOutlier <= 0.0) && !statusText.isEmpty()) {
    ParseMetricsFromStatusText(statusText, curRmse, curInlier, curOutlier);
}

const QaParamRecs recs = GetParamRecs(inspectionMode);

auto passFail = [](bool ok) {
    return ok ? "<span style='color:#2ecc71;font-weight:700;'>OK</span>"
              : "<span style='color:#e74c3c;font-weight:700;'>FAIL</span>";
};
    const bool okPoints = (curPoints.toLongLong() >= t.minPoints);
    const bool okInlier = (curInlier >= t.minPlaneInlierRatio);
    const bool okRmse = (curRmse <= t.maxPlaneRmseMm);
    const bool okOutlier = (curOutlier <= t.maxOutlierRatio);

    QString why;
    QString related;

    switch (reason) {
    case QaFailReason::TooFewPoints:
        why = "Point count is below minimum. Acquisition may be insufficient or voxel too large.";
        related = "Voxel (mm), acquisition settings";
        break;
    case QaFailReason::PlaneUnstable:
        why = "Plane inlier ratio is too low. Plane model is not stable.";
        related = "Plane Thr, Plane Iter, Voxel";
        break;
    case QaFailReason::PlaneNoisy:
        why = "Plane RMSE is too high. Scene/plane is noisy or threshold is not appropriate.";
        related = "Plane Thr, Outlier R/MinN, Voxel";
        break;
    case QaFailReason::ExcessiveNoise:
        why = "Outlier ratio is too high. Noise points remain after filtering.";
        related = "Outlier R, Outlier MinN, Voxel";
        break;
    case QaFailReason::NoCloud:
        why = "No cloud loaded. Load a PLY first.";
        related = "Load PLY";
        break;
    default:
        why = "Criteria summary for the current status.";
        related = "—";
        break;
    }

    // 파라미터 현재값 표시(있을 때만)
    auto curSpin = [](const QDoubleSpinBox* s)->QString {
        if (!s) return "-";
        return QString::number(s->value(), 'f', 2) + " mm";
    };
    auto curSpinI = [](const QSpinBox* s)->QString {
        if (!s) return "-";
        return QString::number(s->value());
    };

    
const double curVoxelVal = voxelMmSpin_ ? voxelMmSpin_->value() : 0.0;
const double curOutlierRVal = outlierRadiusSpin_ ? outlierRadiusSpin_->value() : 0.0;
const int curMinNVal = outlierMinNbSpin_ ? outlierMinNbSpin_->value() : 0;
const double curPlaneThrVal = planeThresholdSpin_ ? planeThresholdSpin_->value() : 0.0;
const int curPlaneIterVal = planeIterSpin_ ? planeIterSpin_->value() : 0;

// with recommended-range 강조 표시
const QString curVoxel = voxelMmSpin_ ? HtmlCurrentWithRec(curVoxelVal, recs.voxelMm, 2) : "-";
const QString curOR = outlierRadiusSpin_ ? HtmlCurrentWithRec(curOutlierRVal, recs.outlierR, 2) : "-";
const QString curMinN = outlierMinNbSpin_ ? HtmlCurrentWithRecInt(curMinNVal, recs.outlierMinN) : "-";
const QString curThr = planeThresholdSpin_ ? HtmlCurrentWithRec(curPlaneThrVal, recs.planeThr, 2) : "-";
const QString curIter = planeIterSpin_ ? HtmlCurrentWithRecInt(curPlaneIterVal, recs.planeIter) : "-";

    QString html;
    html += QString("<h3 style='margin:0 0 6px 0;'>QA Criteria (%1)</h3>").arg(modeName);
    html += QString("<div style='margin:0 0 8px 0;'>Status: %1 &nbsp; <span style='opacity:0.85;'>%2</span></div>")
        .arg(statusBadge, statusText.toHtmlEscaped());

    html += "<hr style='border:0;border-top:1px solid #333;margin:8px 0;'>";

    html += "<b>Criteria</b><br>";
    html += "<table style='width:100%;border-collapse:collapse;'>"
            "<tr><th align='left'>Item</th><th align='left'>Current</th><th align='left'>Threshold</th><th align='left'>Result</th></tr>";

    html += QString("<tr><td>Points</td><td>%1</td><td>≥ %2</td><td>%3</td></tr>")
        .arg(curPoints.toHtmlEscaped())
        .arg(t.minPoints)
        .arg(passFail(okPoints));

    html += QString("<tr><td>Plane inlier</td><td>%1</td><td>≥ %2%</td><td>%3</td></tr>")
        .arg(fmtPct(curInlier))
        .arg(t.minPlaneInlierRatio * 100.0, 0, 'f', 1)
        .arg(passFail(okInlier));

    html += QString("<tr><td>Plane RMSE</td><td>%1 mm</td><td>≤ %2 mm</td><td>%3</td></tr>")
        .arg(curRmse, 0, 'f', 3)
        .arg(t.maxPlaneRmseMm, 0, 'f', 2)
        .arg(passFail(okRmse));

    html += QString("<tr><td>Outlier ratio</td><td>%1</td><td>≤ %2%</td><td>%3</td></tr>")
        .arg(fmtPct(curOutlier))
        .arg(t.maxOutlierRatio * 100.0, 0, 'f', 1)
        .arg(passFail(okOutlier));

    html += "</table>";

    html += "<hr style='border:0;border-top:1px solid #333;margin:10px 0;'>";

    html += QString("<b>Why</b><br><span style='opacity:0.9;'>%1</span><br>").arg(why.toHtmlEscaped());

    html += "<br><b>Suggested tuning</b><br>";
    
html += "<ul style='margin-top:6px;'>";
auto dirD = [](double cur, double mn, double mx)->QString {
    if (cur <= 0.0) return "—";
    if (cur < mn) return "↑ increase";
    if (cur > mx) return "↓ decrease";
    return "fine-tune";
};
auto dirI = [](int cur, int mn, int mx)->QString {
    if (cur <= 0) return "—";
    if (cur < mn) return "↑ increase";
    if (cur > mx) return "↓ decrease";
    return "fine-tune";
};

if (reason == QaFailReason::PlaneUnstable) {
    html += QString("<li><b>Plane Iter</b>: %1 (Recommended %2–%3)</li>")
        .arg(dirI(curPlaneIterVal, recs.planeIter.min, recs.planeIter.max).toHtmlEscaped())
        .arg(recs.planeIter.min).arg(recs.planeIter.max);

    html += QString("<li><b>Plane Thr</b>: %1 (Recommended %2–%3 mm)</li>")
        .arg(dirD(curPlaneThrVal, recs.planeThr.min, recs.planeThr.max).toHtmlEscaped())
        .arg(recs.planeThr.min, 0, 'f', 2).arg(recs.planeThr.max, 0, 'f', 2);

    // heuristic: if inlier low, slightly increase threshold within range
    if (curInlier > 0.0 && curInlier < t.minPlaneInlierRatio) {
        html += "<li>Low inlier ratio → try <b>slightly increasing Plane Thr</b> (within recommended range) to gather more inliers.</li>";
    }
    html += "<li>If you still have too few inliers, consider a smaller <b>Voxel</b> (more points, slower).</li>";
}
else if (reason == QaFailReason::PlaneNoisy) {
    html += "<li>Plane RMSE is high → focus on stabilizing the plane and removing noise.</li>";

    html += QString("<li><b>Outlier R</b>: %1 (Recommended %2–%3 mm)</li>")
        .arg(dirD(curOutlierRVal, recs.outlierR.min, recs.outlierR.max).toHtmlEscaped())
        .arg(recs.outlierR.min, 0, 'f', 2).arg(recs.outlierR.max, 0, 'f', 2);

    html += QString("<li><b>Outlier MinN</b>: %1 (Recommended %2–%3)</li>")
        .arg(dirI(curMinNVal, recs.outlierMinN.min, recs.outlierMinN.max).toHtmlEscaped())
        .arg(recs.outlierMinN.min).arg(recs.outlierMinN.max);

    // heuristic about plane thr direction using rmse and current thr
    if (curPlaneThrVal > 0.0) {
        if (curPlaneThrVal > recs.planeThr.max) {
            html += "<li><b>Plane Thr</b> is high → try <b>decreasing</b> it to avoid fitting noise as plane.</li>";
        } else if (curPlaneThrVal < recs.planeThr.min) {
            html += "<li><b>Plane Thr</b> is very low → try <b>increasing</b> it slightly to get stable inliers.</li>";
        } else {
            html += "<li><b>Plane Thr</b> is in range → fine-tune in small steps (±0.2–0.5 mm).</li>";
        }
    }
}
else if (reason == QaFailReason::ExcessiveNoise) {
    html += "<li>Outlier ratio is high → strengthen outlier filtering first.</li>";

    html += QString("<li><b>Outlier R</b>: %1 (Recommended %2–%3 mm)</li>")
        .arg(dirD(curOutlierRVal, recs.outlierR.min, recs.outlierR.max).toHtmlEscaped())
        .arg(recs.outlierR.min, 0, 'f', 2).arg(recs.outlierR.max, 0, 'f', 2);

    html += QString("<li><b>Outlier MinN</b>: %1 (Recommended %2–%3)</li>")
        .arg(dirI(curMinNVal, recs.outlierMinN.min, recs.outlierMinN.max).toHtmlEscaped())
        .arg(recs.outlierMinN.min).arg(recs.outlierMinN.max);

    if (curOutlier > 0.0 && curOutlier > t.maxOutlierRatio) {
        html += "<li>If noise persists, consider slightly reducing <b>Voxel</b> (more points) only when necessary.</li>";
    }
}
else if (reason == QaFailReason::TooFewPoints) {
    html += "<li>Point count is low → acquisition density or voxel setting is the main suspect.</li>";

    html += QString("<li><b>Voxel</b>: %1 (Recommended %2–%3 mm)</li>")
        .arg(dirD(curVoxelVal, recs.voxelMm.min, recs.voxelMm.max).toHtmlEscaped())
        .arg(recs.voxelMm.min, 0, 'f', 2).arg(recs.voxelMm.max, 0, 'f', 2);

    html += "<li>If possible, re-scan with higher density / better exposure to increase points.</li>";
}
else if (reason == QaFailReason::NoCloud) {
    html += "<li>Load a PLY and run <b>Apply Scan QA</b>.</li>";
}
else {
    html += "<li>Run <b>Apply Scan QA</b>, then click the PASS/FAIL status to see criteria-specific guidance.</li>";
}
html += "</ul>";


    html += "<b>Related parameters</b><br>";
    html += QString("<span style='opacity:0.9;'>%1</span><br>").arg(related.toHtmlEscaped());

    html += "<br><b>Current parameters</b><br>";
    html += "<table style='width:100%;border-collapse:collapse;'>"
            "<tr><td>Voxel</td><td>" + curVoxel + "</td></tr>"
            "<tr><td>Outlier R</td><td>" + curOR + "</td></tr>"
            "<tr><td>Outlier MinN</td><td>" + curMinN + "</td></tr>"
            "<tr><td>Plane Thr</td><td>" + curThr + "</td></tr>"
            "<tr><td>Plane Iter</td><td>" + curIter + "</td></tr>"
            "</table>";

    html += "<div style='margin-top:8px;opacity:0.75;'>Tip: Run <b>Apply Scan QA</b> again after tuning.</div>";

    return html;
}

void MainWindow::ShowQaCriteriaHelp()
{
    if (!helpLabel_) return;

    // 클릭할 때마다 pin 토글: 고정되어 있으면 해제, 아니면 고정
    if (qaHelpPinned_) {
        qaHelpPinned_ = false;
        return;
    }

    qaHelpPinned_ = true;
    qaPinnedHtml_ = BuildQaCriteriaHelpHtml(lastQaReason_, lastQaStatusText_);

    // Add a small pinned badge (click help panel to unpin)
    qaPinnedHtml_ = QString("<div style='display:flex;align-items:center;gap:6px;padding:4px 6px;margin-bottom:6px;border-radius:6px;background:#f2f2f2;'><span>📌</span><b>Pinned</b><span style='opacity:0.75'>(click this panel to unpin)</span></div>") + qaPinnedHtml_;

    helpLabel_->setHtml(qaPinnedHtml_);
    helpLabel_->moveCursor(QTextCursor::Start);

    // 해당 FAIL 사유에 맞는 파라미터를 살짝 강조(시각적 힌트)
    switch (lastQaReason_) {
    case QaFailReason::PlaneUnstable:
        FlashWidget(planeThresholdSpin_);
        FlashWidget(planeIterSpin_);
        break;
    case QaFailReason::PlaneNoisy:
        FlashWidget(planeThresholdSpin_);
        FlashWidget(outlierRadiusSpin_);
        FlashWidget(outlierMinNbSpin_);
        break;
    case QaFailReason::ExcessiveNoise:
        FlashWidget(outlierRadiusSpin_);
        FlashWidget(outlierMinNbSpin_);
        break;
    case QaFailReason::TooFewPoints:
        FlashWidget(voxelMmSpin_);
        break;
    default:
        break;
    }
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
    // Help 패널 클릭 → QA 도움말 고정 해제
    if (obj == helpLabel_ && event->type() == QEvent::MouseButtonRelease) {
        qaHelpPinned_ = false;
        return false;
    }

    // QA Status 클릭 → QA 판정 기준/원인/권장 튜닝 표시
    if (obj == qaStatusLabel_ && event->type() == QEvent::MouseButtonRelease) {
        ShowQaCriteriaHelp();
        return true;
    }


    // QA 도움말이 pin 상태일 때: 특정 파라미터(위젯)를 클릭하면 해당 파라미터 도움말로 전환 (pin 해제)
    if (qaHelpPinned_ && event->type() == QEvent::MouseButtonRelease) {
        if (helpLabel_ && obj != helpLabel_ && obj != qaStatusLabel_ && helpMap_.contains(obj)) {
            qaHelpPinned_ = false;
            const QString html = helpMap_.value(obj);
            helpLabel_->setHtml(html);
            helpLabel_->moveCursor(QTextCursor::Start);
            return true;
        }
    }

    // 마우스 오버 또는 포커스 들어올 때 설명 갱신
    if (event->type() == QEvent::Enter || event->type() == QEvent::FocusIn) {
        if (helpLabel_ && helpMap_.contains(obj)) {
            const QString html = helpMap_.value(obj);

            // QA 기준 도움말이 pin 상태면, Help 패널은 유지하고 Hover 내용은 툴팁으로만 띄운다
            if (qaHelpPinned_) {
                if (auto* w = qobject_cast<QWidget*>(obj)) {
                    QToolTip::showText(QCursor::pos(), HtmlToPlainText(html), w);
                }
            }
            else {
                helpLabel_->setHtml(html);
                helpLabel_->moveCursor(QTextCursor::Start);
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}
