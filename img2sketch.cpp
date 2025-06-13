// img2sketch.cpp      --- C++17 port of img2sketch.py v0.2
//
// g++ -std=c++17 -O3 -Wall -Wextra -march=native img2sketch.cpp -o img2sketch `pkg-config --cflags --libs opencv4`
//
// ./img2sketch --image <image> --out_dir <output_dir>
//
// ---------------------------------------------------------------------------
//  * Requires OpenCV 4.x (with ximgproc) and C++17.
//  * Perlin-noise jitter was unused in the Python source, so it is omitted
//    here.  If you later want it, drop in a small 1-D noise function and
//    modulate line vertices before drawing.
// ---------------------------------------------------------------------------
// Author: Dmitry Tetkin (2025) 


#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <filesystem>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <random>

// ---------------------------------------------------------------------------
//                              Data structures
// ---------------------------------------------------------------------------

struct Point {
    int  x = 0, y = 0;
    bool value = false;

    Point() = default;
    Point(int xx, int yy, bool v = false) : x(xx), y(yy), value(v) {}

    operator bool() const noexcept { return value; }
};

struct Line {
    Point start, end;
    double slope = 0.0;
    double bias  = 0.0;

    Line() = default;
    Line(const Point& s, const Point& e) : start(s), end(e) {
        updateParams();
    }

    void updateEnd(const Point& e) {
        end = e;
        updateParams();
    }

    int manhattan() const {
        return std::abs(start.x - end.x) + std::abs(start.y - end.y);
    }

    static double slopeBetween(const Point& a, const Point& b) {
        return (b.x == a.x) ? std::numeric_limits<double>::max()
                            : static_cast<double>(b.y - a.y) /
                              static_cast<double>(b.x - a.x);
    }

    static double biasOf(const Point& anchor, double m) {
        return anchor.y - m * anchor.x;
    }

    // K-distance test (same as Python)
    static double kDistance(const Point& A, const Point& B,
                            const Point& C, double penalty = 1000.0) {
        double ABx = B.x - A.x, ABy = B.y - A.y;
        double ACx = C.x - A.x, ACy = C.y - A.y;
        double ab2 = ABx * ABx + ABy * ABy;
        if (ab2 == 0.0) return penalty;

        double t = (ACx * ABx + ACy * ABy) / ab2;
        if (t <= 1.0) return penalty;

        double dist = std::abs(ABx * ACy - ABy * ACx) / std::sqrt(ab2);
        return dist;
    }

private:
    void updateParams() {
        slope = slopeBetween(start, end);
        bias  = biasOf(start, slope);
    }
};

// ---------------------------------------------------------------------------
//  Matrix helper that wraps the binary edge map the same way the Python
//  class did.
// ---------------------------------------------------------------------------

class Matrix {
public:
    explicit Matrix(const cv::Mat& edges) :
        w_(edges.cols), h_(edges.rows),
        data_(h_, std::vector<Point>(w_))
    {
        for (int y = 0; y < h_; ++y)
            for (int x = 0; x < w_; ++x)
                data_[y][x] = Point(x, y, edges.at<uchar>(y, x) > 0);
    }

    Point& operator()(int x, int y)             { return data_[y][x]; }
    const Point& operator()(int x, int y) const { return data_[y][x]; }

    int width()  const noexcept { return w_; }
    int height() const noexcept { return h_; }

    // 8-neighbours excluding ‘krome’
    std::vector<Point*> neighbours(Point& p, const Point& krome) {
        static const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
        static const int dy[8] = {-1,-1,-1,  0, 0,  1, 1, 1};

        std::vector<Point*> out;
        for (int k = 0; k < 8; ++k) {
            int nx = p.x + dx[k], ny = p.y + dy[k];
            if (inside(nx, ny) && data_[ny][nx] &&
                !(nx == krome.x && ny == krome.y))
            {
                out.push_back(&data_[ny][nx]);
            }
        }
        return out;
    }

    // 4 neighbours “boarded” version (E, SW, S, SE)
    std::vector<Point*> boarded(Point& p) {
        static const int dx[4] = { 1,-1, 0, 1};
        static const int dy[4] = { 0, 1, 1, 1};

        std::vector<Point*> out;
        for (int k = 0; k < 4; ++k) {
            int nx = p.x + dx[k], ny = p.y + dy[k];
            if (inside(nx, ny) && data_[ny][nx]) out.push_back(&data_[ny][nx]);
        }
        return out;
    }

private:
    bool inside(int x, int y) const noexcept {
        return (0 <= x && x <  w_) && (0 <= y && y < h_);
    }

    int w_, h_;
    std::vector<std::vector<Point>> data_;
};

// ---------------------------------------------------------------------------
//  Image helpers
// ---------------------------------------------------------------------------

cv::Mat toNormalMap(const cv::Mat& gray, float strength = 1.0f) {
    cv::Mat gray32;
    gray.convertTo(gray32, CV_32F, 1.0 / 255.0);

    cv::Mat gx, gy;
    cv::Sobel(gray32, gx, CV_32F, 1, 0, 5);
    cv::Sobel(gray32, gy, CV_32F, 0, 1, 5);

    gx *= strength;
    gy *= strength;

    cv::Mat normal;
    std::vector<cv::Mat> ch = { -gx, -gy, cv::Mat::ones(gray.size(), CV_32F) };
    cv::merge(ch, normal);
    cv::normalize(normal, normal, 0.0, 1.0, cv::NORM_MINMAX);

    normal.convertTo(normal, CV_8UC3, 255.0);
    return normal;
}

cv::Mat extractEdges(const cv::Mat& gray) {
    cv::Mat edges;
    cv::Canny(gray, edges, 30, 350);

    // Thinning if available
    if (cv::ximgproc::thinning) {
        cv::Mat thin;
        cv::ximgproc::thinning(edges, thin);
        return thin;
    }
    return edges;
}

// ---------------------------------------------------------------------------
//  Line-extraction port (DFS similar to Python recursion, but iterative)
// ---------------------------------------------------------------------------

static constexpr int MIN_LINE_LEN = 10;

std::vector<Line> extractLines(const cv::Mat& edges) {
    Matrix M(edges);
    std::vector<Line> lines;
    std::cout << "Extracting lines …\n";

    for (int y = 0; y < M.height(); ++y) {
        for (int x = 0; x < M.width(); ++x) {
            Point& p = M(x, y);
            if (!p) continue;

            auto boarded = M.boarded(p);
            for (Point* nb : boarded) {

                // Non-recursive DFS that mimics go_by_line
                Line candidate(p, *nb);
                Point* cur = nb;
                Point* prev = &p;

                while (cur) {
                    candidate.updateEnd(*cur);
                    auto neighbs = M.neighbours(*cur, *prev);
                    prev = cur;

                    if (neighbs.empty()) {
                        cur->value = false;
                        break;
                    }
                    if (neighbs.size() == 1) {
                        cur->value = false;
                        cur = neighbs[0];
                        continue;
                    }

                    // choose neighbour producing smallest k-distance
                    double best = 1e9;
                    Point* next = nullptr;
                    for (Point* n : neighbs) {
                        double d = Line::kDistance(candidate.start,
                                                    *cur, *n);
                        if (d < best) { best = d; next = n; }
                    }
                    if (best > 0.75) break;
                    cur = next;
                }

                if (candidate.manhattan() >= MIN_LINE_LEN)
                    lines.emplace_back(candidate);
            }
        }

        std::cerr << "\r" << std::fixed << std::setprecision(1)
                  << 100.0 * y / M.height() << "% " << std::flush;
    }
    std::cerr << "\r100%   \n";
    return lines;
}

// Clip infinite line to a rectangle
std::pair<Point,Point> clipToRect(double slope, double bias,
                                  int W, int H) {
    std::vector<Point> ps;

    double y0 = bias, yW = slope * W + bias;
    if (0 <= y0 && y0 <= H) ps.emplace_back(0, static_cast<int>(y0), true);
    if (0 <= yW && yW <= H) ps.emplace_back(W, static_cast<int>(yW), true);

    if (slope != 0.0) {
        double x0 = -bias / slope;
        double xH = (H - bias) / slope;
        if (0 <= x0 && x0 <= W) ps.emplace_back(static_cast<int>(x0), 0, true);
        if (0 <= xH && xH <= W) ps.emplace_back(static_cast<int>(xH), H, true);
    }
    if (ps.size() < 2)
        throw std::runtime_error("Line does not cross rectangle.");

    std::sort(ps.begin(), ps.end(),
              [](const Point& a, const Point& b){ return a.x < b.x; });
    return {ps.front(), ps.back()};
}

// Add sketch-style long support strokes
std::vector<Line> addSupportLines(std::vector<Line>& base,
                                  const cv::Size& sz,
                                  int count = 30,
                                  std::pair<double,double> shiftRange={2,5})
{
    std::sort(base.begin(), base.end(),
              [](const Line& a, const Line& b){
                  return a.manhattan() > b.manhattan();
              });

    std::vector<Line> supports;
    std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<> rnd(shiftRange.first, shiftRange.second);

    const int minDist = 10;

    for (int i = 0; i < count && i < (int)base.size(); ++i) {

        // skip if too close to earlier supports
        bool skip = false;
        if (i > 3) {
            for (int j = 0; j < i; ++j) {
                if (std::abs(base[i].start.x - base[j].start.x) < minDist &&
                    std::abs(base[i].start.y - base[j].start.y) < minDist)
                { skip = true; break; }
            }
        }
        if (skip) continue;

        Line ln = base[i];
        Point s = (ln.start.x < ln.end.x) ? ln.start : ln.end;
        Point e = (ln.start.x >= ln.end.x) ? ln.start : ln.end;

        int shift = static_cast<int>(rnd(rng) *
                    std::min(sz.width, sz.height));

        if (shift > std::abs(shift * ln.slope)) {
            s.x -= shift; s.y -= int(shift * ln.slope);
            e.x += shift; e.y += int(shift * ln.slope);
        } else {
            s.x -= int(shift / ln.slope); s.y -= shift;
            e.x += int(shift / ln.slope); e.y += shift;
        }

        auto clipped = clipToRect(ln.slope, ln.bias,
                                  sz.width-1, sz.height-1);
        if (s.x < 0 || s.x >= sz.width || s.y < 0 || s.y >= sz.height)
            s = clipped.first;
        if (e.x < 0 || e.x >= sz.width || e.y < 0 || e.y >= sz.height)
            e = clipped.second;

        supports.emplace_back(s, e);
    }
    return supports;
}

// Render primitives onto a BGR canvas
cv::Mat renderLines(const cv::Size& sz, std::vector<Line>& lines,
                    const cv::Scalar& lineColor    = {0,0,0},
                    const cv::Scalar& bgColor      = {235,250,250},
                    int  thick                     = 6,
                    int  supportThick              = 1)
{
    cv::Mat canvas(sz, CV_8UC3, bgColor);

    for (const auto& ln : lines)
        cv::line(canvas,
                 {ln.start.x, ln.start.y},
                 {ln.end.x  , ln.end.y},
                 lineColor, thick, cv::LINE_AA);

    auto support = addSupportLines(lines, sz);
    for (const auto& ln : support)
        cv::line(canvas,
                 {ln.start.x, ln.start.y},
                 {ln.end.x  , ln.end.y},
                 lineColor, supportThick, cv::LINE_AA);

    return canvas;
}

// ---------------------------------------------------------------------------
//                               CLI helpers
// ---------------------------------------------------------------------------

struct Args {
    std::filesystem::path imagePath = "examples/house_realistic.png";
    std::filesystem::path outDir    = "out";
    double strength      = 10.0;
    double mergeAngle    = 60.0;   // kept for parity; not used
    double mergeDist     = 15.0;   // kept for parity; not used
    double jitterAmp     = 0.0;    // kept for parity; not used
};

Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key == "--out_dir" && i+1 < argc) a.outDir = argv[++i];
        else if (key == "--strength"  && i+1 < argc) a.strength  = std::stod(argv[++i]);
        else if (key == "--merge_angle" && i+1 < argc) a.mergeAngle = std::stod(argv[++i]);
        else if (key == "--merge_dist"  && i+1 < argc) a.mergeDist  = std::stod(argv[++i]);
        else if (key == "--jitter"      && i+1 < argc) a.jitterAmp  = std::stod(argv[++i]);
        else if (key[0] != '-') a.imagePath = key;   // positional
        else {
            std::cerr << "Unknown option: " << key << "\n";
            std::exit(EXIT_FAILURE);
        }
    }
    return a;
}

// ---------------------------------------------------------------------------
//                                    main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    std::ios::sync_with_stdio(false);

    Args args = parseArgs(argc, argv);
    std::cout << "Processing " << args.imagePath << " …\n";

    cv::Mat img = cv::imread(args.imagePath.string(), cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "Could not read image\n";
        return EXIT_FAILURE;
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    cv::Mat edges = extractEdges(gray);
    auto lines    = extractLines(edges);

    std::filesystem::create_directories(args.outDir);
    cv::imwrite((args.outDir / (args.imagePath.stem().string()+"_edges.png")).string(),
                edges);

    cv::Mat sketch = renderLines(edges.size(), lines);
    cv::imwrite((args.outDir / (args.imagePath.stem().string()+"_lines.png")).string(),
                sketch);

    std::cout << "Results saved to " << std::filesystem::absolute(args.outDir)
              << "\n";
    return 0;
}