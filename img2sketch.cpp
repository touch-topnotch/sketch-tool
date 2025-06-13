/**
 * @file img2sketch.cpp
 * @brief Algorithmic photo to hand-sketch converter
 * 
 * This program converts a photo into a hand-drawn sketch style image.
 * The process involves:
 * 1. Edge detection using Canny algorithm
 * 2. Line extraction and vectorization
 * 3. Rendering with support lines for artistic effect
 * 
 * @author Original Python version by unknown, C++ port by Claude
 * @version 1.0
 */

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

/**
 * @brief Point class representing a 2D point with value
 */
class Point {
public:
    int x, y;
    int value;

    Point(int x = 0, int y = 0, int v = 0) : x(x), y(y), value(v) {}
    
    operator bool() const { return value == 1; }
    operator int() const { return value; }
    
    std::string toString() const {
        return "Point(x: " + std::to_string(x) + 
               ", y: " + std::to_string(y) + 
               ", v: " + std::to_string(value) + ")";
    }
    
    std::pair<int, int> toPair() const {
        return {x, y};
    }
};

/**
 * @brief Line class representing a line segment between two points
 */
class Line {
public:
    Point start;
    Point end;
    double slope;
    double bias;

    Line(const Point& start, const Point& end) 
        : start(start), end(end) {
        slope = calcSlope(start, end);
        bias = calcBias(start, slope);
    }

    void updateEnd(const Point& newEnd) {
        end = newEnd;
        slope = calcSlope(start, end);
        bias = calcBias(start, slope);
    }

    int manhDist() const {
        return calcManhDist(start, end);
    }

    static int calcManhDist(const Point& start, const Point& end) {
        return std::abs(start.x - end.x) + std::abs(start.y - end.y);
    }

    static double calcEucDist(const Point& start, const Point& end) {
        return std::sqrt(std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2));
    }

    static double calcSlope(const Point& start, const Point& end) {
        if (end.x - start.x == 0) return 10000;
        return static_cast<double>(end.y - start.y) / (end.x - start.x);
    }

    static double calcBias(const Point& anchor, double slope) {
        return anchor.y - slope * anchor.x;
    }

    static double calcAngle(double slope1, double slope2) {
        double denominator = 1 + slope1 * slope2;
        if (std::abs(denominator) < 1e-10) {
            return M_PI / 2;
        }
        return std::atan(std::abs((slope1 - slope2) / denominator));
    }

    static double calcKDistance(const Point& A, const Point& B, const Point& C, double penalty = 1000.0) {
        double ax = A.x, ay = A.y;
        double bx = B.x, by = B.y;
        double cx = C.x, cy = C.y;

        double ABx = bx - ax, ABy = by - ay;
        double ACx = cx - ax, ACy = cy - ay;
        double ab2 = ABx * ABx + ABy * ABy;

        if (ab2 == 0) {
            throw std::runtime_error("A and B must be distinct to define a line");
        }

        double t = (ACx * ABx + ACy * ABy) / ab2;

        if (t <= 1) {
            return penalty;
        }

        double distance = std::abs(ABx * ACy - ABy * ACx) / std::sqrt(ab2);
        return distance;
    }
};

/**
 * @brief Matrix class for handling the edge detection image
 */
class Matrix {
public:
    int width;
    int height;
    std::vector<std::vector<Point>> matrix;

    Matrix(const cv::Mat& edges) {
        width = edges.cols;
        height = edges.rows;
        std::cout << "width: " << width << " height: " << height << std::endl;

        matrix.resize(height);
        for (int y = 0; y < height; y++) {
            matrix[y].resize(width);
            for (int x = 0; x < width; x++) {
                matrix[y][x] = Point(x, y, static_cast<int>(std::ceil(edges.at<uchar>(y, x) / 255.0)));
            }
        }
    }

    std::vector<Point> getNeighbs(const Point& point, const Point& krome) {
        int x = point.x;
        int y = point.y;
        std::vector<std::pair<int, int>> ids = {
            {x-1, y-1}, {x, y-1}, {x+1, y-1},
            {x-1, y},             {x+1, y},
            {x-1, y+1}, {x, y+1}, {x+1, y+1}
        };
        std::pair<int, int> krome_id = {krome.x, krome.y};
        std::vector<Point> res;

        for (const auto& id : ids) {
            if (!(0 <= id.first && id.first < width && 0 <= id.second && id.second < height)) {
                continue;
            }
            if ((*this)[id] && id != krome_id) {
                res.push_back((*this)[id]);
            }
        }
        return res;
    }

    std::vector<Point> getBoarderNeighbs(const Point& point) {
        int x = point.x;
        int y = point.y;
        std::vector<std::pair<int, int>> ids = {
            {x+1, y}, {x-1, y+1}, {x, y+1}, {x+1, y+1}
        };
        std::vector<Point> res;

        for (const auto& id : ids) {
            if (!(0 <= id.first && id.first < width && 0 <= id.second && id.second < height)) {
                continue;
            }
            if ((*this)[id]) {
                res.push_back((*this)[id]);
            }
        }
        return res;
    }

    Point& operator[](const std::pair<int, int>& key) {
        return matrix[key.second][key.first];
    }

    void visualize(const Point& point, int size) {
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                std::cout << static_cast<int>((*this)[{j + point.x - size/2, i + point.y - size/2}]) << " ";
            }
            std::cout << std::endl;
        }
    }

    void visualizeArray(const std::vector<Point>& points) {
        int x_min = std::numeric_limits<int>::max();
        int x_max = std::numeric_limits<int>::min();
        int y_min = std::numeric_limits<int>::max();
        int y_max = std::numeric_limits<int>::min();

        for (const auto& p : points) {
            x_min = std::min(x_min, p.x);
            x_max = std::max(x_max, p.x);
            y_min = std::min(y_min, p.y);
            y_max = std::max(y_max, p.y);
        }

        for (int y = y_min; y <= y_max; y++) {
            for (int x = x_min; x <= x_max; x++) {
                bool found = false;
                for (const auto& p : points) {
                    if (p.x == x && p.y == y) {
                        found = true;
                        break;
                    }
                }
                std::cout << (found ? "1" : "0");
            }
            std::cout << std::endl;
        }
    }
};

// Function declarations
std::vector<Line> extractLines(const cv::Mat& edges, int minLength = 10);
std::pair<Point, Point> clipLineToRect(double slope, double bias, int W, int H);
std::vector<Line> addSupportLines(const std::vector<Line>& lines, const cv::Size& shape, int count = 30, 
                                const std::pair<double, double>& xShiftRange = {0.2, 0.5});
cv::Mat renderLines(const cv::Size& screenSize, const std::vector<Line>& lines,
                   const cv::Scalar& lineColor = cv::Scalar(0, 0, 0),
                   const cv::Scalar& backgroundColor = cv::Scalar(250, 250, 235),
                   int lineThickness = 6,
                   int supportLineThickness = 1);

/**
 * @brief Main processing function
 * @param inputPath Path to input image
 * @param outputPath Path to save output image
 * @param strength Gradient scaling for normal map
 * @param mergeAngle Max angle between lines to merge
 * @param mergeDist Max endpoint distance to merge lines
 * @param jitter Stroke jitter amplitude
 * @return true if processing successful, false otherwise
 */
bool process(const fs::path& inputPath, const fs::path& outputPath,
            double strength = 10.0, double mergeAngle = 60.0,
            double mergeDist = 15.0, double jitter = 0.0) {
    try {
        std::cout << "Processing " << inputPath << " ..." << std::endl;
        
        cv::Mat img = cv::imread(inputPath.string());
        if (img.empty()) {
            throw std::runtime_error("Failed to load image: " + inputPath.string());
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::cout << "Extracting edges ..." << std::endl;
        cv::Mat edges;
        cv::Canny(gray, edges, 50, 250);
        if (cv::ximgproc::thinning) {
            cv::ximgproc::thinning(edges, edges);
        }

        std::vector<Line> lines = extractLines(edges);

        // Create output directory if it doesn't exist
        fs::create_directories(outputPath.parent_path());

        // Save edges image
        cv::imwrite((outputPath.parent_path() / (outputPath.stem().string() + "_edges.png")).string(), edges);

        // Render and save lines image
        cv::Mat sketchImg = renderLines(edges.size(), lines);
        cv::imwrite((outputPath.parent_path() / (outputPath.stem().string() + "_lines.png")).string(), sketchImg);

        std::cout << "Saved results to " << outputPath.parent_path() << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <input_image> <output_image> [strength] [merge_angle] [merge_dist] [jitter]" << std::endl;
        return 1;
    }

    fs::path inputPath = argv[1];
    fs::path outputPath = argv[2];
    
    double strength = (argc > 3) ? std::stod(argv[3]) : 10.0;
    double mergeAngle = (argc > 4) ? std::stod(argv[4]) : 60.0;
    double mergeDist = (argc > 5) ? std::stod(argv[5]) : 15.0;
    double jitter = (argc > 6) ? std::stod(argv[6]) : 0.0;

    return process(inputPath, outputPath, strength, mergeAngle, mergeDist, jitter) ? 0 : 1;
}

/**
 * @brief Extract lines from edge detection image
 * @param edges Edge detection image
 * @param minLength Minimum length of lines to extract
 * @return Vector of extracted lines
 */
std::vector<Line> extractLines(const cv::Mat& edges, int minLength) {
    Matrix matrix(edges);
    std::vector<Line> lines;
    
    std::cout << "Extracting lines ..." << std::endl;
    for (int y = 0; y < matrix.height; y++) {
        for (int x = 0; x < matrix.width; x++) {
            std::cout << "\rExtracting lines: " << y << "/" << matrix.height 
                     << " [" << (static_cast<double>(y) / matrix.height * 100) << "%]" << std::flush;
            
            Point p = matrix[{x, y}];
            if (!p) continue;
            
            std::vector<Point> neighbs = matrix.getBoarderNeighbs(p);
            
            std::function<Line(const Point&, Line)> goByLine = [&](const Point& currentPoint, Line lineCandidate) -> Line {
                std::vector<Point> neighbs;
                if (currentPoint.x == lineCandidate.end.x && currentPoint.y == lineCandidate.end.y) {
                    neighbs = matrix.getNeighbs(currentPoint, lineCandidate.start);
                } else {
                    neighbs = matrix.getNeighbs(currentPoint, lineCandidate.end);
                }

                int n = neighbs.size();
                Point* nextPoint = nullptr;
                lineCandidate.updateEnd(currentPoint);
                
                if (n < 2) {
                    currentPoint.value = 0;
                    if (n == 1) {
                        nextPoint = &neighbs[0];
                    }
                    if (n == 0) {
                        return lineCandidate;
                    }
                } else {
                    if (lineCandidate.start.x == currentPoint.x && lineCandidate.start.y == currentPoint.y) {
                        return lineCandidate;
                    }
                    
                    std::vector<double> slopesDiff;
                    for (const auto& neighb : neighbs) {
                        slopesDiff.push_back(Line::calcKDistance(lineCandidate.start, currentPoint, neighb));
                    }
                    
                    auto minIt = std::min_element(slopesDiff.begin(), slopesDiff.end());
                    if (*minIt > 0.75) {
                        return lineCandidate;
                    }
                    nextPoint = &neighbs[std::distance(slopesDiff.begin(), minIt)];
                }
                
                return goByLine(*nextPoint, lineCandidate);
            };
            
            std::vector<Line> lineCandidates;
            for (const auto& neighb : neighbs) {
                lineCandidates.push_back(goByLine(neighb, Line(p, neighb)));
            }
            
            for (const auto& line : lineCandidates) {
                if (line.manhDist() >= minLength) {
                    lines.push_back(line);
                }
            }
        }
    }
    std::cout << std::endl;
    return lines;
}

/**
 * @brief Clip a line to rectangle boundaries
 * @param slope Line slope
 * @param bias Line bias
 * @param W Rectangle width
 * @param H Rectangle height
 * @return Pair of points representing clipped line endpoints
 */
std::pair<Point, Point> clipLineToRect(double slope, double bias, int W, int H) {
    std::vector<Point> pts;

    // Intersection with left edge x=0
    double y0 = bias;
    if (0 <= y0 && y0 <= H) {
        pts.emplace_back(0, static_cast<int>(y0));
    }

    // Intersection with right edge x=W
    double yW = slope * W + bias;
    if (0 <= yW && yW <= H) {
        pts.emplace_back(W, static_cast<int>(yW));
    }

    // If slope is non-zero, check horizontal boundaries
    if (std::abs(slope) > 1e-10) {
        // Intersection with bottom edge y=0
        double x0 = -bias / slope;
        if (0 <= x0 && x0 <= W) {
            pts.emplace_back(static_cast<int>(x0), 0);
        }

        // Intersection with top edge y=H
        double xH = (H - bias) / slope;
        if (0 <= xH && xH <= W) {
            pts.emplace_back(static_cast<int>(xH), H);
        }
    }

    if (pts.size() < 2) {
        throw std::runtime_error("Line does not cross the rectangle.");
    }

    // Sort by x coordinate and take leftmost and rightmost points
    std::sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) { return a.x < b.x; });
    return {pts.front(), pts.back()};
}

/**
 * @brief Add support lines for artistic effect
 * @param lines Original lines
 * @param shape Image shape
 * @param count Number of support lines to add
 * @param xShiftRange Range for x-shift of support lines
 * @return Vector of support lines
 */
std::vector<Line> addSupportLines(const std::vector<Line>& lines, const cv::Size& shape, int count,
                                const std::pair<double, double>& xShiftRange) {
    std::vector<Line> sortedLines = lines;
    std::sort(sortedLines.begin(), sortedLines.end(),
              [](const Line& a, const Line& b) { return a.manhDist() > b.manhDist(); });

    std::vector<Line> createSupports;
    int minDist = 10;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    for (int i = 0; i < count && i < sortedLines.size(); i++) {
        if (i > 3) {
            bool tooClose = false;
            for (int j = 0; j < i; j++) {
                if ((std::abs(sortedLines[i].start.x - sortedLines[j].start.x) < minDist &&
                     std::abs(sortedLines[i].start.y - sortedLines[j].start.y) < minDist) ||
                    (std::abs(sortedLines[i].end.x - sortedLines[j].end.x) < minDist &&
                     std::abs(sortedLines[i].end.y - sortedLines[j].end.y) < minDist)) {
                    tooClose = true;
                    break;
                }
            }
            if (tooClose) continue;
        }

        Point start = (sortedLines[i].start.x < sortedLines[i].end.x) ? 
                     sortedLines[i].start : sortedLines[i].end;
        Point end = (sortedLines[i].start.x >= sortedLines[i].end.x) ? 
                   sortedLines[i].start : sortedLines[i].end;

        int minDim = std::min(shape.width, shape.height);
        std::uniform_int_distribution<> shiftDist(
            static_cast<int>(xShiftRange.first * minDim),
            static_cast<int>(xShiftRange.second * minDim)
        );
        int shift = shiftDist(gen);

        if (shift > std::abs(shift * sortedLines[i].slope)) {
            start = Point(start.x - shift, 
                        static_cast<int>(start.y - shift * sortedLines[i].slope), 1);
            end = Point(end.x + shift,
                       static_cast<int>(end.y + shift * sortedLines[i].slope), 1);
        } else {
            start = Point(static_cast<int>(start.x - shift / sortedLines[i].slope),
                         start.y - shift, 1);
            end = Point(static_cast<int>(end.x + shift / sortedLines[i].slope),
                       end.y + shift, 1);
        }

        Line line(start, end);
        try {
            auto [startClip, endClip] = clipLineToRect(line.slope, line.bias, shape.width, shape.height);
            if (!(0 <= start.x && start.x <= shape.width && 0 <= start.y && start.y <= shape.height)) {
                start = startClip;
            }
            if (!(0 <= end.x && end.x <= shape.width && 0 <= end.y && end.y <= shape.height)) {
                end = endClip;
            }
            createSupports.emplace_back(start, end);
        } catch (const std::exception&) {
            continue;
        }
    }
    
    return createSupports;
}

/**
 * @brief Render lines to an image
 * @param screenSize Size of output image
 * @param lines Lines to render
 * @param lineColor Color of main lines
 * @param backgroundColor Background color
 * @param lineThickness Thickness of main lines
 * @param supportLineThickness Thickness of support lines
 * @return Rendered image
 */
cv::Mat renderLines(const cv::Size& screenSize, const std::vector<Line>& lines,
                   const cv::Scalar& lineColor, const cv::Scalar& backgroundColor,
                   int lineThickness, int supportLineThickness) {
    cv::Mat img(screenSize, CV_8UC3, backgroundColor);

    // Draw main lines
    for (const auto& ln : lines) {
        cv::line(img, 
                cv::Point(ln.start.x, ln.start.y),
                cv::Point(ln.end.x, ln.end.y),
                lineColor, lineThickness);
    }

    // Draw support lines
    std::vector<Line> supportLines = addSupportLines(lines, screenSize);
    for (const auto& ln : supportLines) {
        cv::line(img,
                cv::Point(ln.start.x, ln.start.y),
                cv::Point(ln.end.x, ln.end.y),
                lineColor, supportLineThickness);
    }

    return img;
}
