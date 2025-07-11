/**
 * nmap2surfaces.cpp – planar patch extraction **and** 3‑D reconstruction
 * -------------------------------------------------------------------------
 *  macOS / clang 15  ·  OpenCV 4.9  ·  Eigen 3.4
 *
 * 2025‑06‑14  —  *Basis‑from‑vertices* version
 *   ⤷ previous edge‑pair method failed when polygons did not share identical
 *     edge segments.  We now derive the 2‑D projection of every world axis
 *     **directly from vertex triples**:
 *       – two vertices that differ only in X give a sample of the +X vector;
 *       – differ only in Y → +Y sample; differ only in Z → +Z sample.
 *     The three averaged directions form the columns of the 2×3 basis matrix.
 *
 * Build
 *   g++ nmap2surfaces.cpp -std=c++17 `pkg-config --cflags --libs opencv4 eigen3` -o nmap2surfaces
 */

#include "converter_lib/NMap2SurfacesConverter.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_map>
#include <random>
#include <iostream>
#include <cmath>
#include <memory>
#include <fstream>
#include <filesystem>

namespace converter_lib {

// ---------- helper to fetch option or default --------------------------------
static float optF(const Options& o, const std::string& key, float def) {
    auto it = o.params.find(key); return it==o.params.end()? def : std::stof(it->second);
}
static std::string optS(const Options& o,const std::string&key,const std::string&def){
    auto it=o.params.find(key); return it==o.params.end()?def:it->second;
}

// ----------------------------- data structures ---------------------------
struct Line;    // Forward declaration
struct Surface; // Forward declaration
using PointPtr = std::shared_ptr<struct Point>;
using LinePtr  = std::shared_ptr<Line>;
struct Point {
    cv::Point2f          uv_coords;
    cv::Vec3f            world_coords{};
    std::vector<std::weak_ptr<Line>> lines;   // weak – нет цикла
    bool visited;
    explicit Point(const cv::Point2f& uv)
        : uv_coords(uv), world_coords(0,0,0), visited(false) {}
};

struct Surface {
    cv::Vec3f direction;                   // world normal (axis‑aligned)
    std::vector<cv::Point2f> corners;      // visible polygon in image px
    std::vector<LinePtr> calculated_lines; 

    Surface(const cv::Vec3f& dir, std::vector<cv::Point2f>&& crns) 
        : direction(dir), corners(std::move(crns)), calculated_lines() {}
};


struct Line : public std::enable_shared_from_this<Line> {
    PointPtr a, b;  // Pointer to Point
    std::vector<cv::Vec3f> enclosures;
    cv::Vec3f autodetected_direction;

   static LinePtr make(const PointPtr& pa,
                        const PointPtr& pb,
                        const cv::Vec3f& dir)
    {
        auto lp = LinePtr(new Line(pa, pb));
        lp->register_with_points();
        lp->enclosures.emplace_back(dir);
        
        return lp;
    }

private:
    Line(const PointPtr& pa, const PointPtr& pb) : a(pa), b(pb) {}

    void register_with_points()
    {
        auto self = shared_from_this();           // ← теперь можно
        a->lines.emplace_back(self);
        b->lines.emplace_back(self);
    }

public:
    /* ---------- геометрия оставляем почти без изменений ---------- */
    float rot() const {
        cv::Point2f v(b->uv_coords.x - a->uv_coords.x, b->uv_coords.y - a->uv_coords.y);
        // atan2 returns [-π, π], dividing by π gives [-1, 1]
        // For vector (1,1), atan2(1,1) = π/4, so result should be 0.25
        return std::atan2(v.y, v.x) / M_PI;
    }
    float rot_abs() const {
        cv::Point2f v(b->uv_coords.x - a->uv_coords.x, b->uv_coords.y - a->uv_coords.y);
        return std::abs(std::atan2(v.y, v.x) / M_PI);
    }

    cv::Vec3f dof() const {
        cv::Vec3f s(0,0,0);
        for (const auto& e : enclosures) {
            s[0] += std::abs(e[0]);
            s[1] += std::abs(e[1]);
            s[2] += std::abs(e[2]);
        }
        return cv::Vec3f(s[0]==0, s[1]==0, s[2]==0);
    }
    int  dof_size() const { auto d = dof(); return d[0]+d[1]+d[2]; }

    std::string dof_str() const {
        static const char* axis[]{"X ","Y ","Z "};
        auto d = dof();
        std::string r;
        for(int i=0;i<3;++i) if (d[i]) r += axis[i];
        return r;
    }
};

// ----------------------------- helpers -----------------------------------
static inline int domAxis(const cv::Vec3f& n){
    float ax=fabs(n[0]), ay=fabs(n[1]), az=fabs(n[2]);
    if(ax>=ay && ax>=az) return n[0]>=0?0:1;   // +X / −X
    if(ay>=ax && ay>=az) return n[1]>=0?2:3;   // +Y / −Y
    return n[2]>=0?4:5;                        // +Z / −Z
}

// hash/eq for bucketed Point2f (≈1 px tolerance) --------------------------
struct PtHash{ size_t operator()(const cv::Point2f& p) const noexcept{
    return (static_cast<size_t>(lround(p.x))<<20) ^ static_cast<size_t>(lround(p.y)); }};
struct PtEq{ bool operator()(const cv::Point2f&a,const cv::Point2f&b) const noexcept{
    return fabs(a.x-b.x)<1.0f && fabs(a.y-b.y)<1.0f; }};

static void removeCollinear(std::vector<cv::Point>& poly,float AREA_EPS){
    if(poly.size()<3) return; bool ch;
    do{ ch=false; for(size_t i=0;i<poly.size();++i){ cv::Point2f a=poly[(i+poly.size()-1)%poly.size()]; cv::Point2f b=poly[i]; cv::Point2f c=poly[(i+1)%poly.size()]; cv::Point2f v1=b-a,v2=c-b; float area=fabs(v1.x*v2.y-v1.y*v2.x); if(area<=AREA_EPS){ poly.erase(poly.begin()+static_cast<long>(i)); ch=true; break; } } }while(ch&&poly.size()>2);
}

// ----------------------------- stage 1 – surface extraction --------------
static std::vector<Surface> extract_surfaces(
    const cv::Mat& img,
    const cv::Mat& dirIdx,
    const cv::Vec3f AXIS[6],
    float MIN_MAG,
    int MIN_AREA,
    bool DISCARD_CORNERS,
    float POLY_EPS,
    float AREA_EPS)
{
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
    std::vector<Surface> S;

    for (int d = 0; d < 6; ++d) {
        // Create mask for current direction
        cv::Mat mask(img.rows, img.cols, CV_8U);
        for (int y = 0; y < img.rows; ++y) {
            const int8_t* di = dirIdx.ptr<int8_t>(y);
            uchar* m = mask.ptr<uchar>(y);
            for (int x = 0; x < img.cols; ++x) {
                m[x] = (di[x] == d ? 255 : 0);
            }
        }

        // Process mask
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::Mat lbl, stats, cent;
        int n = cv::connectedComponentsWithStats(mask, lbl, stats, cent, 8, CV_32S);

        // Process each connected component
        for (int l = 1; l < n; ++l) {
            // Skip small components
            if (stats.at<int>(l, cv::CC_STAT_AREA) < MIN_AREA) {
                continue;
            }

            // Get component bounds
            int x0 = stats.at<int>(l, cv::CC_STAT_LEFT);
            int y0 = stats.at<int>(l, cv::CC_STAT_TOP);
            int w = stats.at<int>(l, cv::CC_STAT_WIDTH);
            int h = stats.at<int>(l, cv::CC_STAT_HEIGHT);

            // Skip corner components if requested
            if (DISCARD_CORNERS) {
                if ((x0 == 0 && y0 == 0) ||
                    (x0 + w == img.cols && y0 == 0) ||
                    (x0 == 0 && y0 + h == img.rows) ||
                    (x0 + w == img.cols && y0 + h == img.rows)) {
                    continue;
                }
            }

            // Extract component mask
            cv::Rect roi(x0, y0, w, h);
            cv::Mat cm(roi.size(), CV_8U, cv::Scalar::all(0));
            for (int y = roi.y; y < roi.y + h; ++y) {
                const int* ll = lbl.ptr<int>(y);
                uchar* cc = cm.ptr<uchar>(y - roi.y);
                for (int x = roi.x; x < roi.x + w; ++x) {
                    if (ll[x] == l) {
                        cc[x - roi.x] = 255;
                    }
                }
            }

            // Find and process contours
            std::vector<std::vector<cv::Point>> cont;
            cv::findContours(cm, cont, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if (cont.empty()) {
                continue;
            }

            // Adjust contour points to global coordinates
            for (auto& p : cont[0]) {
                p += roi.tl();
            }

            // Approximate polygon
            double peri = cv::arcLength(cont[0], true);
            std::vector<cv::Point> poly;
            cv::approxPolyDP(cont[0], poly, peri * POLY_EPS, true);
            removeCollinear(poly, AREA_EPS);

            // Skip invalid polygons
            if (poly.size() < 3) {
                continue;
            }

            // Convert to floating point coordinates
            std::vector<cv::Point2f> fp;
            fp.reserve(poly.size());
            for (auto& p : poly) {
                fp.emplace_back(p.x, p.y);
            }

            // Add surface
            S.emplace_back(Surface{AXIS[d], std::move(fp)});
        }
    }
    return S;
}
static std::string axis_str(int axis) {
    switch (axis) {
        case 0: return "X";
        case 1: return "Y";
        case 2: return "Z";
        default: return "";
    }
}

// -------------------------------------------------------------------------
//  reconstruction with *fixed* image basis                                
// -------------------------------------------------------------------------
static void calculated_world_coords(std::vector<PointPtr>& points, PointPtr current, std::vector<cv::Point2f> basis) {
    current->visited = true;
    
    for (const auto& line : current->lines) {
        if (auto line_ptr = line.lock()) {
            PointPtr other = (line_ptr->a == current) ? line_ptr->b : line_ptr->a;
            if (other != current && !other->visited) {
                // Get the degrees of freedom for this line
                cv::Vec3f dof = line_ptr->dof();
                
                // Calculate the UV displacement between points
                cv::Point2f uv_delta = other->uv_coords - current->uv_coords;
                
                // Calculate world coordinates based on basis vectors and DOF
                cv::Vec3f world_delta(0, 0, 0);
                for (int i = 0; i < 3; i++) {
                    if (dof[i] > 0) {  // If this dimension is free
                        // Project UV delta onto corresponding basis vector
                        world_delta[i] = uv_delta.dot(basis[i]);
                    }
                }
                other->world_coords = current->world_coords + world_delta;
                
                // std::cout << "Basis: " << basis[0] << ", " << basis[1] << ", " << basis[2] 
                //           << " | Current UV: " << current->uv_coords 
                //           << " | Other UV: " << other->uv_coords 
                //           << " | Current World: " << current->world_coords 
                //           << " | DOF: " << dof << std::endl;
                
                // Recursively process the child point
                calculated_world_coords(points, other, basis);
            }
        }
    }
}

static void reconstruct(std::vector<Surface>& S)
{
    using LinesByAngle = std::unordered_map<float, std::vector<LinePtr>>;

    constexpr float MIN_POINT_DIST = 5.f;
    constexpr float DEG_EPS        = 0.05f;

    LinesByAngle         line_dict;
    std::vector<float>   keys;
    std::vector<PointPtr> points;   // теперь shared_ptr
    std::vector<LinePtr>  all_lines;

    /* ---------- поиск/создание точки ---------- */
    auto find_or_add_point = [&](const cv::Point2f& uv)->PointPtr
    {
        for (auto& p : points)
            if ( std::abs(p->uv_coords.x-uv.x) < MIN_POINT_DIST &&
                 std::abs(p->uv_coords.y-uv.y) < MIN_POINT_DIST )
                return p;

        auto np = std::make_shared<Point>(uv);
        points.emplace_back(np);
        return np;
    };

    /* ---------- обход поверхностей ---------- */
    for (auto& surf : S)
    {
        const auto& c = surf.corners;
        for (size_t i = 0; i < c.size(); ++i)
        {
            size_t nxt = (i+1)%c.size();
            bool swap  = (c[i].x+c[i].y) > (c[nxt].x+c[nxt].y);

            PointPtr p1 = find_or_add_point( swap ? c[nxt] : c[i]   );
            PointPtr p2 = find_or_add_point( swap ? c[i]   : c[nxt] );

            /* ―― создаём линию через фабрику ―― */
            auto line = Line::make(p1, p2, surf.direction);
            all_lines.push_back(line);
            surf.calculated_lines.emplace_back(line);
            float ang = line->rot();          // |angle|
            /* группировка по углу */
            auto key_it = std::find_if(keys.begin(), keys.end(),
                        [&](float k){ return std::abs(k-ang) < DEG_EPS; });

            float key = (key_it==keys.end()? ang : *key_it);

            if (key_it==keys.end()) keys.push_back(key);

            auto& bucket = line_dict[key];
            bool  unique = true;
            for (auto& ex : bucket)
            {
                if ( std::abs(ex->a->uv_coords.x - line->a->uv_coords.x) < MIN_POINT_DIST &&
                     std::abs(ex->a->uv_coords.y - line->a->uv_coords.y) < MIN_POINT_DIST &&
                     std::abs(ex->b->uv_coords.x - line->b->uv_coords.x) < MIN_POINT_DIST &&
                     std::abs(ex->b->uv_coords.y - line->b->uv_coords.y) < MIN_POINT_DIST )
                {
                    ex->enclosures.emplace_back(surf.direction);
                    surf.calculated_lines.emplace_back(ex);
                    unique = false;
                    break;
                }
            }
            if (unique)  bucket.emplace_back(line);
        }
    }

    /* ---------- вывод отладочной инфы ---------- */
    // std::cout << "Lines with coordinates:\n";
    // for (float key : keys)
    //     for (auto& ln : line_dict[key])
            
    //         std::cout << "  (" << ln->a->uv_coords << ") – (" << ln->b->uv_coords << ")\n";

    // Initialize basis vector with 3 points
    std::vector<cv::Point2f> basis(3, cv::Point2f(0, 0));
    std::vector<float> angular_basis(3, 0.0f);

    for (const auto& key : keys) {
        for (const auto& line : line_dict[key]) {
            if (line->dof_size() == 1) {
                auto dof = line->dof();
                float x = std::round(std::cos(key * M_PI) * 1000) / 1000;
                float y = std::round(std::sin(key * M_PI) * 1000) / 1000;
                for (int i = 0; i < 3; i++) {
                    if (dof[i] == 1 && basis[i].x == 0 && basis[i].y == 0) {
                        basis[i] = cv::Point2f(x, y);
                        angular_basis[i] = key;
                    }
                }
            }
            all_lines.push_back(line);
        }
    }
    for (int i = 0; i < 3; i ++){ 
        auto b = basis[i];
        for (auto& line : line_dict[angular_basis[i]]) {
            auto dof = line->dof();
      
            if (dof[i] == 1) {
                // If basis axis is in dof, set enclosures to I - axis vector
                cv::Vec3f new_enclosure(1, 1, 1);
                new_enclosure[i] = 0;
                line->enclosures.clear();
                line->enclosures.push_back(new_enclosure);
            }
        }
    }

    // std::cout << "Basis angles: " << basis[0] << ", " << basis[1] << ", " << basis[2] << std::endl;
    // uint index = 0;
    // for (const auto& b : basis) {
    //     std::cout << "For axis " << axis_str(index) << " found " << line_dict[angular_basis[index]].size() << " edges" << std::endl;
    //     for (const auto& line : line_dict[angular_basis[index]]) {
    //         std::cout << "  Line from (" << line->a->uv_coords.x << "," << line->a->uv_coords.y
    //                 << ") to (" << line->b->uv_coords.x << "," << line->b->uv_coords.y << ")"
    //                 << ". It has " << line->dof_size() << " dof: " << line->dof_str() << std::endl;
    //         std::cout << std::endl;
    //     }
    //     index++;
    // }
    // auto origin = points[0];
    // origin->world_coords = cv::Vec3f(0,0,0);
    // origin->visited = true;
    // calculated_world_coords(points, origin, basis);
    // std::cout << "\nPoint visit status:\n";
    // for (size_t i = 0; i < points.size(); i++) {
    //     std::cout << "Point " << i << ": " 
    //    <<"(" << points[i]->uv_coords.x << ","
    //               << points[i]->uv_coords.y << "), ("
    //     << points[i]->world_coords[0] << ","
    //               << points[i]->world_coords[1] << ","
    //               << points[i]->world_coords[2] << ")" << std::endl;
            
    //         std::cout << std::endl;
    // }

    std::ofstream outfile("output.txt");
    outfile << "\nSurface Information:\n";
    for (size_t i = 0; i < S.size(); i++) {
        const auto& surf = S[i];
        outfile << "Surface " << i << ":\n";
        outfile << "  Direction: (" << surf.direction[0] << ", " 
                << surf.direction[1] << ", " << surf.direction[2] << ")\n";
        outfile << "  Lines with world coordinates:\n";
        
        for (const auto& line : surf.calculated_lines) {
            outfile << "    Line from (" << line->a->world_coords[0] << ","
                    << line->a->world_coords[1] << "," << line->a->world_coords[2] 
                    << ") to (" << line->b->world_coords[0] << ","
                    << line->b->world_coords[1] << "," << line->b->world_coords[2] 
                    << ")\n";
        }
        outfile << std::endl;
    }
    outfile.close();
    
}

// -----------------------------------------------------------------------------
/// core
void NMap2SurfacesConverter::convert(const std::string& inPath,
                                     const std::string& outPath,
                                     const Options& opts)
{
    // ---------- settings -----------------------------------------------------
    float MIN_MAG   = optF(opts, "min_mag", 0.25f);    // ignore almost-flat normals
    int   MIN_AREA  = static_cast<int>(optF(opts, "min_area", 10));       // discard tiny blobs
    bool  DISCARD   = optS(opts, "discard_corners", "true") == "true";     // skip image-corner patches
    float POLYEPS   = optF(opts, "poly_eps", 0.007f);   // ε for approxPolyDP
    float AREA_EPS  = optF(opts, "area_eps", 0.5f);     // collinear test

    // ---------- load image ---------------------------------------------------
    cv::Mat img = cv::imread(inPath, cv::IMREAD_COLOR);
    if (img.empty()) {
        throw std::runtime_error("Cannot load " + inPath);
    }

    // --------------------- parameters & per-pixel axis class -------------
    cv::Mat dlab(img.rows, img.cols, CV_8S);
    for (int y = 0; y < img.rows; ++y) {
        const cv::Vec3b* row = img.ptr<cv::Vec3b>(y);
        int8_t*           dl = dlab.ptr<int8_t>(y);

        for (int x = 0; x < img.cols; ++x) {
            // restore [-1,1] normal from RGB
            float nx = row[x][2] / 255.f * 2.f - 1.f;
            float ny = row[x][1] / 255.f * 2.f - 1.f;
            float nz = row[x][0] / 255.f * 2.f - 1.f;
            cv::Vec3f n(nx, ny, nz);

            dl[x] = (cv::norm(n) < MIN_MAG) ? -1 : domAxis(n);
        }
    }

    const cv::Vec3f AXIS[6] = {
        {+1, 0, 0}, {-1, 0, 0},
        {0, +1, 0}, {0, -1, 0},
        {0, 0, +1}, {0, 0, -1}
    };

    // --------------------------- pipeline --------------------------------
    auto surfaces = extract_surfaces(img, dlab, AXIS,
                                     MIN_MAG, MIN_AREA,
                                     DISCARD, POLYEPS, AREA_EPS);

    // std::cout << "Surfaces: " << surfaces.size() << '\n';
    // for(size_t i=0;i<surfaces.size();++i){
    //     std::cout << "Surface " << i << " direction: " << surfaces[i].direction << '\n';
    //     for(const auto& p: surfaces[i].corners){
    //         std::cout << "  (" << p.x << ", " << p.y << ")\n";
    //     }
    // }

    reconstruct(surfaces);

    // ------------------------ simple overlay -----------------------------
    cv::Mat vis(img.size(), CV_8UC3, cv::Scalar::all(0));

    std::mt19937 rng(0xC0FFEE);                  // deterministic colours
    std::uniform_int_distribution<int> randCol(64, 255);

    // draw each planar patch
    for (const auto& s : surfaces) {
        std::vector<cv::Point> poly;
        poly.reserve(s.corners.size());
        for (const auto& p : s.corners)
            poly.emplace_back(cv::Point(cvRound(p.x), cvRound(p.y)));

        cv::Scalar col(randCol(rng), randCol(rng), randCol(rng));
        std::vector<std::vector<cv::Point>> arr{poly};

        cv::drawContours(vis, arr, -1, col, cv::FILLED, cv::LINE_AA);
    }

    // blend overlay with the original normal map for context
    cv::Mat blend;
    cv::addWeighted(img, 0.35, vis, 0.65, 0.0, blend);

    // ---------- store output -------------------------------------------------
    std::filesystem::path out = outPath.empty()
        ? std::filesystem::path(inPath).replace_extension("_surfaces.png")
        : std::filesystem::path(outPath);
    
    if (!cv::imwrite(out.string(), blend))
        throw std::runtime_error("Failed to save " + out.string());
    else
        std::cout << "Overlay written to " << out << '\n';
}

} // namespace converter_lib