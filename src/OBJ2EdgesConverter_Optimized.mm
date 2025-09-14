// visible_edges_simple.cpp  —  perspective / ortho edges that match OBJ2NMap camera
// -----------------------------------------------------------------------------
// Build:  clang++ -std=c++17 -O2 visible_edges_simple.cpp -o visible_edges_simple
// Example: ./visible_edges_simple part.obj out.svg --width 1024 --height 768 \
//                    --pos "(10,10,10)" --fov 45 --samples 150
// Same <pos,fov> pair fed into OBJ2NMap gives perfectly aligned normal-map.
// -----------------------------------------------------------------------------

#include "converter_lib/OBJ2EdgesConverter_Optimized.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <filesystem>
#include <optional>
#include <embree4/rtcore.h>          // + добавить
#include <embree4/rtcore_common.h>
#include <embree4/rtcore_ray.h>

namespace converter_lib {

// ───────────────────────── option helpers ───────────────────────────────────
static float optF(const Options& o,const std::string& k,float d){auto it=o.params.find(k);return it==o.params.end()?d:std::stof(it->second);} 
static bool  optE(const Options& o,const std::string& k){return o.params.count(k);} 
static std::string optS(const Options& o,const std::string& k,const std::string& d){auto it=o.params.find(k);return it==o.params.end()?d:it->second;}

static std::array<double,3> parseVec3(const std::string& s){
    if(s.size()<5) return {0,0,0};                         // minimal (x,y)
    std::string body = s.substr(1,s.size()-2);              // drop parens
    size_t p1=body.find(','), p2=body.find(',',p1+1);
    return { std::stod(body.substr(0,p1)),
             std::stod(body.substr(p1+1,p2-p1-1)),
             std::stod(body.substr(p2+1)) };
}

// ───────────────────────── geometry types ───────────────────────────────────
struct Vec3{double x,y,z;};
inline Vec3 operator+(Vec3 a,Vec3 b){return{a.x+b.x,a.y+b.y,a.z+b.z};}
inline Vec3 operator-(Vec3 a,Vec3 b){return{a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec3 operator*(Vec3 v,double s){return{v.x*s,v.y*s,v.z*s};}
inline double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 cross(Vec3 a,Vec3 b){return{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double len(Vec3 v){return std::sqrt(dot(v,v));}
inline Vec3 norm(Vec3 v){double l=len(v);return l? v*(1.0/l):v;}

struct Vec2{double x,y;};
struct Seg {Vec2 a,b;};

// ───────────────────── joinCollinearChains() — швы → длинные линии ─────────
static void joinCollinearChains(std::vector<Seg>& segs,
                                double snap = 0.5,          // квантизация узлов (px)
                                double tol  = 2.0)          // отклонение RDP (px)
{
    if (segs.empty()) return;

    /* ❶ квантуем координаты = надёжно совпадут даже с fp-шумом */
    auto q = [snap](double v){ return std::round(v / snap) * snap; };

    /* ❷ заменяем жёсткое совпадение на допуск "близко" */
    auto near = [](const Vec2& a, const Vec2& b, double eps = 1.0) {
        return std::abs(a.x - b.x) < eps && std::abs(a.y - b.y) < eps;
    };

    /* ❸ строим список рёбер вместо графа */
    std::vector<std::pair<Vec2, Vec2>> edges2;
    for (auto& s : segs) {
        Vec2 a{q(s.a.x), q(s.a.y)}, b{q(s.b.x), q(s.b.y)};
        if (near(a, b, 0.1)) continue;  // нулевая длина
        edges2.emplace_back(a, b);
    }

    /* ❹ функция для поиска следующего ребра */
    auto popNext = [&](const Vec2& cur) -> std::optional<Vec2> {
        for (size_t i = 0; i < edges2.size(); ++i) {
            if (near(edges2[i].first, cur)) {
                Vec2 nxt = edges2[i].second;
                edges2.erase(edges2.begin() + i);
                return nxt;
            }
        }
        return std::nullopt;
    };

    /* ❺ DFS-проход: собираем цепочки */
    std::vector<std::vector<Vec2>> chains;
    
    while (!edges2.empty()) {
        Vec2 start = edges2[0].first;
        Vec2 cur = edges2[0].second;
        edges2.erase(edges2.begin());
        
        std::vector<Vec2> chain{start, cur};
        
        // растём вперёд
        while (true) {
            auto opt = popNext(cur);
            if (!opt) break;
            Vec2 nxt = *opt;
            chain.push_back(nxt);
            cur = nxt;
        }
        
        chains.push_back(std::move(chain));
    }

    /* ❻ упрощаем каждую polyline Ramer–Douglas–Peucker */
    auto distPtSeg = [](const Vec2& p, const Vec2& a, const Vec2& b) {
        double vx = b.x - a.x, vy = b.y - a.y;
        double len2 = vx * vx + vy * vy; 
        if (len2 < 1e-12) return std::hypot(p.x - a.x, p.y - a.y);
        double t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2; 
        t = std::clamp(t, 0.0, 1.0);
        double px = a.x + t * vx, py = a.y + t * vy;
        return std::hypot(p.x - px, p.y - py);
    };
    
    std::function<void(const std::vector<Vec2>&, size_t, size_t, std::vector<bool>&)> rdp =
        [&](const std::vector<Vec2>& arr, size_t l, size_t r, std::vector<bool>& keep) {
            if (r <= l + 1) return;
            double maxd = 0; 
            size_t idx = 0;
            for (size_t i = l + 1; i < r; ++i) {
                double d = distPtSeg(arr[i], arr[l], arr[r]);
                if (d > maxd) { maxd = d; idx = i; }
            }
            if (maxd > tol) { 
                keep[idx] = true; 
                rdp(arr, l, idx, keep); 
                rdp(arr, idx, r, keep);
            }
        };

    std::vector<Seg> result;
    for (auto& poly : chains) {
        if (poly.size() < 2) continue;
        std::vector<bool> keep(poly.size(), false);
        keep[0] = keep.back() = true;
        rdp(poly, 0, poly.size() - 1, keep);

        Vec2 prev{};
        bool first = true;
        for (size_t i = 0; i < poly.size(); ++i) {
            if (!keep[i]) continue;
            if (first) { 
                prev = poly[i]; 
                first = false; 
                continue; 
            }
            result.push_back({{prev.x, prev.y}, {poly[i].x, poly[i].y}});
            prev = poly[i];
        }
    }
    segs.swap(result);
}

struct EdgeKey{int a,b;};
struct EdgeHash{size_t operator()(const EdgeKey&e)const noexcept{return(size_t)e.a<<32 ^ (size_t)e.b;}};
struct EdgeEq  {bool operator()(const EdgeKey&l,const EdgeKey&r)const noexcept{return l.a==r.a&&l.b==r.b;}};

// ───────────────────────── scene analysis ───────────────────────────────────
struct SceneStats {
    double diagPx;   // диагональ bbox после project()
    size_t edges;    // |edges| до отбраковки
};

struct AlgoParams {
    int samples;
    double depthEps;
    double snap;
    double tol;
};



static AlgoParams chooseParams(const SceneStats& s) {
    // 2.1 количество сэмплов вдоль ребра:
    //     хотим не более N≈150 000 лучей всего
    int samples = std::clamp<int>(350'000 / s.edges, 20, 400);

    // 2.2 глубинный ε = 0.0005 от диагонали в мировых координатах
    //     (предполагаем, что 1 юмп == 1 мир. является уже @proj)
    double depthEps = 0.0005 * s.diagPx;

    // 2.3 квантизация узлов snap = 0.1 % диагонали (уменьшено с 0.3%)
    double snap = 0.001 * s.diagPx;

    // 2.4 допуск RDP tol = 0.2 % диагонали (уменьшено с 0.8%)
    double tol = 0.002 * s.diagPx;

    return {samples, depthEps, snap, tol};
}

// ─────────────────────────── globals (camera) ───────────────────────────────
static Vec3 g_cam;         // eye position
static Vec3 g_U,g_V,g_D;   // camera basis: right, up, forward
static bool g_persp=false; static double g_focal_px=1.0;
static double g_shiftX = 0.0;
static double g_shiftY = 0.0;
static RTCDevice gDevice;
static RTCScene  gScene;

// ───────────────────────── projection ---------------------------------------
static Vec2 project(const Vec3& P,int W,int H)
{
    Vec3 pc = P - g_cam;                       // to camera space
    double x_cam = dot(pc,g_U);
    double y_cam = dot(pc,g_V);
    double z_cam = dot(pc,g_D);
    if(g_persp){                               // perspective
        if(z_cam<1e-6) z_cam=1e-6;
            double u =  g_focal_px * x_cam / z_cam + g_shiftX;
            double v = -g_focal_px * y_cam / z_cam + g_shiftY;
        return {u,v};
    }
    // orthographic: simple linear map into pixel space (no fit)
    return { x_cam + g_shiftX, -y_cam + g_shiftY };
}

// ───────────────────────── scene analysis ───────────────────────────────────
static SceneStats analyse(const std::vector<Vec3>& V,
                         const std::unordered_set<EdgeKey, EdgeHash, EdgeEq>& edges,
                         int W, int H)
{
    double mnx = 1e30, mny = 1e30, mxx = -1e30, mxy = -1e30;
    for (auto& v : V) {
        Vec2 p = project(v, W, H);
        mnx = std::min(mnx, p.x); mxx = std::max(mxx, p.x);
        mny = std::min(mny, p.y); mxy = std::max(mxy, p.y);
    }
    double diagPx = std::hypot(mxx - mnx, mxy - mny);
    return {diagPx, edges.size()};
}

// ─────────────────── Embree ray tracing (replaces manual ray/triangle intersection) ───────────────────
// ───────────────────────── SVG writer ---------------------------------------
static void writeSVG(const std::vector<Seg>& segs, const std::string& path, double margin = 10, const std::string& background = "none")
{
    if(segs.empty()) return; 
    double mnx=1e30, mny=1e30, mxx=-1e30, mxy=-1e30;
    for(auto& s : segs) {
        mnx = std::min({mnx, s.a.x, s.b.x});
        mxx = std::max({mxx, s.a.x, s.b.x});
        mny = std::min({mny, s.a.y, s.b.y});
        mxy = std::max({mxy, s.a.y, s.b.y});
    }
    double W = (mxx - mnx) + 2 * margin, H = (mxy - mny) + 2 * margin;
    std::ofstream o(path);
    o << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W << "' height='" << H
      << "' viewBox='0 0 " << W << ' ' << H << "' stroke='black' fill='none' stroke-width='2' stroke-linecap='round' vector-effect='non-scaling-stroke'>\n";
    if (background != "none" && background != "" && background != "transparent") {
        o << "<rect x='0' y='0' width='" << W << "' height='" << H << "' fill='" << background << "' stroke='none'/>\n";
    }
    
    // Используем path с M/L командами для избежания "гармошки"
    o << "<path d='";
    for (auto& s : segs) {
        double x1 = s.a.x - mnx + margin, y1 = s.a.y - mny + margin;
        double x2 = s.b.x - mnx + margin, y2 = s.b.y - mny + margin;
        o << "M " << x1 << ' ' << y1 << " L " << x2 << ' ' << y2 << ' ';
    }
    o << "' stroke='black' stroke-width='1' stroke-linecap='round' "
         "fill='none' vector-effect='non-scaling-stroke'/>\n";
    
    o << "</svg>\n"; 
    std::cout << "Saved SVG to " << path << "\n";
}

// ───────────────────────── simple OBJ loader --------------------------------
static void loadOBJ(const std::string& p,std::vector<Vec3>& V,std::vector<std::vector<int>>& F){
    std::ifstream in(p); if(!in) throw std::runtime_error("cannot open "+p);
    std::string line; while(std::getline(in,line)){
        std::istringstream ss(line); std::string t; ss>>t;
        if(t=="v"){double x,y,z; ss>>x>>y>>z; V.push_back({x,y,z});}
        else if(t=="f"){std::vector<int> poly; std::string v; while(ss>>v){size_t s=v.find('/');int idx=std::stoi(s==std::string::npos?v:v.substr(0,s)); if(idx<0) idx=int(V.size())+idx+1; poly.push_back(idx-1);} if(poly.size()>=3)F.push_back(poly);} }
}

// ───────────────────────── convert() implementation -------------------------

void OBJ2EdgesConverter_Optimized::convert(const std::string &inPath,
                                           const std::string &outPath,
                                           const Options      &opts)
{
    const int    W       = int(optF(opts, "w", 800));
    const int    H       = int(optF(opts, "h", 800));
    const double fovDeg  = optF(opts, "fov", 0);
    const double margin  = optF(opts, "margin", 10);

    // ───────────────── load OBJ ────────────────────────────────────────────
    std::vector<Vec3> V;  std::vector<std::vector<int>> F;  loadOBJ(inPath, V, F);

    // ───────────────── camera setup ───────────────────────────────────────
    Vec3 centre{};
    for (auto &v : V) centre = centre + v;
    centre = centre * (1.0 / V.size());

    if (optE(opts, "pos")) {
        auto a  = parseVec3(optS(opts, "pos", "(0,0,0)"));
        g_cam   = {a[0], a[1], a[2]};
    } else {
        g_cam = centre - Vec3{10, -10, len(centre) * 3};
    }

    g_D = norm(centre - g_cam);
    const Vec3 worldUp  = (std::fabs(g_D.y) < 0.99) ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
    g_U = norm(cross(worldUp, g_D));
    g_V = cross(g_D, g_U);

    // ───────────────── projection params (same as before) ────────────────
    g_persp = fovDeg > 0;
    if (g_persp) {
        double minX = 1e30, maxX = -1e30, minY = 1e30, maxY = -1e30;
        for (const auto &v : V) {
            Vec3 pc = v - g_cam;
            double z = dot(pc, g_D);  if (z < 1e-6) z = 1e-6;
            double x =  g_focal_px * dot(pc, g_U) / z;
            double y = -g_focal_px * dot(pc, g_V) / z;
            minX = std::min(minX, x);  maxX = std::max(maxX, x);
            minY = std::min(minY, y);  maxY = std::max(maxY, y);
        }
        double scale = std::min(double(W) / (maxX - minX), double(H) / (maxY - minY));
        g_focal_px *= scale;  minX *= scale; maxX *= scale;  minY *= scale; maxY *= scale;
        g_shiftX = (W - (minX + maxX)) * 0.5;  g_shiftY = (H - (minY + maxY)) * 0.5;
    } else {
        g_shiftX = W * 0.5;  g_shiftY = H * 0.5;
    }

    // ───────────────── triangulate faces ──────────────────────────────────
    std::vector<std::array<int, 3>> tris;
    for (auto &p : F) {
        if (p.size() == 3) {
            tris.push_back({p[0], p[1], p[2]});
        } else {
            for (size_t i = 1; i + 1 < p.size(); ++i)
                tris.push_back({p[0], p[i], p[i + 1]});
        }
    }

    const std::size_t N  = tris.size();

    // ---- Build Embree scene once --------------------------------------------
    gDevice = rtcNewDevice(nullptr);                       // ➊
    gScene  = rtcNewScene(gDevice);

    RTCGeometry geom = rtcNewGeometry(gDevice, RTC_GEOMETRY_TYPE_TRIANGLE);

    // vertex buffer
    struct V3f { float x,y,z; };
    auto* vb = (V3f*) rtcSetNewGeometryBuffer(
                  geom, RTC_BUFFER_TYPE_VERTEX, 0,
                  RTC_FORMAT_FLOAT3, sizeof(V3f), V.size());
    for (size_t i=0;i<V.size();++i) vb[i] = {float(V[i].x),float(V[i].y),float(V[i].z)};  // ➋

    // index buffer
    struct U3  { unsigned i0,i1,i2; };
    auto* ib = (U3*) rtcSetNewGeometryBuffer(
                  geom, RTC_BUFFER_TYPE_INDEX, 0,
                  RTC_FORMAT_UINT3, sizeof(U3), tris.size());
    for (size_t i=0;i<tris.size();++i) ib[i] = {unsigned(tris[i][0]),unsigned(tris[i][1]),unsigned(tris[i][2])};

    rtcCommitGeometry(geom);
    rtcAttachGeometry(gScene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(gScene);                                // BVH build SAH

    // ───────────────── unique edges ───────────────────────────────────────
    std::unordered_set<EdgeKey, EdgeHash, EdgeEq> edges;
    auto addEdge = [&](int a, int b) {
        if (a > b) std::swap(a, b);
        edges.insert({a, b});
    };
    for (auto &p : F)
        for (size_t i = 0; i < p.size(); ++i)
            addEdge(p[i], p[(i + 1) % p.size()]);

    // ───────────────── scene analysis & auto-params ───────────────────────
    SceneStats stats = analyse(V, edges, W, H);
    AlgoParams p = chooseParams(stats);

    // если пользователь передал свою опцию, оставляем
    if (optE(opts, "samples"))        p.samples   = int(optF(opts, "samples", p.samples));
    if (optE(opts, "depth-epsilon"))  p.depthEps  =      optF(opts, "depth-epsilon", p.depthEps);

    // hard-limits – защищаем от «безумных» чисел
    if (p.samples < 5 || p.samples > 1000)
        throw std::runtime_error("samples out of [5,1000]");
    if (p.depthEps < 1e-6 * stats.diagPx || p.depthEps > 0.05 * stats.diagPx)
        throw std::runtime_error("depth-epsilon out of reasonable range");

    std::cout << "[auto-params] samples=" << p.samples
              << "  depthEps=" << p.depthEps
              << "  snap=" << p.snap
              << "  tol=" << p.tol << '\n';

    // ───────────────── sampling ───────────────────────────────────────────
    std::vector<Seg> visible;
    const std::size_t total = edges.size();
    std::size_t done = 0, lastPct = 0;
    std::cout << "Check " << total << " edges...\n";

    auto rayHitsAnything = [&](const Vec3& o, const Vec3& d, double maxDist)->bool
    {
        RTCRay ray{};
        ray.org_x = float(o.x);  ray.org_y = float(o.y);  ray.org_z = float(o.z);
        ray.dir_x = float(d.x);  ray.dir_y = float(d.y);  ray.dir_z = float(d.z);
        ray.tnear = float(p.depthEps);            // используем автоматический depthEps
        ray.tfar  = float(maxDist);
        ray.mask  = 0xFFFFFFFF;
        ray.time  = 0.f;

        RTCOccludedArguments args;
        rtcInitOccludedArguments(&args);
        args.context = nullptr;
        rtcOccluded1(gScene, &ray, &args);
        return ray.tfar < 0.0f;                  // tfar < 0  ⇒  что-то перекрыло луч
    };

    for (auto &e : edges) {
        ++done;
        std::size_t pct = done * 100 / total;
        if (pct >= lastPct + 10 || done == total) { std::cout << pct << "%\r"; lastPct = pct; }

        Vec3 A = V[e.a], B = V[e.b];
        const int S = p.samples;                     // переименовал для читаемости
        std::vector<char> mask(S+1, 0);              // 0/1 видимость вдоль ребра

        // 1.1 Строим маску видимости
        for (int s = 0; s <= S; ++s) {
            double t = double(s) / S;
            Vec3 P = A + (B - A) * t;
            Vec3 dir = P - g_cam;
            double dist = len(dir);
            dir = dir * (1.0 / dist);
            
            mask[s] = !rayHitsAnything(g_cam, dir, dist);
        }

        // 1.2 Сканируем маску с допуском на шум - собираем видимые интервалы
        const int noiseTolerance = std::max(4, S / 25);   // увеличили допуск: ~4% от samples
        const int minIntervalLength = std::max(1, S / 50); // уменьшили минимальную длину: ~2% от samples
        
        for (int s = 0; s <= S; ) {
            // Ищем начало видимого интервала
            while (s <= S && mask[s] == 0) ++s;
            if (s > S) break;
            int s0 = s;
            
            // Ищем конец видимого интервала с допуском на шум
            int consecutiveHidden = 0;
            while (s <= S && consecutiveHidden <= noiseTolerance) {
                if (mask[s] == 1) {
                    consecutiveHidden = 0;  // сбрасываем счётчик
                } else {
                    consecutiveHidden++;
                }
                s++;
            }
            int s1 = s - consecutiveHidden - 1;  // откатываемся к последнему видимому
            
            // Проверяем, что интервал достаточно длинный
            if (s1 >= s0 && (s1 - s0 + 1) >= minIntervalLength) {
                Vec3 P0 = A + (B - A) * (double(s0) / S);
                Vec3 P1 = A + (B - A) * (double(s1) / S);
                Vec2 proj0 = project(P0, W, H);
                Vec2 proj1 = project(P1, W, H);

                // отфильтровать микроскопические (ещё уменьшили порог)
                if (std::hypot(proj1.x - proj0.x, proj1.y - proj0.y) >= 0.3) {
                    visible.push_back({proj0, proj1});
                }
            }
        }
    }

    std::cout << "\nvisible segments=" << visible.size() << "\n";

    const std::filesystem::path out = outPath.empty() ?
        std::filesystem::path(inPath).replace_extension(".svg") : std::filesystem::path(outPath);

    // joinCollinearChains больше не нужен - у нас уже правильные отрезки
    std::cout << "final segments=" << visible.size() << "\n";
    writeSVG(visible, out.string(), margin, "#fdf6e3");

    // Cleanup Embree resources
    rtcReleaseScene(gScene);
    rtcReleaseDevice(gDevice);
}
}