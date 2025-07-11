// visible_edges_simple.cpp  (re-built: perspective camera + SVG + preprocess_obj)
// ---------------------------------------------------------------------------
// Build:  clang++ -std=c++17 -O2 visible_edges_simple.cpp -o visible_edges_simple
// Usage:  ./visible_edges_simple model.obj  →  writes visible_edges.svg
// ---------------------------------------------------------------------------

#include "converter_lib/VisibleEdgesConverter.hpp"
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

namespace converter_lib {

// ---------- helper to fetch option or default --------------------------------
static float optF(const Options& o, const std::string& key, float def) {
    auto it = o.params.find(key); return it==o.params.end()? def : std::stof(it->second);
}
static std::string optS(const Options& o,const std::string&key,const std::string&def){
    auto it=o.params.find(key); return it==o.params.end()?def:it->second;
}

struct Vec3 { double x{}, y{}, z{}; };

inline Vec3 operator+(Vec3 a, Vec3 b){ return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b){ return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec3 operator*(Vec3 v, double s){ return {v.x*s,v.y*s,v.z*s}; }
inline double dot(Vec3 a, Vec3 b){ return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(Vec3 a, Vec3 b){
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
inline double len(Vec3 v){ return std::sqrt(dot(v,v)); }
inline Vec3 normalize(Vec3 v){ double l=len(v); return (l>0)?v*(1.0/l):v; }

struct Mesh{
    std::vector<Vec3>               V;          // unique verts
    std::vector<std::vector<int>>   F;          // polygons (CCW)
};

// ------------------------- 2-D & misc types ------------------------
struct Vec2 { double x{}, y{}; };
struct Seg  { Vec2 a, b; };

struct EdgeKey { int a,b; };             // a<b
struct EdgeHash{ size_t operator()(const EdgeKey&e)const noexcept{ return (size_t)e.a<<32 ^ (size_t)e.b; }};
struct EdgeEq { bool operator()(const EdgeKey&l,const EdgeKey&r)const noexcept{ return l.a==r.a && l.b==r.b; }};

static Vec3 g_cam;                 // global camera position

// --------------------- perspective projection ---------------------
static Vec2 project(const Vec3& p, int WIDTH, int HEIGHT, double FOCAL)
{
    double z = p.z - g_cam.z;
    if(z < 1e-6) z = 1e-6;
    double x =  (p.x - g_cam.x) * FOCAL / z + WIDTH  * 0.5;
    double y = -(p.y - g_cam.y) * FOCAL / z + HEIGHT * 0.5;  // flip Y
    return {x,y};
}

// ------------------ ray–triangle intersection ---------------------
static bool rayTri(const Vec3& orig,const Vec3& dir,
                   const Vec3& v0,const Vec3& v1,const Vec3& v2,
                   double distMax, double EPS)
{
    Vec3 e1=v1-v0, e2=v2-v0;
    Vec3 p=cross(dir,e2);
    double det = dot(e1,p);
    if(std::fabs(det) < EPS) return false;
    double inv=1.0/det;
    Vec3 t = orig - v0;
    double u=dot(t,p)*inv; if(u<0||u>1) return false;
    Vec3 q=cross(t,e1);
    double v=dot(dir,q)*inv; if(v<0||u+v>1) return false;
    double hit=dot(e2,q)*inv;
    return hit>EPS && hit<distMax-EPS;
}

// ------------------------ SVG writer -------------------------------
static void writeSVG(const std::vector<Seg>& segs,
                     const std::string& path="visible_edges.svg",
                     double margin=10.0)
{
    if(segs.empty()) return;
    double minx=1e30,maxx=-1e30,miny=1e30,maxy=-1e30;
    for(auto&s:segs){
        minx=std::min({minx,s.a.x,s.b.x}); maxx=std::max({maxx,s.a.x,s.b.x});
        miny=std::min({miny,s.a.y,s.b.y}); maxy=std::max({maxy,s.a.y,s.b.y});
    }
    double w=(maxx-minx)+2*margin, h=(maxy-miny)+2*margin;
    std::ofstream svg(path);
    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\""<<w
        <<"\" height=\""<<h<<"\" viewBox=\"0 0 "<<w<<' '<<h
        <<"\" stroke=\"black\" stroke-width=\"1\" fill=\"none\">\n";
    for(auto&s:segs)
        svg << "  <line x1=\""<<s.a.x-minx+margin<<"\" y1=\""<<s.a.y-miny+margin
            <<"\" x2=\""<<s.b.x-minx+margin<<"\" y2=\""<<s.b.y-miny+margin<<"\"/>\n";
    svg << "</svg>\n";
    std::cout << "Saved SVG to "<<path<<"\n";
}

// ---------------------------- Load OBJ --------------------------------
static void LoadObj(const std::string& path, std::vector<Vec3>& V, std::vector<std::vector<int>>& F) {
    std::ifstream in(path);
    if (!in) { 
        throw std::runtime_error("Failed to open OBJ: " + path);
    }
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;
        if (type == "v") {
            double x, y, z;
            iss >> x >> y >> z;
            V.push_back({x, y, z});
        } else if (type == "f") {
            std::vector<int> poly;
            std::string vert;
            while (iss >> vert) {
                size_t slash = vert.find('/');
                int idx = std::stoi(slash == std::string::npos ? vert : vert.substr(0, slash));
                if (idx < 0) idx = int(V.size()) + idx + 1;
                poly.push_back(idx - 1); // OBJ is 1-based
            }
            if (poly.size() >= 3)
                F.push_back(poly);
        }
    }
}

// -----------------------------------------------------------------------------
/// core
void VisibleEdgesConverter::convert(const std::string& inPath,
                                    const std::string& outPath,
                                    const Options& opts)
{
    // ---------- settings -----------------------------------------------------
    int    WIDTH   = static_cast<int>(optF(opts, "width", 800));
    int    HEIGHT  = static_cast<int>(optF(opts, "height", 600));
    double FOCAL   = optF(opts, "focal", 1000.0);   // perspective focal length in px
    int    SAMPLES = static_cast<int>(optF(opts, "samples", 100));    // samples per edge
    double EPS     = optF(opts, "eps", 1e-2);
    double margin  = optF(opts, "margin", 10.0);

    // ---------- load OBJ -----------------------------------------------------
    std::vector<Vec3> V;
    std::vector<std::vector<int>> F;
    LoadObj(inPath, V, F);

    std::cout << "V="<<V.size()<<"  Polys="<<F.size()<<"\n";

    // 1.1 Build triangle list for intersection once
    std::vector<std::array<int,3>> tris;
    for(const auto& poly:F){
        if(poly.size()==3) tris.push_back({poly[0],poly[1],poly[2]});
        else {
            for(size_t i=1;i+1<poly.size();++i)
                tris.push_back({poly[0], poly[i], poly[i+1]});
        }
    }

    // 2. camera position (centre –Z)
    Vec3 cent{}; for(auto&v:V) cent = cent + v; cent = cent*(1.0/V.size());
    double diag=0; for(auto&v:V) diag = std::max(diag, len(v-cent));
    g_cam = cent - Vec3{10,-10,2*diag};

    // 3. unique edge set (no internal diagonals because from polygons)
    std::unordered_set<EdgeKey,EdgeHash,EdgeEq> edges;
    auto add=[&](int a,int b){ if(a>b) std::swap(a,b); edges.insert({a,b}); };
    for(const auto& poly:F)
        for(size_t i=0;i<poly.size();++i)
            add(poly[i], poly[(i+1)%poly.size()]);
    

    // 4. sample-and-test visibility
    std::vector<Seg> visSegs;
    size_t totalEdges = edges.size();
    size_t edgeCount = 0;
    size_t lastPercent = 0;
    std::cout << "Testing visibility for " << totalEdges << " edges...\n";
    for(const auto& e:edges){
        ++edgeCount;
        // Print progress every 5% or at the end
        size_t percent = (edgeCount * 100) / totalEdges;
        if (percent >= lastPercent + 5 || edgeCount == totalEdges) {
            std::cout << "\rProgress: " << percent << "% (" << edgeCount << "/" << totalEdges << ")" << std::flush;
            lastPercent = percent;
        }

        Vec3 A=V[e.a], B=V[e.b];
        bool inSeg=false; Vec2 segStart{};
        for(int s=0;s<=SAMPLES;++s){
            double t=(double)s/SAMPLES;
            Vec3 P = A + (B-A)*t;
            Vec3 dir = P - g_cam; double dist=len(dir); dir=dir*(1.0/dist);
            bool occluded=false;
            for(const auto& tri:tris){
                if(rayTri(g_cam,dir,V[tri[0]],V[tri[1]],V[tri[2]],dist, EPS))
                { occluded=true; break; }
            }
            if(!occluded && !inSeg){ inSeg=true; segStart=project(P, WIDTH, HEIGHT, FOCAL); }
            if((occluded||s==SAMPLES) && inSeg){
                Vec2 segEnd=project(P, WIDTH, HEIGHT, FOCAL); visSegs.push_back({segStart,segEnd}); inSeg=false;
            }
        }
    }
    std::cout << "\nVisibility test complete.\n";

    // 5. output & svg
    std::cout << "visible segments: "<<visSegs.size()<<"\n";
    for(auto&s:visSegs) std::cout << '('<<s.a.x<<','<<s.a.y<<") -> ("<<s.b.x<<','<<s.b.y<<")\n";

    // ---------- store output -------------------------------------------------
    std::filesystem::path out = outPath.empty()
        ? std::filesystem::path(inPath).replace_extension(".svg")
        : std::filesystem::path(outPath);
    writeSVG(visSegs, out.string(), margin);
}

} // namespace converter_lib
