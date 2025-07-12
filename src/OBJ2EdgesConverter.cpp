// visible_edges_simple.cpp  —  perspective / ortho edges that match OBJ2NMap camera
// -----------------------------------------------------------------------------
// Build:  clang++ -std=c++17 -O2 visible_edges_simple.cpp -o visible_edges_simple
// Example: ./visible_edges_simple part.obj out.svg --width 1024 --height 768 \
//                    --pos "(10,10,10)" --fov 45 --samples 150
// Same <pos,fov> pair fed into OBJ2NMap gives perfectly aligned normal-map.
// -----------------------------------------------------------------------------

#include "converter_lib/OBJ2EdgesConverter.hpp"
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

struct EdgeKey{int a,b;};
struct EdgeHash{size_t operator()(const EdgeKey&e)const noexcept{return(size_t)e.a<<32 ^ (size_t)e.b;}};
struct EdgeEq  {bool operator()(const EdgeKey&l,const EdgeKey&r)const noexcept{return l.a==r.a&&l.b==r.b;}};

// ─────────────────────────── globals (camera) ───────────────────────────────
static Vec3 g_cam;         // eye position
static Vec3 g_U,g_V,g_D;   // camera basis: right, up, forward
static bool g_persp=false; static double g_focal_px=1.0;
static double g_shiftX = 0.0;
static double g_shiftY = 0.0;
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

// ─────────────────── ray/triangle intersection ------------------------------
static bool rayTri(const Vec3& o,const Vec3& d,
                   const Vec3& v0,const Vec3& v1,const Vec3& v2,
                   double maxDist,double EPS)
{
    Vec3 e1=v1-v0,e2=v2-v0,p=cross(d,e2);
    double det=dot(e1,p); if(std::fabs(det)<EPS) return false;
    double inv=1.0/det; Vec3 t=o-v0;
    double u=dot(t,p)*inv; if(u<0||u>1) return false;
    Vec3 q=cross(t,e1);    double v=dot(d,q)*inv; if(v<0||u+v>1) return false;
    double hit=dot(e2,q)*inv; return hit>EPS && hit<maxDist-EPS;
}

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
      << "' viewBox='0 0 " << W << ' ' << H << "' stroke='black' fill='none' stroke-width='1'>\n";
    if (background != "none" && background != "" && background != "transparent") {
        o << "<rect x='0' y='0' width='" << W << "' height='" << H << "' fill='" << background << "' stroke='none'/>\n";
    }
    for(auto& s : segs) 
        o << "<line x1='" << s.a.x - mnx + margin << "' y1='" << s.a.y - mny + margin
          << "' x2='" << s.b.x - mnx + margin << "' y2='" << s.b.y - mny + margin << "'/>\n";
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
void OBJ2EdgesConverter::convert(const std::string& inPath,const std::string& outPath,const Options& opts)
{
    int W = (int)optF(opts,"w",800), H=(int)optF(opts,"h",800);
    double fov = optF(opts,"fov",0);            // 0 → ortho
    int samples=(int)optF(opts,"samples",120);  double EPS=optF(opts,"eps",1e-3);
    double margin=optF(opts,"margin",10);

    // --- OBJ ---
    std::vector<Vec3> V; std::vector<std::vector<int>> F; loadOBJ(inPath,V,F);

    // --- camera dir/pos -----------------------------------------------------
    Vec3 centre{}; for(auto&v:V) centre=centre+v; centre=centre*(1.0/V.size());
    // pos = --pos "(x,y,z)" or default far corner
    if(optE(opts,"pos")){
        auto a=parseVec3(optS(opts,"pos","(0,0,0)")); g_cam={a[0],a[1],a[2]};
    }else g_cam= centre - Vec3{10,-10, len(centre)*3};

    g_D = norm(centre - g_cam);
    Vec3 worldUp = std::fabs(g_D.y)<0.99? Vec3{0,1,0}:Vec3{1,0,0};
    g_U = norm(cross(worldUp,g_D));
    g_V = cross(g_D,g_U);

    // --- projection setup ---------------------------------------------------
    g_persp = fov>0; // --- perspective auto-fit --------------------------------------------------
    if (g_persp)
    {
        double minX =  1e30, maxX = -1e30;
        double minY =  1e30, maxY = -1e30;

        // bbox в экранных координатах (без сдвига)
        for (const auto& v : V)
        {
            Vec3 pc = v - g_cam;
            double z = dot(pc, g_D); if (z < 1e-6) z = 1e-6;

            double x =  g_focal_px * dot(pc, g_U) / z;
            double y = -g_focal_px * dot(pc, g_V) / z;

            minX = std::min(minX, x);  maxX = std::max(maxX, x);
            minY = std::min(minY, y);  maxY = std::max(maxY, y);
        }

        double scale = std::min( static_cast<double>(W) / (maxX - minX),
                                static_cast<double>(H) / (maxY - minY) );

        g_focal_px *= scale;           // новый focal
        minX *= scale;  maxX *= scale; // те же *scale
        minY *= scale;  maxY *= scale;

        g_shiftX = (W - (minX + maxX)) * 0.5;
        g_shiftY = (H - (minY + maxY)) * 0.5;
    }
    else {            // орто: смещения = центр кадра
        g_shiftX = W * 0.5;
        g_shiftY = H * 0.5;
    }
    // --- build triangle list ------------------------------------------------
    std::vector<std::array<int,3>> tris;
    for(auto&p:F) if(p.size()==3) tris.push_back({p[0],p[1],p[2]});
        else for(size_t i=1;i+1<p.size();++i) tris.push_back({p[0],p[i],p[i+1]});

    // --- gather unique edges -----------------------------------------------
    std::unordered_set<EdgeKey,EdgeHash,EdgeEq> edges; auto add=[&](int a,int b){if(a>b)std::swap(a,b);edges.insert({a,b});};
    for(auto&p:F) for(size_t i=0;i<p.size();++i) add(p[i],p[(i+1)%p.size()]);

    // --- visibility sampling ------------------------------------------------
    std::vector<Seg> visible; size_t total=edges.size(),done=0,last=0;
    std::cout<<"Check "<<total<<" edges...\n";
    for(auto&e:edges){ ++done; size_t pct=done*100/total; if(pct>=last+10||done==total){std::cout<<pct<<"%\r";last=pct;}
        Vec3 A=V[e.a],B=V[e.b]; bool open=false; Vec2 start;
        for(int s=0;s<=samples;++s){ double t=(double)s/samples; Vec3 P=A+(B-A)*t; Vec3 dir=P-g_cam; double dist=len(dir); dir=dir*(1.0/dist);
            bool occ=false; for(auto&t_:tris) if(rayTri(g_cam,dir,V[t_[0]],V[t_[1]],V[t_[2]],dist,EPS)){occ=true;break;}
            if(!occ&&!open){open=true; start=project(P,W,H);} if((occ||s==samples)&&open){ Vec2 end=project(P,W,H); visible.push_back({start,end}); open=false; }
        }
    }
    std::cout<<"\nvisible segments="<<visible.size()<<"\n";

    std::filesystem::path out = outPath.empty() ? std::filesystem::path(inPath).replace_extension(".svg")
                                                : std::filesystem::path(outPath);
    writeSVG(visible, out.string(), margin, "#fdf6e3");
}

} // namespace converter_lib