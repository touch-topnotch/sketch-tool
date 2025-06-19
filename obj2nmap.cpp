/*  obj2nmap.cpp  (single-file, header-only dependencies)
    -----------------------------------------------------------------------------
    Generates a tangent-space normal map from a Wavefront **.obj** model.
      • Accepts a *camera position* (x, y, z). The camera is always rotated to
        look at the model's **centre of mass**, computed on load.
      • Uniform (isotropic) scaling: preserves aspect ratio so geometry is never
        stretched in U vs. V. The model is letter-boxed inside the requested
        resolution, centred with optional world-space offsets.
      • Optional in-plane rotation **--rot w** (degrees, applied around view
        vector after look-at is established).
    -----------------------------------------------------------------------------
    Usage
      --obj    <file>             OBJ path (required)
      --cam    <x> <y> <z>        Camera world-space position (required)
      --rot    <w>                In-plane rotation around view vector (deg)
      --size   <width> <height>   Output resolution (default 1024×1024)
      --offset <u> <v>            Translate projection plane in world units
      --index  <n>                Suffix integer for output file name

    Legacy option --dir <x> <y> <z> [w] is still accepted for orthographic
    projections that do *not* depend on camera position.
    -----------------------------------------------------------------------------
    Build (Mac/Linux):
      g++ -std=c++17 -O2 obj2nmap.cpp -o obj2nmap
    -----------------------------------------------------------------------------
    Author: Dmitry Tetkin (2025) – modified June 2025 for look-at camera input.
    ----------------------------------------------------------------------------- */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <limits>
#include <filesystem>
#include <iostream>

// -------------------------- External single-header libs ----------------------
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ------------------------------ Math helpers ---------------------------------
struct Vec3 { float x, y, z; };
inline Vec3 operator+(const Vec3&a,const Vec3&b){return{a.x+b.x,a.y+b.y,a.z+b.z};}
inline Vec3 operator-(const Vec3&a,const Vec3&b){return{a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec3 operator*(const Vec3&a,float s){return{a.x*s,a.y*s,a.z*s};}
inline float dot(const Vec3&a,const Vec3&b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 cross(const Vec3&a,const Vec3&b){return{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline Vec3 normalize(const Vec3&v){float l=std::sqrt(dot(v,v));return l? v*(1.f/l):Vec3{0,0,0};}

// ---------------------------------- types ------------------------------------
struct Pixel{float depth;Vec3 normal;bool filled;Pixel():depth(std::numeric_limits<float>::max()),normal{0,0,0},filled(false){}};

struct Settings{
    std::string objPath;
    Vec3 cam{0,0,0}; bool useCam=false;   // new
    Vec3 dir{0,0,1};
    float rot=0;
    int W=1024, H=1024;
    float offU=0, offV=0;
    int idx=0;
};

// ------------------------------ CLI parsing ----------------------------------
static bool isNumber(const char*s){char*c;std::strtod(s,&c);return *c==0;}
static void parseArgs(int argc,char**argv,Settings&s){
    for(int i=1;i<argc;++i){std::string a=argv[i];
        if(a=="--obj" && i+1<argc){s.objPath=argv[++i];}
        else if(a=="--cam" && i+3<argc){
            s.cam={std::stof(argv[++i]),std::stof(argv[++i]),std::stof(argv[++i])};
            s.useCam=true;
        }
        else if(a=="--dir" && i+3<argc){
            s.dir={std::stof(argv[++i]),std::stof(argv[++i]),std::stof(argv[++i])};
            if(i+1<argc && isNumber(argv[i+1])) s.rot=std::stof(argv[++i]);
        }
        else if(a=="--rot" && i+1<argc){ s.rot=std::stof(argv[++i]); }
        else if(a=="--size" && i+2<argc){s.W=std::stoi(argv[++i]); s.H=std::stoi(argv[++i]);}
        else if(a=="--offset" && i+2<argc){s.offU=std::stof(argv[++i]); s.offV=std::stof(argv[++i]);}
        else if(a=="--index" && i+1<argc){s.idx=std::stoi(argv[++i]);}
        else {std::cerr << "Unknown/invalid option: " << a << "\n"; std::exit(EXIT_FAILURE);} }
    if(s.objPath.empty()){std::cerr << "--obj parameter required\n"; std::exit(EXIT_FAILURE);} 
    if(!s.useCam && s.dir.x==0 && s.dir.y==0 && s.dir.z==0){
        std::cerr << "Either --cam or non-zero --dir must be provided\n";
        std::exit(EXIT_FAILURE);
    }
}

// ----------------------------------------------------------------------------
int main(int argc,char**argv){
    Settings cfg; parseArgs(argc,argv,cfg);

    // Load OBJ
    tinyobj::ObjReaderConfig rc; rc.triangulate = true; tinyobj::ObjReader reader;
    if(!reader.ParseFromFile(cfg.objPath, rc)){
        std::cerr << "OBJ load failed: " << reader.Error() << "\n"; return EXIT_FAILURE; }
    const auto &at = reader.GetAttrib(); const auto &sh = reader.GetShapes();

    // ---------------------------------------------------------------------
    // Calculate centre of mass (simple vertex average)
    // ---------------------------------------------------------------------
    Vec3 centre{0,0,0};
    size_t vCnt = at.vertices.size()/3;
    for(size_t i=0;i<vCnt;++i){ centre = centre + Vec3{at.vertices[3*i], at.vertices[3*i+1], at.vertices[3*i+2]}; }
    if(vCnt) centre = centre * (1.f/static_cast<float>(vCnt));

    // If camera position provided, build look-at direction
    if(cfg.useCam){
        Vec3 view = centre - cfg.cam;
        float len2 = dot(view,view);
        if(len2 < 1e-12f){ std::cerr << "Camera is at model centre – undefined view direction\n"; return EXIT_FAILURE; }
        cfg.dir = normalize(view);
    }

    // Build basis (U, V, D) – D is already normalised
    Vec3 D = normalize(cfg.dir);
    Vec3 up = std::fabs(D.y) < 0.99f ? Vec3{0,1,0} : Vec3{1,0,0};
    Vec3 U0 = normalize(cross(up, D));
    Vec3 V0 = cross(D, U0);
    float cR = std::cos(cfg.rot*M_PI/180.f), sR = std::sin(cfg.rot*M_PI/180.f);
    Vec3 U = U0 * cR - V0 * sR;
    Vec3 V = U0 * sR + V0 * cR;

    // Compute bounds in (U,V) space
    float minU =  std::numeric_limits<float>::max(), maxU = -std::numeric_limits<float>::max();
    float minV =  std::numeric_limits<float>::max(), maxV = -std::numeric_limits<float>::max();
    for(size_t i = 0; i < vCnt; ++i){
        Vec3 p{at.vertices[3*i], at.vertices[3*i+1], at.vertices[3*i+2]};
        float u = dot(p,U), v = dot(p,V);
        minU = std::min(minU,u); maxU = std::max(maxU,u);
        minV = std::min(minV,v); maxV = std::max(maxV,v); }

    // Apply user world-unit offsets BEFORE scaling
    minU += cfg.offU; maxU += cfg.offU;
    minV += cfg.offV; maxV += cfg.offV;

    // Uniform scale that fits the larger extent, preserving aspect ratio
    float scaleU = (maxU - minU) ? (cfg.W - 1) / (maxU - minU) : 1.f;
    float scaleV = (maxV - minV) ? (cfg.H - 1) / (maxV - minV) : 1.f;
    float scale  = std::min(scaleU, scaleV);         // isotropic scaling

    // Pixel-space padding to center the projection (letterbox/pillarbox)
    float padX = 0.5f * ((cfg.W - 1) - (maxU - minU) * scale);
    float padY = 0.5f * ((cfg.H - 1) - (maxV - minV) * scale);

    // Rasterisation framebuffer
    std::vector<Pixel> fb(cfg.W * cfg.H);
    auto addTri = [&](const Vec3&p0,const Vec3&p1,const Vec3&p2){
        auto proj = [&](const Vec3&p){
            float u = (dot(p,U) + cfg.offU - minU) * scale + padX;
            float v = (dot(p,V) + cfg.offV - minV) * scale + padY;
            float w = dot(p,D);
            return std::tuple<float,float,float>(u,v,w);
        };
        float u0,v0,w0; std::tie(u0,v0,w0) = proj(p0);
        float u1,v1,w1; std::tie(u1,v1,w1) = proj(p1);
        float u2,v2,w2; std::tie(u2,v2,w2) = proj(p2);

        int minX=(int)std::floor(std::min({u0,u1,u2})), maxX=(int)std::ceil(std::max({u0,u1,u2}));
        int minY=(int)std::floor(std::min({v0,v1,v2})), maxY=(int)std::ceil(std::max({v0,v1,v2}));
        minX = std::max(minX,0); maxX = std::min(maxX,cfg.W-1);
        minY = std::max(minY,0); maxY = std::min(maxY,cfg.H-1);
        float area = (u1-u0)*(v2-v0) - (v1-v0)*(u2-u0);
        if(std::fabs(area) < 1e-8f) return; float invA = 1.f / area;
        Vec3 faceN = normalize(cross(p1-p0, p2-p0));
        for(int y=minY; y<=maxY; ++y) for(int x=minX; x<=maxX; ++x){
            float w0e=(u1-u0)*(y-v0)-(v1-v0)*(x-u0);
            float w1e=(u2-u1)*(y-v1)-(v2-v1)*(x-u1);
            float w2e=(u0-u2)*(y-v2)-(v0-v2)*(x-u2);
            if((w0e>=0&&w1e>=0&&w2e>=0)||(w0e<=0&&w1e<=0&&w2e<=0)){
                float a=w1e*invA, b=w2e*invA, cλ=1.f-a-b;
                float depth = a*w0 + b*w1 + cλ*w2;
                Pixel &px = fb[y*cfg.W + x];
                if(depth < px.depth){ px.depth = depth; px.normal = faceN; px.filled = true; } } }
    };

    // Rasterise every triangle
    for(const auto &shape : sh){ size_t off = 0;
        for(size_t f=0; f<shape.mesh.num_face_vertices.size(); ++f){ Vec3 v[3];
            for(int k=0; k<3; ++k){ int id = shape.mesh.indices[off+k].vertex_index;
                v[k] = { at.vertices[3*id], at.vertices[3*id+1], at.vertices[3*id+2] }; }
            off += 3; addTri(v[0],v[1],v[2]); } }

    // Encode into RGB normal map (tangent-space)
    std::vector<unsigned char> img(cfg.W * cfg.H * 3);
    for(int y=0; y<cfg.H; ++y) for(int x=0; x<cfg.W; ++x){ const Pixel&p = fb[y*cfg.W + x]; size_t idx=3*(y*cfg.W+x);
        if(p.filled){ img[idx]  =(unsigned char)((p.normal.x*0.5f+0.5f)*255);
                       img[idx+1]=(unsigned char)((p.normal.y*0.5f+0.5f)*255);
                       img[idx+2]=(unsigned char)((p.normal.z*0.5f+0.5f)*255); }
        else { 
            img[idx] = 0; img[idx+1] = 0; img[idx+2] = 0;
        }
    }

    std::string out = cfg.objPath + "_normal_map_" + std::to_string(cfg.idx) + ".png";
    if(!stbi_write_png(out.c_str(), cfg.W, cfg.H, 3, img.data(), cfg.W*3)){
        std::cerr << "Failed to write image\n"; return EXIT_FAILURE; }
    std::cout << "Normal map saved to " << out << "\n";
    return EXIT_SUCCESS;
}