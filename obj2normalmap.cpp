// obj2normalmap.cpp  (single‑file, header‑only dependencies)
// -----------------------------------------------------------------------------
// Generates a tangent‑space normal map from a Wavefront **.obj** model.
// (FBX support removed for this minimal version.)
// -----------------------------------------------------------------------------
// Usage flags
//   --obj    <file>             OBJ path (required)
//   --dir    <x> <y> <z> [w]    Projection direction **D** and optional in‑plane
//                               rotation **w** in radians (counter‑clockwise).
//   --size   <width> <height>   Output resolution (default 1024 × 1024)
//   --offset <u> <v>            Translate projection plane (world units)
//   --index  <n>                Suffix integer for output file name
// -----------------------------------------------------------------------------
// Dependencies (header‑only):
//   tiny_obj_loader.h       – https://github.com/tinyobjloader/tinyobjloader
//   stb_image_write.h       – https://github.com/nothings/stb
// Build (Mac/Linux):
//   g++ -std=c++17 -O2 obj2normalmap.cpp -o obj2normalmap
// -----------------------------------------------------------------------------
// In‑plane rotation explanation
//   dir = (x, y, z, w)
//     • D = normalize(x, y, z) → projection vector.
//     • Basis (U0, V0) is built orthogonal to D.
//     • Apply 2‑D rotation of **w** radians around D:  U = U0·cos − V0·sin,  V = U0·sin + V0·cos.
// -----------------------------------------------------------------------------
// Author: 2025
// -----------------------------------------------------------------------------

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <limits>
#include <filesystem>
#include <iostream>

// -------------------------- External single‑header libs ----------------------
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

// --------------------------------‑‑ types ------------------------------------
struct Pixel{float depth;Vec3 normal;bool filled;Pixel():depth(std::numeric_limits<float>::max()),normal{0,0,0},filled(false){}};

struct Settings{
    std::string objPath; Vec3 dir{0,0,1}; float rot=0; int W=1024, H=1024; float offU=0, offV=0; int idx=0;
};

// ------------------------------ CLI parsing ----------------------------------
static bool isNumber(const char*s){char*c;std::strtod(s,&c);return *c==0;}
static void parseArgs(int argc,char**argv,Settings&s){
    for(int i=1;i<argc;++i){std::string a=argv[i];
        if(a=="--obj" && i+1<argc){s.objPath=argv[++i];}
        else if(a=="--dir" && i+3<argc){
            s.dir={std::stof(argv[++i]),std::stof(argv[++i]),std::stof(argv[++i])};
            if(i+1<argc && isNumber(argv[i+1])) s.rot=std::stof(argv[++i]);
        }
        else if(a=="--size" && i+2<argc){s.W=std::stoi(argv[++i]); s.H=std::stoi(argv[++i]);}
        else if(a=="--offset" && i+2<argc){s.offU=std::stof(argv[++i]); s.offV=std::stof(argv[++i]);}
        else if(a=="--index" && i+1<argc){s.idx=std::stoi(argv[++i]);}
        else {std::cerr << "Unknown/invalid option: " << a << "\n"; std::exit(EXIT_FAILURE);} }
    if(s.objPath.empty()){std::cerr << "--obj parameter required\n"; std::exit(EXIT_FAILURE);} }

// ----------------------------------------------------------------------------
int main(int argc,char**argv){
    Settings cfg; parseArgs(argc,argv,cfg);

    // Load OBJ
    tinyobj::ObjReaderConfig rc; rc.triangulate = true; tinyobj::ObjReader reader;
    if(!reader.ParseFromFile(cfg.objPath, rc)){
        std::cerr << "OBJ load failed: " << reader.Error() << "\n"; return EXIT_FAILURE; }
    const auto &at = reader.GetAttrib(); const auto &sh = reader.GetShapes();

    // Build basis (U, V, D)
    Vec3 D = normalize(cfg.dir);
    Vec3 up = std::fabs(D.y) < 0.99f ? Vec3{0,1,0} : Vec3{1,0,0};
    Vec3 U0 = normalize(cross(up, D));
    Vec3 V0 = cross(D, U0);
    float c = std::cos(cfg.rot), s = std::sin(cfg.rot);
    Vec3 U = U0 * c - V0 * s;
    Vec3 V = U0 * s + V0 * c;

    // Compute bounds in (U,V)
    float minU = 1e30f, maxU = -1e30f, minV = 1e30f, maxV = -1e30f;
    for(size_t i = 0; i < at.vertices.size()/3; ++i){
        Vec3 p{at.vertices[3*i], at.vertices[3*i+1], at.vertices[3*i+2]};
        float u = dot(p,U), v = dot(p,V);
        minU = std::min(minU,u); maxU = std::max(maxU,u);
        minV = std::min(minV,v); maxV = std::max(maxV,v); }
    minU += cfg.offU; maxU += cfg.offU; minV += cfg.offV; maxV += cfg.offV;
    float sU = (maxU-minU)? (cfg.W-1)/(maxU-minU) : 1;
    float sV = (maxV-minV)? (cfg.H-1)/(maxV-minV) : 1;

    // Rasterise
    std::vector<Pixel> fb(cfg.W * cfg.H);
    auto addTri = [&](const Vec3&p0,const Vec3&p1,const Vec3&p2){
        float u0=(dot(p0,U)+cfg.offU-minU)*sU, v0=(dot(p0,V)+cfg.offV-minV)*sV, w0=dot(p0,D);
        float u1=(dot(p1,U)+cfg.offU-minU)*sU, v1=(dot(p1,V)+cfg.offV-minV)*sV, w1=dot(p1,D);
        float u2=(dot(p2,U)+cfg.offU-minU)*sU, v2=(dot(p2,V)+cfg.offV-minV)*sV, w2=dot(p2,D);
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
                float a = w1e*invA, b = w2e*invA, cB = 1.f - a - b;
                float depth = a*w0 + b*w1 + cB*w2;
                Pixel &px = fb[y*cfg.W + x];
                if(depth < px.depth){ px.depth = depth; px.normal = faceN; px.filled = true; } } }
    };

    for(const auto &shape : sh){ size_t off = 0;
        for(size_t f=0; f<shape.mesh.num_face_vertices.size(); ++f){ Vec3 v[3];
            for(int k=0; k<3; ++k){ int id = shape.mesh.indices[off+k].vertex_index;
                v[k] = { at.vertices[3*id], at.vertices[3*id+1], at.vertices[3*id+2] }; }
            off += 3; addTri(v[0],v[1],v[2]); } }

    // Output PNG
    std::vector<unsigned char> img(cfg.W * cfg.H * 3);
    for(int y=0; y<cfg.H; ++y) for(int x=0; x<cfg.W; ++x){ const Pixel&p = fb[y*cfg.W + x]; size_t idx=3*(y*cfg.W+x);
        if(p.filled){ img[idx]=(unsigned char)((p.normal.x*0.5f+0.5f)*255);
                       img[idx+1]=(unsigned char)((p.normal.y*0.5f+0.5f)*255);
                       img[idx+2]=(unsigned char)((p.normal.z*0.5f+0.5f)*255); }
        else { img[idx]=128; img[idx+1]=128; img[idx+2]=255; } }

    std::string out = cfg.objPath + "_normal_map_" + std::to_string(cfg.idx) + ".png";
    if(!stbi_write_png(out.c_str(), cfg.W, cfg.H, 3, img.data(), cfg.W*3)){
        std::cerr << "Failed to write image\n"; return EXIT_FAILURE; }
    std::cout << "Normal map saved to " << out << "\n";
    return EXIT_SUCCESS;
}
