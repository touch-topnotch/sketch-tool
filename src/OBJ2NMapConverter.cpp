/*============================================================================
  OBJ2NMapConverter.cpp  –  part of sketch2cad
  --------------------------------------------------------------------------
  Converts a Wavefront OBJ model into a tangent-space normal map (PNG).
  Supports both orthographic (focal == 0) and perspective projection via
  either explicit focal length *or* field-of-view (vertical FOV in degrees).

  Author: Dmitry Tetkin, 2025 – rewritten June 2025 for perspective/FOV.
============================================================================*/

#define TINYOBJLOADER_IMPLEMENTATION     // include *once* in project
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "converter_lib/OBJ2NMapConverter.hpp"

#include "tiny_obj_loader.h"
#include "stb_image_write.h"

#include <cmath>
#include <cfloat>
#include <filesystem>
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>

namespace converter_lib {

// ──────────────────────── math helpers ─────────────────────────────────────
struct Vec3 { float x, y, z; };
inline Vec3 operator+(Vec3 a, Vec3 b){ return {a.x+b.x, a.y+b.y, a.z+b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b){ return {a.x-b.x, a.y-b.y, a.z-b.z}; }
inline Vec3 operator*(Vec3 a, float s){ return {a.x*s, a.y*s, a.z*s}; }
inline float dot(Vec3 a, Vec3 b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3 cross(Vec3 a, Vec3 b){ return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
inline Vec3 normalize(Vec3 v){ float l=std::sqrt(dot(v,v)); return l? v*(1.f/l):Vec3{0,0,0}; }

// ─────────────────────── option helpers ────────────────────────────────────
static float optF(const Options& o,const std::string& k,float def){auto it=o.params.find(k);return it==o.params.end()?def:std::stof(it->second);} 
static bool  optExists(const Options& o,const std::string& k){return o.params.count(k);} 

static Vec3 parseVec3(const std::string& s, Vec3 def={0,0,0})
{
    if(s.empty()) return def;
    if(s.front()=='(' && s.back()==')'){
        auto body = s.substr(1,s.size()-2);
        size_t p1=body.find(','), p2=body.find(',',p1+1);
        if(p1!=std::string::npos && p2!=std::string::npos){
            try{
                return { std::stof(body.substr(0,p1)),
                         std::stof(body.substr(p1+1,p2-p1-1)),
                         std::stof(body.substr(p2+1)) };
            }catch(...){/* fallthrough */}
        }
    }
    return def; // fallback
}

static Vec3 optVec3(const Options& o,const std::string& key,Vec3 def={0,0,0}){
    auto it=o.params.find(key);
    return it==o.params.end()?def:parseVec3(it->second,def);
}

// ───────────────────────────── convert() ───────────────────────────────────
void OBJ2NMapConverter::convert(const std::string& inPath,
                                const std::string& outPath,
                                const Options&     opts)
{
    // 1. ─────────────── configuration & CLI params ────────────────────────
    Vec3 cam = optExists(opts,"pos")      ? optVec3(opts,"pos")
              : optExists(opts,"pos_x")  ? Vec3{optF(opts,"pos_x",0), optF(opts,"pos_y",0), optF(opts,"pos_z",0)}
              : Vec3{0,0,0};

    Vec3 dir = optExists(opts,"dir")      ? optVec3(opts,"dir", {0,0,1})
              : Vec3{ optF(opts,"dir_x",0), optF(opts,"dir_y",0), optF(opts,"dir_z",1) };
    float rotDeg = optF(opts,"rot",0);             // in-plane (CW) rotation
    int   W      = static_cast<int>(optF(opts,"w",800));
    int   H      = static_cast<int>(optF(opts,"h",800));
    float offU   = optF(opts,"offU",0);            // world-unit offset before fit
    float offV   = optF(opts,"offV",0);

    // perspective vs orthographic ------------------------------------------------
    float focal = optF(opts,"focal", 0);           // px; =0 → ortho
    if(optExists(opts,"fov")){
        float fovDeg = optF(opts,"fov", 45);
        float fovRad = fovDeg * static_cast<float>(M_PI/180.0);
        // convert vertical FOV to focal length in pixels (simple pin-hole)
        focal = (H * 0.5f) / std::tan(fovRad * 0.5f);
    }

    const bool perspective = focal > 0.0f;

    // 2. ─────────────── load OBJ ───────────────────────────────────────────
    tinyobj::ObjReaderConfig rc; rc.triangulate=true;
    tinyobj::ObjReader rdr;
    if(!rdr.ParseFromFile(inPath, rc))
        throw std::runtime_error("OBJ load failed: "+rdr.Error());

    const auto& at  = rdr.GetAttrib();
    const auto& shp = rdr.GetShapes();
    size_t vCnt = at.vertices.size()/3;
    if(!vCnt) throw std::runtime_error("OBJ contains no vertices");

    // 3. ─────────────── compute centre + default camera -------------------
    Vec3 centre{0,0,0};
    for(size_t i=0;i<vCnt;++i)
        centre = centre + Vec3{at.vertices[3*i],at.vertices[3*i+1],at.vertices[3*i+2]};
    centre = centre * (1.f/static_cast<float>(vCnt));

    if(optExists(opts,"pos") || optExists(opts,"pos_x"))
        dir = normalize(centre - cam);                   // look-at
    else
        dir = normalize(dir);

    // If perspective & camera position unspecified → place camera on dir opposite side
    if(perspective && cam.x==0 && cam.y==0 && cam.z==0){
        // crude radius estimate (max distance to centre)
        float r2 = 0.f;
        for(size_t i=0;i<vCnt;++i){
            Vec3 p{at.vertices[3*i],at.vertices[3*i+1],at.vertices[3*i+2]};
            float d2 = dot(p-centre, p-centre); r2 = std::max(r2,d2);
        }
        float dist = std::sqrt(r2)*3.f;                 // step back x3 radius
        cam = centre - dir * dist;
    }

    // 4. ─────────────── camera basis --------------------------------------
    Vec3 D = dir;                                       // forward
    Vec3 up = std::fabs(D.y)<0.99f ? Vec3{0,1,0} : Vec3{1,0,0};
    Vec3 U0 = normalize(cross(up,D));                   // right
    Vec3 V0 = cross(D,U0);                              // up (camera)
    float cr = std::cos(rotDeg*M_PI/180.f), sr=std::sin(rotDeg*M_PI/180.f);
    Vec3 U = U0*cr - V0*sr;
    Vec3 V = U0*sr + V0*cr;

    // 5. ─────────────── produce AABB in projected UV (needed for ortho) ----
    float minU=FLT_MAX,maxU=-FLT_MAX,minV=FLT_MAX,maxV=-FLT_MAX;
    if(!perspective){
        for(size_t i=0;i<vCnt;++i){
            Vec3 p{at.vertices[3*i],at.vertices[3*i+1],at.vertices[3*i+2]};
            float u=dot(p,U), v=dot(p,V);
            minU=std::min(minU,u); maxU=std::max(maxU,u);
            minV=std::min(minV,v); maxV=std::max(maxV,v);
        }
        minU+=offU; maxU+=offU; minV+=offV; maxV+=offV;
    }

    float scale=1.f, padX=0.f, padY=0.f;
    if(!perspective){
        scale = std::min( (W-1)/(maxU-minU), (H-1)/(maxV-minV) );
        padX  = 0.5f*((W-1)-(maxU-minU)*scale);
        padY  = 0.5f*((H-1)-(maxV-minV)*scale);
    }

    // ─────────────── perspective auto-fit (НОВЫЙ БЛОК) ────────────────
    float focalAdj = focal;          // будем использовать вместо focal
    float shiftX   = W * 0.5f;       // сдвиги центра проекции
    float shiftY   = H * 0.5f;

    if (perspective)
    {
        double minX =  1e30, maxX = -1e30;
        double minY =  1e30, maxY = -1e30;

        // проецируем ВСЕ вершины «как есть», без сдвига
        for (size_t i = 0; i < vCnt; ++i)
        {
            Vec3 p{at.vertices[3*i], at.vertices[3*i+1], at.vertices[3*i+2]};
            Vec3 pc = p - cam;
            float z = dot(pc, D); if (z < 1e-4f) z = 1e-4f;

            double x =  focal * dot(pc, U) / z;
            double y = -focal * dot(pc, V) / z;

            minX = std::min(minX, x);  maxX = std::max(maxX, x);
            minY = std::min(minY, y);  maxY = std::max(maxY, y);
        }

        // isotropic масштаб так, чтобы весь bbox влез
        double scale = std::min( static_cast<double>(W) / (maxX - minX),
                                static_cast<double>(H) / (maxY - minY) );

        focalAdj = focal * static_cast<float>(scale);

        // применяем тот же scale к bbox и центрируем
        minX *= scale;  maxX *= scale;
        minY *= scale;  maxY *= scale;

        shiftX = (W - (minX + maxX)) * 0.5;
        shiftY = (H - (minY + maxY)) * 0.5;
    }

    // 6. ─────────────── framebuffer ---------------------------------------
    struct Pixel{ float depth; Vec3 n; bool filled; };
    std::vector<Pixel> fb(W*H,{FLT_MAX,{0,0,0},false});

    auto project = [&](Vec3 p){
        if(perspective){
            Vec3 pc = p - cam;
            float zCam = dot(pc,D);                // distance along forward
            if(zCam < 1e-4f) zCam = 1e-4f;
            float u =  focalAdj * dot(pc,U) / zCam + shiftX;
            float v = -focalAdj * dot(pc,V) / zCam + shiftY;
            return std::tuple<float,float,float>(u, v, zCam);
        }else{
            float u = (dot(p,U)+offU-minU)*scale + padX;
            float v = (dot(p,V)+offV-minV)*scale + padY;
            float z = dot(p,D);
            return std::tuple<float,float,float>(u,v,z);
        }
    };

    auto rasterTri=[&](Vec3 p0,Vec3 p1,Vec3 p2){
        float u0,v0,z0; std::tie(u0,v0,z0)=project(p0);
        float u1,v1,z1; std::tie(u1,v1,z1)=project(p1);
        float u2,v2,z2; std::tie(u2,v2,z2)=project(p2);

        int minX=std::max(0, (int)std::floor(std::min({u0,u1,u2})));
        int maxX=std::min(W-1,(int)std::ceil (std::max({u0,u1,u2})));
        int minY=std::max(0, (int)std::floor(std::min({v0,v1,v2})));
        int maxY=std::min(H-1,(int)std::ceil (std::max({v0,v1,v2})));

        float area=(u1-u0)*(v2-v0)-(v1-v0)*(u2-u0);
        if(std::fabs(area)<1e-6f) return;
        float invA = 1.f/area;
        Vec3 n = normalize(cross(p1-p0,p2-p0));

        for(int y=minY;y<=maxY;++y) for(int x=minX;x<=maxX;++x){
            float w0e=(u1-u0)*(y-v0)-(v1-v0)*(x-u0);
            float w1e=(u2-u1)*(y-v1)-(v2-v1)*(x-u1);
            float w2e=(u0-u2)*(y-v2)-(v0-v2)*(x-u2);
            if((w0e>=0&&w1e>=0&&w2e>=0)||(w0e<=0&&w1e<=0&&w2e<=0)){
                float a=w1e*invA, b=w2e*invA, cλ=1.f-a-b;
                float z = a*z0 + b*z1 + cλ*z2;
                Pixel& px = fb[y*W + x];
                if(z < px.depth){ px.depth = z; px.n = n; px.filled=true; }
            }
        }
    };

    // 7. ─────────────── raster loop ----------------------------------------
    for(const auto& s: shp){
        size_t off=0;
        for(size_t f=0; f<s.mesh.num_face_vertices.size(); ++f){
            Vec3 v[3];
            for(int k=0;k<3;++k){ int id=s.mesh.indices[off+k].vertex_index;
                v[k] = { at.vertices[3*id], at.vertices[3*id+1], at.vertices[3*id+2] }; }
            off += 3;
            rasterTri(v[0],v[1],v[2]);
        }
    }

    // 8. ─────────────── save PNG ------------------------------------------
    std::vector<unsigned char> img(W*H*3,0);
    for(int y=0;y<H;++y) for(int x=0;x<W;++x){
        const Pixel& p = fb[y*W + x]; size_t i = 3*(y*W+x);
        if(p.filled){
            img[i  ]=(unsigned char)((p.n.x*0.5f+0.5f)*255);
            img[i+1]=(unsigned char)((p.n.y*0.5f+0.5f)*255);
            img[i+2]=(unsigned char)((p.n.z*0.5f+0.5f)*255);
        }
    }

    std::filesystem::path out = outPath.empty() ? std::filesystem::path(inPath).replace_extension(".png")
                                               : std::filesystem::path(outPath);
    if(!stbi_write_png(out.string().c_str(), W, H, 3, img.data(), W*3))
        throw std::runtime_error("stb_image_write failed");

    std::cout << "[obj2nmap] wrote " << out << " ("<<W<<"×"<<H<<")\n";
}

} // namespace converter_lib