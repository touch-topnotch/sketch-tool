#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "converter_lib/OBJ2NMapConverter.hpp"
#include <cmath>
#include <cstring>
#include <cfloat>
#include <filesystem>
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>

namespace converter_lib {

// ---------- small math -------------------------------------------------------
struct Vec3 { float x,y,z; };
inline Vec3 operator+(Vec3 a,Vec3 b){return{a.x+b.x,a.y+b.y,a.z+b.z};}
inline Vec3 operator-(Vec3 a,Vec3 b){return{a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec3 operator*(Vec3 a,float s){return{a.x*s,a.y*s,a.z*s};}
inline float dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 cross(Vec3 a,Vec3 b){return{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline Vec3 normalize(Vec3 v){float l=std::sqrt(dot(v,v));return l? v*(1.f/l):Vec3{0,0,0};}

// ---------- helper to fetch option or default --------------------------------
static float optF(const Options& o, const std::string& key, float def) {
    auto it = o.params.find(key); return it==o.params.end()? def : std::stof(it->second);
}
static std::string optS(const Options& o,const std::string&key,const std::string&def){
    auto it=o.params.find(key); return it==o.params.end()?def:it->second;
}

// -----------------------------------------------------------------------------
/// core
void OBJ2NMapConverter::convert(const std::string& inPath,
                                const std::string& outPath,
                                const Options& opts)
{
    // ---------- settings -----------------------------------------------------
    Vec3 cam { optF(opts,"cam_x",0), optF(opts,"cam_y",0), optF(opts,"cam_z",0) };
    bool useCam = opts.params.count("cam_x")||opts.params.count("cam_y")||opts.params.count("cam_z");
    Vec3 dir { optF(opts,"dir_x",0), optF(opts,"dir_y",0), optF(opts,"dir_z",1) };
    float rotDeg = optF(opts,"rot",0);
    int   W      = static_cast<int>(optF(opts,"W",1024));
    int   H      = static_cast<int>(optF(opts,"H",1024));
    float offU   = optF(opts,"offU",0);
    float offV   = optF(opts,"offV",0);

    // ---------- load OBJ -----------------------------------------------------
    tinyobj::ObjReaderConfig rc; rc.triangulate = true;
    tinyobj::ObjReader rdr;
    if(!rdr.ParseFromFile(inPath, rc))
        throw std::runtime_error("OBJ load failed: "+rdr.Error());

    const auto& at  = rdr.GetAttrib();
    const auto& shp = rdr.GetShapes();
    size_t vCnt = at.vertices.size()/3;
    if(vCnt==0) throw std::runtime_error("OBJ contains no vertices");

    // ---------- centre of mass & view dir ------------------------------------
    Vec3 centre{0,0,0};
    for(size_t i=0;i<vCnt;++i)
        centre = centre + Vec3{at.vertices[3*i], at.vertices[3*i+1], at.vertices[3*i+2]};
    centre = centre*(1.f/static_cast<float>(vCnt));

    if(useCam)
        dir = normalize(centre - cam);
    else
        dir = normalize(dir);

    // ---------- camera basis (U,V,D) ----------------------------------------
    Vec3 D = dir;
    Vec3 up = std::fabs(D.y) < .99f ? Vec3{0,1,0} : Vec3{1,0,0};
    Vec3 U0 = normalize(cross(up,D));
    Vec3 V0 = cross(D,U0);
    float cr = std::cos(rotDeg*M_PI/180.f), sr = std::sin(rotDeg*M_PI/180.f);
    Vec3 U = U0*cr - V0*sr;
    Vec3 V = U0*sr + V0*cr;

    // ---------- project bounds ----------------------------------------------
    float minU=FLT_MAX,maxU=-FLT_MAX,minV=FLT_MAX,maxV=-FLT_MAX;
    for(size_t i=0;i<vCnt;++i){
        Vec3 p{at.vertices[3*i],at.vertices[3*i+1],at.vertices[3*i+2]};
        float u=dot(p,U), v=dot(p,V);
        minU=std::min(minU,u); maxU=std::max(maxU,u);
        minV=std::min(minV,v); maxV=std::max(maxV,v);
    }
    minU+=offU; maxU+=offU; minV+=offV; maxV+=offV;

    float scale = std::min( (W-1)/(maxU-minU), (H-1)/(maxV-minV) );
    float padX  = 0.5f*((W-1)-(maxU-minU)*scale);
    float padY  = 0.5f*((H-1)-(maxV-minV)*scale);

    // ---------- tiny framebuffer --------------------------------------------
    struct Pixel{float depth;Vec3 n;bool filled;};
    std::vector<Pixel> fb(W*H,{FLT_MAX,{0,0,0},false});

    auto rasterTri = [&](Vec3 p0,Vec3 p1,Vec3 p2)
    {
        auto proj=[&](Vec3 p){
            float u=(dot(p,U)+offU-minU)*scale+padX;
            float v=(dot(p,V)+offV-minV)*scale+padY;
            float w=dot(p,D);
            return std::tuple<float,float,float>(u,v,w);
        };

        float u0,v0,w0; std::tie(u0,v0,w0)=proj(p0);
        float u1,v1,w1; std::tie(u1,v1,w1)=proj(p1);
        float u2,v2,w2; std::tie(u2,v2,w2)=proj(p2);

        int minX=std::max(0,(int)std::floor(std::min({u0,u1,u2})));
        int maxX=std::min(W-1,(int)std::ceil (std::max({u0,u1,u2})));
        int minY=std::max(0,(int)std::floor(std::min({v0,v1,v2})));
        int maxY=std::min(H-1,(int)std::ceil (std::max({v0,v1,v2})));

        float area=(u1-u0)*(v2-v0)-(v1-v0)*(u2-u0);
        if(std::fabs(area)<1e-6f) return;
        float invA=1.f/area;
        Vec3 n=normalize(cross(p1-p0,p2-p0));

        for(int y=minY;y<=maxY;++y) for(int x=minX;x<=maxX;++x){
            float w0e=(u1-u0)*(y-v0)-(v1-v0)*(x-u0);
            float w1e=(u2-u1)*(y-v1)-(v2-v1)*(x-u1);
            float w2e=(u0-u2)*(y-v2)-(v0-v2)*(x-u2);
            if((w0e>=0&&w1e>=0&&w2e>=0)||(w0e<=0&&w1e<=0&&w2e<=0)){
                float a=w1e*invA,b=w2e*invA,cλ=1.f-a-b;
                float depth=a*w0+b*w1+cλ*w2;
                Pixel& px=fb[y*W+x];
                if(depth<px.depth){px.depth=depth;px.n=n;px.filled=true;}
            }
        }
    };

    for(const auto& s:shp){
        size_t off=0;
        for(size_t f=0;f<s.mesh.num_face_vertices.size();++f){
            Vec3 v[3]; for(int k=0;k<3;++k){
                int id=s.mesh.indices[off+k].vertex_index;
                v[k]={at.vertices[3*id],at.vertices[3*id+1],at.vertices[3*id+2]};
            }
            off+=3; rasterTri(v[0],v[1],v[2]);
        }
    }

    // ---------- store PNG ----------------------------------------------------
    std::vector<unsigned char> img(W*H*3,0);
    for(int y=0;y<H;++y)for(int x=0;x<W;++x){
        const Pixel& p=fb[y*W+x]; size_t i=3*(y*W+x);
        if(p.filled){
            img[i  ]=(unsigned char)((p.n.x*0.5f+0.5f)*255);
            img[i+1]=(unsigned char)((p.n.y*0.5f+0.5f)*255);
            img[i+2]=(unsigned char)((p.n.z*0.5f+0.5f)*255);
        }
    }

    std::filesystem::path out=outPath.empty()
        ? std::filesystem::path(inPath).replace_extension(".png")
        : std::filesystem::path(outPath);
    if(!stbi_write_png(out.string().c_str(),W,H,3,img.data(),W*3))
        throw std::runtime_error("stb_image_write failed");

    std::cout<<"[obj2nmap] wrote "<<out<<" ("<<W<<"×"<<H<<")\n";
}

} // namespace converter_lib