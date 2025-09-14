/*============================================================================
  OBJ2NMapEdgesConverter.cpp – part of sketch2cad
  --------------------------------------------------------------------------
  Converts a Wavefront OBJ model into an SVG of the *edges* of a
  tangent-space normal map: silhouettes (depth discontinuities) and
  hard normal discontinuities. Shares the same camera/projection options
  as OBJ2NMapConverter (ortho/perspective, FOV, rotation, offsets, WxH).

  Author: Dmitry Tetkin, 2025
============================================================================*/

#include "tiny_obj_loader.h"

#include "converter_lib/OBJ2NMapEdgesConverter.hpp"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>
#include <queue>
#include <unordered_set>

namespace converter_lib {

// ─────────────── simple math ──────────────────────────────────────────────
struct Vec3 { float x,y,z; };
inline Vec3 operator+(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline Vec3 operator-(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec3 operator*(Vec3 a,float s){return {a.x*s,a.y*s,a.z*s};}
inline float dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline Vec3 normalize(Vec3 v){float l=std::sqrt(dot(v,v));return l? v*(1.f/l):Vec3{0,0,0};}

// ─────────────── options helpers (совместимы с твоими) ────────────────────
static float optF(const Options& o,const std::string& k,float def){
    auto it=o.params.find(k); return it==o.params.end()?def:std::stof(it->second);
}
static bool  optExists(const Options& o,const std::string& k){return o.params.count(k);} 

static Vec3 parseVec3(const std::string& s, Vec3 def={0,0,0}){
    if(s.empty()) return def;
    if(s.front()=='(' && s.back()==')'){
        auto body=s.substr(1,s.size()-2);
        size_t p1=body.find(','), p2=body.find(',',p1+1);
        if(p1!=std::string::npos && p2!=std::string::npos){
            try{
                return { std::stof(body.substr(0,p1)),
                         std::stof(body.substr(p1+1,p2-p1-1)),
                         std::stof(body.substr(p2+1)) };
            }catch(...){}
        }
    } return def;
}
static Vec3 optVec3(const Options& o,const std::string& k,Vec3 def={0,0,0}){
    auto it=o.params.find(k); return it==o.params.end()?def:parseVec3(it->second,def);
}

// ─────────────── Douglas–Peucker ─────────────────────────────────────────
static float perpDist(const std::pair<float,float>& p,
                      const std::pair<float,float>& a,
                      const std::pair<float,float>& b)
{
    float x=p.first,  y=p.second;
    float x1=a.first, y1=a.second, x2=b.first, y2=b.second;
    float dx=x2-x1, dy=y2-y1;
    if(std::fabs(dx)+std::fabs(dy) < 1e-6f) return std::hypot(x-x1,y-y1);
    float t=((x-x1)*dx+(y-y1)*dy)/(dx*dx+dy*dy);
    float px=x1+t*dx, py=y1+t*dy;
    return std::hypot(x-px,y-py);
}

static void rdp(const std::vector<std::pair<float,float>>& in,
                float eps,
                std::vector<std::pair<float,float>>& out)
{
    if(in.size()<3){ out=in; return; }
    std::vector<char> keep(in.size(),0);
    std::function<void(size_t,size_t)> rec=[&](size_t s,size_t e){
        float maxd=-1; size_t idx=s;
        for(size_t i=s+1;i<e;i++){
            float d=perpDist(in[i],in[s],in[e]);
            if(d>maxd){maxd=d;idx=i;}
        }
        if(maxd>eps){
            rec(s,idx); rec(idx,e);
        }else{
            keep[s]=1; keep[e]=1;
        }
    };
    rec(0,in.size()-1);
    out.clear(); out.reserve(in.size());
    for(size_t i=0;i<in.size();++i) if(keep[i]) out.push_back(in[i]);
    if(out.size()<2){ out.clear(); out.push_back(in.front()); out.push_back(in.back()); }
}

// ─────────────── основной конвертер ───────────────────────────────────────
class OBJ2NMapEdgesConverter_Impl {
public:
    static void convert(const std::string& inPath,
                        const std::string& outPath,
                        const Options&     opts)
    {
        // 1) параметры камеры (как в OBJ2NMapConverter)
        Vec3 cam = optExists(opts,"pos") ? optVec3(opts,"pos")
                   : optExists(opts,"pos_x") ? Vec3{optF(opts,"pos_x",0),optF(opts,"pos_y",0),optF(opts,"pos_z",0)}
                   : Vec3{0,0,0};
        Vec3 dir = optExists(opts,"dir") ? optVec3(opts,"dir",{0,0,1})
                   : Vec3{optF(opts,"dir_x",0),optF(opts,"dir_y",0),optF(opts,"dir_z",1)};
        float rotDeg = optF(opts,"rot",0);
        int   W      = (int)optF(opts,"w",1200);
        int   H      = (int)optF(opts,"h",800);
        float offU   = optF(opts,"offU",0), offV = optF(opts,"offV",0);

        float focal = optF(opts,"focal",0);
        if(optExists(opts,"fov")){
            float fovDeg=optF(opts,"fov",45.f);
            float fovRad=fovDeg*float(M_PI/180.0);
            focal=(H*0.5f)/std::tan(fovRad*0.5f);
        }
        const bool perspective = focal>0;

        // 2) загрузка OBJ
        tinyobj::ObjReaderConfig rc; rc.triangulate = true;
        tinyobj::ObjReader rdr;
        if(!rdr.ParseFromFile(inPath, rc))
            throw std::runtime_error("OBJ load failed: "+rdr.Error());
        const auto& at  = rdr.GetAttrib();
        const auto& shp = rdr.GetShapes();
        size_t vCnt = at.vertices.size()/3;
        if(!vCnt) throw std::runtime_error("OBJ contains no vertices");

        // 3) центр и авто-камера
        Vec3 centre{0,0,0};
        for(size_t i=0;i<vCnt;++i)
            centre = centre + Vec3{at.vertices[3*i],at.vertices[3*i+1],at.vertices[3*i+2]};
        centre = centre*(1.f/float(vCnt));
        if(optExists(opts,"pos") || optExists(opts,"pos_x")) dir = normalize(centre - cam);
        else dir = normalize(dir);
        if(perspective && cam.x==0 && cam.y==0 && cam.z==0){
            float r2=0; 
            for(size_t i=0;i<vCnt;++i){
                Vec3 p{at.vertices[3*i],at.vertices[3*i+1],at.vertices[3*i+2]};
                float d2=dot(p-centre,p-centre); r2=std::max(r2,d2);
            }
            float dist=std::sqrt(r2)*3.f; cam=centre - dir*dist;
        }

        // 4) базис камеры
        Vec3 D=dir;
        Vec3 up = std::fabs(D.y)<0.99f?Vec3{0,1,0}:Vec3{1,0,0};
        Vec3 U0 = normalize(cross(up,D));
        Vec3 V0 = cross(D,U0);
        float cr=std::cos(rotDeg*float(M_PI/180.0)), sr=std::sin(rotDeg*float(M_PI/180.0));
        Vec3 U=U0*cr - V0*sr;
        Vec3 V=U0*sr + V0*cr;

        // 5) подготовка fit (ортho/persp)
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

        float focalAdj=focal; float shiftX=W*0.5f, shiftY=H*0.5f;
        if(perspective){
            double minX=1e30,maxX=-1e30,minY=1e30,maxY=-1e30;
            for(size_t i=0;i<vCnt;++i){
                Vec3 p{at.vertices[3*i],at.vertices[3*i+1],at.vertices[3*i+2]};
                Vec3 pc=p-cam; float z=dot(pc,D); if(z<1e-4f) z=1e-4f;
                double x=focal*dot(pc,U)/z, y=-focal*dot(pc,V)/z;
                minX=std::min(minX,x); maxX=std::max(maxX,x);
                minY=std::min(minY,y); maxY=std::max(maxY,y);
            }
            double sc = std::min(double(W)/(maxX-minX), double(H)/(maxY-minY));
            focalAdj=focal*float(sc);
            minX*=sc; maxX*=sc; minY*=sc; maxY*=sc;
            shiftX=(W-(minX+maxX))*0.5; shiftY=(H-(minY+maxY))*0.5;
        }

        // 6) framebuffer (depth+normal+mask)
        struct Pixel{ float depth; Vec3 n; bool filled; };
        std::vector<Pixel> fb(W*H,{FLT_MAX,{0,0,0},false});

        auto project = [&](Vec3 p){
            if(perspective){
                Vec3 pc=p-cam; float zCam=dot(pc,D); if(zCam<1e-4f) zCam=1e-4f;
                float u=focalAdj*dot(pc,U)/zCam + shiftX;
                float v=-focalAdj*dot(pc,V)/zCam + shiftY;
                return std::tuple<float,float,float>(u,v,zCam);
            }else{
                float u=(dot(p,U)+offU-minU)*scale + padX;
                float v=(dot(p,V)+offV-minV)*scale + padY;
                float z=dot(p,D);
                return std::tuple<float,float,float>(u,v,z);
            }
        };

        auto rasterTri=[&](Vec3 p0,Vec3 p1,Vec3 p2){
            float u0,v0,z0; std::tie(u0,v0,z0)=project(p0);
            float u1,v1,z1; std::tie(u1,v1,z1)=project(p1);
            float u2,v2,z2; std::tie(u2,v2,z2)=project(p2);

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
                    float a=w1e*invA, b=w2e*invA, cλ=1.f-a-b;
                    float z=a*z0+b*z1+cλ*z2;
                    Pixel& px=fb[y*W+x];
                    if(z<px.depth){ px.depth=z; px.n=n; px.filled=true; }
                }
            }
        };

        for(const auto& s: shp){
            size_t off=0;
            for(size_t f=0; f<s.mesh.num_face_vertices.size(); ++f){
                Vec3 v[3];
                for(int k=0;k<3;++k){ int id=s.mesh.indices[off+k].vertex_index;
                    v[k]={at.vertices[3*id],at.vertices[3*id+1],at.vertices[3*id+2]};
                }
                off+=3; rasterTri(v[0],v[1],v[2]);
            }
        }

        // 7) детект рёбер (силуэты + скачки нормалей)
        float normalAngleDeg = optF(opts,"edge_normal_deg", 25.0f);
        float depthJumpRel   = optF(opts,"edge_depth_rel",  0.02f);   // 2% от глубины
        bool  closeLoops     = optF(opts,"edge_close", 1)!=0;

        float cosThr = std::cos(normalAngleDeg*float(M_PI/180.0));

        std::vector<unsigned char> edge(W*H, 0);
        auto idx=[&](int x,int y){return y*W+x;};
        const int dx[8]={1,1,0,-1,-1,-1,0,1};
        const int dy[8]={0,1,1,1,0,-1,-1,-1};

        for(int y=1;y<H-1;++y){
            for(int x=1;x<W-1;++x){
                const Pixel& p = fb[idx(x,y)];
                if(!p.filled){ 
                    // пиксель пуст → возможно силуэт, если сосед заполнен
                    bool neighFilled=false;
                    for(int k=0;k<8;++k){
                        const Pixel& q=fb[idx(x+dx[k],y+dy[k])];
                        neighFilled |= q.filled;
                    }
                    if(neighFilled){ edge[idx(x,y)]=255; }
                    continue;
                }
                // сравнение с 8-соседями
                bool mark=false;
                for(int k=0;k<8 && !mark;++k){
                    const Pixel& q=fb[idx(x+dx[k],y+dy[k])];
                    if(!q.filled){ mark=true; break; } // граница покрытия
                    // скачок глубины (силуэт по Z)
                    float zA=p.depth, zB=q.depth;
                    float dz=std::fabs(zA-zB);
                    if(dz > std::max(1e-6f, depthJumpRel*std::max(zA,zB))){ mark=true; break; }
                    // скачок нормали
                    float c = dot(p.n, q.n);
                    if(c < cosThr){ mark=true; break; }
                }
                if(mark) edge[idx(x,y)]=255;
            }
        }

        // 8) трассировка полилиний (8-связность)
        std::vector<char> vis(W*H,0);
        std::vector<std::vector<std::pair<float,float>>> polylines;

        for(int y=0;y<H;++y){
            for(int x=0;x<W;++x){
                if(!edge[idx(x,y)] || vis[idx(x,y)]) continue;

                // старт новой цепочки – расширяем в обе стороны
                std::vector<std::pair<int,int>> chain;
                auto extend=[&](int sx,int sy,int dirSign){
                    int cx=sx, cy=sy;
                    while(true){
                        vis[idx(cx,cy)]=1; chain.push_back({cx,cy});
                        bool step=false;
                        // смотрим соседей – предпочтительно туда, куда «продолжается» линия
                        for(int k=0;k<8;++k){
                            int nx=cx+dx[k], ny=cy+dy[k];
                            if(nx<0||ny<0||nx>=W||ny>=H) continue;
                            int id=idx(nx,ny);
                            if(edge[id] && !vis[id]){
                                cx=nx; cy=ny; step=true; break;
                            }
                        }
                        if(!step) break;
                    }
                };
                extend(x,y,+1);

                // координаты → float, БЕЗ инверсии для SVG
                std::vector<std::pair<float,float>> poly;
                poly.reserve(chain.size());
                for(auto& pxy: chain){
                    poly.emplace_back( float(pxy.first)+0.5f, float(pxy.second)+0.5f );
                }
                if(poly.size()>=2) polylines.push_back(std::move(poly));
            }
        }

        // 9) упрощение RDP
        float simplifyEps = optF(opts,"edge_simplify_px", 0.8f);
        for(auto& pl : polylines){
            std::vector<std::pair<float,float>> simp;
            rdp(pl, simplifyEps, simp);
            pl.swap(simp);
        }

        // 10) вывод SVG
        std::filesystem::path out = outPath.empty()
            ? std::filesystem::path(inPath).replace_extension(".svg")
            : std::filesystem::path(outPath);

        std::ofstream os(out.string());
        if(!os) throw std::runtime_error("cannot open SVG for write");

        const float strokeWidth = optF(opts,"stroke", 2.0f);
        const std::string strokeColor = "#000"; // чёрный контур

        os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        os << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
           << "width=\""<<W<<"\" height=\""<<H<<"\" viewBox=\"0 0 "<<W<<" "<<H<<"\" "
           << "fill=\"none\" stroke=\""<<strokeColor<<"\" stroke-width=\""<<strokeWidth<<"\" "
           << "stroke-linecap=\"round\" stroke-linejoin=\"round\">\n";

        // фон (опционально: бумага/сепия)
        if(optF(opts,"bg", 1)!=0){
            std::string bg = optExists(opts,"bg_hex") ? opts.params.at("bg_hex") : "#f4ead2";
            os << "<rect x=\"0\" y=\"0\" width=\"100%\" height=\"100%\" fill=\""<<bg<<"\"/>\n";
            os << "<g stroke=\"#000\" fill=\"none\">\n";
        }

        for(const auto& pl : polylines){
            if(pl.size()<2) continue;
            os << "<path d=\"M";
            for(size_t i=0;i<pl.size();++i){
                os << (i? " L":" ") << pl[i].first << " " << pl[i].second;
            }
            if(closeLoops && (std::hypot(pl.front().first-pl.back().first,
                                         pl.front().second-pl.back().second) < 1.01f))
                os << " Z";
            os << "\"/>\n";
        }

        if(optF(opts,"bg",1)!=0) os << "</g>\n";
        os << "</svg>\n";
        os.close();

        std::cout << "[obj2nmap-edges] wrote " << out << " ("<<W<<"×"<<H<<")\n";
    }
};

// ────────── реализация интерфейса IConverter ──────────────────────────────
void OBJ2NMapEdgesConverter::convert(const std::string& inputPath,
                                     const std::string& outputPath,
                                     const Options&     opts)
{
    OBJ2NMapEdgesConverter_Impl::convert(inputPath, outputPath, opts);
}

} // namespace converter_lib