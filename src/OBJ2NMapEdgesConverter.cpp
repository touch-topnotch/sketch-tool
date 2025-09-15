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
#include <unordered_map>

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
        
        // Параметры для видимости рёбер (нужны для rasterTri)
        float polyOffsetZ = optF(opts, "poly_offset_z", 0.002f); // увеличенный оффсет

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
                    z -= polyOffsetZ; // полигон-оффсет для избежания самозакрытия
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

        // 7) Собираем adjacency рёбер
        struct Edge { 
            int a, b; 
            int f1 = -1, f2 = -1; 
            Vec3 n1, n2; 
            float d1 = 0, d2 = 0; 
        };
        std::unordered_map<uint64_t, int> eMap;
        std::vector<Edge> edges;

        auto keyAB = [&](int i, int j) { 
            if(i > j) std::swap(i, j); 
            return (uint64_t(i) << 32) | uint32_t(j); 
        };

        // Параметры для классификации рёбер
        float edgeNormalDeg = optF(opts, "edge_normal_deg", 20.0f);
        float planeGap = optF(opts, "plane_gap", 0.01f);
        float tauZ = optF(opts, "tau_z", 0.5f);
        float stepPx = optF(opts, "step_px", 0.5f);
        bool closeLoops = optF(opts, "edge_close", 1) != 0;
        
        // Параметры для видимости рёбер (улучшенные для лучшего скрытия)
        float edgeSamplePx = optF(opts, "edge_sample_px", 0.3f);  // более мелкая дискретизация
        float depthEpsAbs = optF(opts, "edge_tau_abs", 0.1f);     // более строгий допуск
        float depthEpsRel = optF(opts, "edge_tau_rel", 0.005f);   // более строгий относительный допуск

        float cosThr = std::cos(edgeNormalDeg * float(M_PI/180.0));

        // Собираем рёбра из треугольников
        int triIndex = 0;
        for(const auto& s: shp){
            size_t off = 0;
            for(size_t f = 0; f < s.mesh.num_face_vertices.size(); ++f){
                int vi[3];
                Vec3 p[3];
                for(int k = 0; k < 3; ++k){ 
                    int id = s.mesh.indices[off + k].vertex_index;
                    vi[k] = id;
                    p[k] = {at.vertices[3*id], at.vertices[3*id+1], at.vertices[3*id+2]};
                }
                
                Vec3 n = normalize(cross(p[1] - p[0], p[2] - p[0]));
                float d = -dot(n, p[0]);
                
                for(int k = 0; k < 3; ++k){
                    int a = vi[k], b = vi[(k+1) % 3];
                    if(a > b) std::swap(a, b);
                    auto K = keyAB(a, b);
                    int idx;
                    if(!eMap.count(K)){ 
                        idx = edges.size(); 
                        eMap[K] = idx; 
                        edges.push_back({a, b}); 
                    }
                    idx = eMap[K];
                    Edge& e = edges[idx];
                    if(e.f1 == -1){ 
                        e.f1 = triIndex; 
                        e.n1 = n; 
                        e.d1 = d; 
                    } else { 
                        e.f2 = triIndex; 
                        e.n2 = n; 
                        e.d2 = d; 
                    }
                }
                off += 3;
                triIndex++;
            }
        }

        // 8) Фильтруем, проецируем, скрываем
        std::vector<std::vector<std::pair<float,float>>> paths;
        
        // Функция для отсечения отрезка по прямоугольнику (возвращает параметры t)
        auto clipToRectParametric = [](float x0, float y0, float x1, float y1, 
                                      float minX, float minY, float maxX, float maxY, 
                                      float& t0, float& t1) -> bool {
            float dx = x1 - x0, dy = y1 - y0;
            if(std::fabs(dx) < 1e-6f && std::fabs(dy) < 1e-6f) return false;
            
            t0 = 0.0f; t1 = 1.0f;
            
            // X-отсечение
            if(dx > 0) {
                if(x0 < minX) t0 = std::max(t0, (minX - x0) / dx);
                if(x1 > maxX) t1 = std::min(t1, (maxX - x0) / dx);
            } else if(dx < 0) {
                if(x0 > maxX) t0 = std::max(t0, (maxX - x0) / dx);
                if(x1 < minX) t1 = std::min(t1, (minX - x0) / dx);
            } else {
                if(x0 < minX || x0 > maxX) return false;
            }
            
            // Y-отсечение
            if(dy > 0) {
                if(y0 < minY) t0 = std::max(t0, (minY - y0) / dy);
                if(y1 > maxY) t1 = std::min(t1, (maxY - y0) / dy);
            } else if(dy < 0) {
                if(y0 > maxY) t0 = std::max(t0, (maxY - y0) / dy);
                if(y1 < minY) t1 = std::min(t1, (minY - y0) / dy);
            } else {
                if(y0 < minY || y0 > maxY) return false;
            }
            
            return t0 <= t1;
        };
        
        // Функция для определения видимых интервалов на ребре (Z-буферный подход)
        auto visibleIntervalsOnEdge = [&](Vec3 A, Vec3 B) -> std::vector<std::pair<float,float>> {
            // Проекция концов
            auto P0 = project(A); float x0=std::get<0>(P0), y0=std::get<1>(P0), z0=std::get<2>(P0);
            auto P1 = project(B); float x1=std::get<0>(P1), y1=std::get<1>(P1), z1=std::get<2>(P1);

            // Клип по экрану → t∈[t0,t1]
            float t0=0.f, t1=1.f;
            auto clipped = clipToRectParametric(x0,y0,x1,y1, 0,0, float(W-1), float(H-1), t0, t1);
            if(!clipped) return {};

            // Сколько сэмплов (более плотная дискретизация)
            float len_px = std::hypot((x1-x0)*(t1-t0), (y1-y0)*(t1-t0));
            int S = std::max(4, int(len_px / 0.2f)); // шаг 0.2px

            std::vector<std::pair<float,float>> out;
            bool visPrev=false; float tStart=t0, tPrev=t0;

            auto sample = [&](float t){
                Vec3 P = A*(1.f-t)+B*t;
                auto Pr = project(P);
                float x = std::get<0>(Pr), y = std::get<1>(Pr), z = std::get<2>(Pr);
                int ix = std::clamp(int(std::floor(x+0.5f)), 0, W-1);
                int iy = std::clamp(int(std::floor(y+0.5f)), 0, H-1);
                const auto& px = fb[iy*W+ix];

                if(!px.filled) return true; // фон/дырка → видно

                // Строгий допуск по глубине
                float tau = 0.05f; // фиксированный допуск
                return (z <= px.depth + tau);
            };

            for(int s=0; s<=S; ++s){
                float t = t0 + (t1-t0) * (float(s)/float(S));
                bool vis = sample(t);

                if(vis && !visPrev){ tStart = t; }
                if(!vis && visPrev){ out.emplace_back(tStart, tPrev); }
                visPrev = vis; tPrev = t;
            }
            if(visPrev) out.emplace_back(tStart, tPrev);

            // Слить близкие интервалы
            const float joinGap = (t1-t0) / float(std::max(4,S));
            std::vector<std::pair<float,float>> merged;
            for(auto seg: out){
                if(!merged.empty() && seg.first - merged.back().second <= 3.0f*joinGap){
                    merged.back().second = seg.second;
                } else merged.push_back(seg);
            }
            
            // Фильтрация: удаляем очень короткие интервалы
            std::vector<std::pair<float,float>> filtered;
            for(auto seg: merged){
                float len = seg.second - seg.first;
                if(len > 0.02f) { // минимум 2% от длины ребра
                    filtered.push_back(seg);
                }
            }
            return filtered;
        };

        for(const Edge& e: edges) {
            // Критерии отрисовки
            bool draw = false;
            Vec3 n1 = e.n1, n2 = e.n2;
            
            if(e.f2 == -1) {
                draw = true; // граница меша
            } else {
                float c = dot(n1, n2);
                if(c < cosThr) draw = true; // жёсткий излом нормалей
                
                // Силуэт
                float s1 = dot(n1, dir); 
                float s2 = dot(n2, dir);
                if(s1 * s2 < 0) draw = true;
                
                // Параллельные, но разные плоскости
                if(std::fabs(c) > 1-1e-4f && std::fabs(e.d1 - e.d2) > planeGap) draw = true;
            }
            if(!draw) continue;

            // 3D отрезок
            Vec3 A = {at.vertices[3*e.a], at.vertices[3*e.a+1], at.vertices[3*e.a+2]};
            Vec3 B = {at.vertices[3*e.b], at.vertices[3*e.b+1], at.vertices[3*e.b+2]};
            
            // Определяем видимые интервалы
            auto vis = visibleIntervalsOnEdge(A, B);
            for(auto [ta, tb] : vis){
                Vec3 Pa = A*(1.f-ta) + B*ta;
                Vec3 Pb = A*(1.f-tb) + B*tb;
                auto A2 = project(Pa); auto B2 = project(Pb);
                paths.push_back({{std::get<0>(A2), std::get<1>(A2)}, 
                               {std::get<0>(B2), std::get<1>(B2)}});
            }
        }

        // 9) Слияние/упрощение и вывод SVG
        float simplifyEps = optF(opts, "edge_simplify_px", 0.4f);
        std::vector<std::vector<std::pair<float,float>>> polylines;
        
        // Простое слияние коллинеарных сегментов
        for(auto& path : paths) {
            if(path.size() < 2) continue;
            
            std::vector<std::pair<float,float>> merged;
            merged.push_back(path[0]);
            
            for(size_t i = 1; i < path.size(); ++i) {
                auto& prev = merged.back();
                auto& curr = path[i];
                
                // Проверяем коллинеарность и близость
                if(merged.size() > 1) {
                    auto& pprev = merged[merged.size()-2];
                    float dx1 = prev.first - pprev.first, dy1 = prev.second - pprev.second;
                    float dx2 = curr.first - prev.first, dy2 = curr.second - prev.second;
                    
                    float cross = std::fabs(dx1 * dy2 - dy1 * dx2);
                    float dist = std::hypot(dx2, dy2);
                    
                    if(cross < 0.1f && dist < 0.75f) {
                        // Коллинеарны и близко - заменяем среднюю точку
                        merged.back() = {(prev.first + curr.first) * 0.5f, 
                                       (prev.second + curr.second) * 0.5f};
                    } else {
                        merged.push_back(curr);
                    }
                } else {
                    merged.push_back(curr);
                }
            }
            
            if(merged.size() >= 2) {
                // RDP упрощение
            std::vector<std::pair<float,float>> simp;
                rdp(merged, simplifyEps, simp);
                polylines.push_back(std::move(simp));
            }
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