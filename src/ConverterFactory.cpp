#include "ConverterFactory.hpp"
#include "OBJ2NMapConverter.hpp"
#include "OBJ2EdgesConverter.hpp"
#include "OBJ2EdgesConverter_Optimized.hpp"
#include "OBJ2NMapEdgesConverter.hpp"
#include "SVG2RoughConverter.hpp"
#include "NMap2SurfacesConverter.hpp"
#include <stdexcept>

namespace converter_lib {

std::unique_ptr<IConverter> ConverterFactory::create(const std::string& id) {
    if (id == "obj2nmap")       return std::make_unique<OBJ2NMapConverter>();
    if (id == "obj2nmap_edges") return std::make_unique<OBJ2NMapEdgesConverter>();
    if (id == "visible_edges") return std::make_unique<OBJ2EdgesConverter>();
    if (id == "visible_edges_gpu") return std::make_unique<OBJ2EdgesConverter_Optimized>();
    if (id == "svg2roughjs")   return std::make_unique<SVG2RoughConverter>();
    // if (id == "nmap2surfaces") return std::make_unique<NMap2SurfacesConverter>();
    throw std::invalid_argument("Unknown converter: " + id);
}

} // namespace converter_lib