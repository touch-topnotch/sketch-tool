#pragma once
#include "Converter.hpp"

namespace converter_lib {

class OBJ2EdgesConverter_Optimized final : public IConverter {
public:
    void convert(const std::string& inputPath,
                 const std::string& outputPath,
                 const Options&     opts) override;
};

} // namespace converter_lib 