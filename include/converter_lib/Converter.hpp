#pragma once
#include <string>
#include <unordered_map>

namespace converter_lib {

struct Options {
    std::unordered_map<std::string, std::string> params;
};

class IConverter {
public:
    virtual ~IConverter() = default;
    // inputPath → outputPath, с опциями
    virtual void convert(const std::string& inputPath,
                         const std::string& outputPath,
                         const Options& opts) = 0;
};

} // namespace converter_lib