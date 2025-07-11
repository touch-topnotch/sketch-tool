#pragma once
#include "Converter.hpp"
#include <memory>
#include <string>

namespace converter_lib {

class ConverterFactory {
public:
    // создаёт нужный конвертер по ключу, например "obj2nmap"
    static std::unique_ptr<IConverter> create(const std::string& converterId);
};

} // namespace converter_lib