#pragma once
#include <string>
#include <vector>

namespace converter_lib {

class FileReader {
public:
    static std::vector<char> readAll(const std::string& path);
};

class FileWriter {
public:
    static void writeAll(const std::string& path, const std::vector<char>& data);
};

} // namespace converter_lib