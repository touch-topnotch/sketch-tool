#include "converter_lib/SVG2RoughConverter.hpp"

#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace converter_lib {

// --- helper -----------------------------------------------------------------
static std::string quote(const std::string& s)
{
    // экранируем, если есть пробелы или спец-символы
    if (s.find_first_of(" \t\n\"'") == std::string::npos)
        return s;
#ifdef _WIN32
    return '"' + s + '"';          // windows — двойные кавычки
#else
    return '\'' + s + '\'';        // posix — одинарные кавычки (проще с backslash’ами)
#endif
}

// --- основная функция --------------------------------------------------------
void SVG2RoughConverter::convert(const std::string& in,
                                   const std::string& out,
                                   const Options&    opts)
{
    // где лежит JS-репозиторий
    const auto repoDir = std::filesystem::path(__FILE__)
                           .parent_path()      // src/
                           .parent_path()      // converter_lib/
                           .parent_path()      // project root
                        / "external" / "svg2roughjs";

    const auto cliPath = repoDir / "nodejs-cli" / "src" / "svg2roughjs";

    // --- алиасы, чтобы пользователю было короче писать ----------------------
    static const std::unordered_map<std::string,std::string> alias {
        {"roughness",     "roughConfig.roughness"},
        {"bowing",        "roughConfig.bowing"},
        {"fillStyle",     "roughConfig.fillStyle"},
        {"fillWeight",    "roughConfig.fillWeight"},
        {"hachureGap",    "roughConfig.hachureGap"},
        {"hachureAngle",  "roughConfig.hachureAngle"},
        {"font",          "fontFamily"}
    };

    // --- собираем командную строку ------------------------------------------
    std::ostringstream cmd;
    cmd << "node " << quote(cliPath.string())              // скрипт
        << ' '     << quote(in)                            // входной SVG
        << " -o "  << quote(out);                          // выход

    for (const auto& [k,v] : opts.params)
    {
        std::string key = k;
        auto it = alias.find(k);
        if (it != alias.end()) key = it->second;           // разворачиваем алиас

        // если параметр уже учтён (например, roughness+alias) — пропускаем
        if (key == "input"  || key == "output") continue;

        cmd << " --" << key;

        // булевым true/false тоже нужен value, CLI svg2roughjs так ожидает
        if (!v.empty())
            cmd << ' ' << quote(v);
    }

    // --- запускаем -----------------------------------------------------------
    const std::string fullCmd = cmd.str();
    int ret = std::system(fullCmd.c_str());
    if (ret != 0)
        throw std::runtime_error("[svg2roughjs] failed, exit code "
                                 + std::to_string(ret)
                                 + "\nCommand: " + fullCmd);
}

} // namespace converter_lib