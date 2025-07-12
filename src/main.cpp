#include "Pipeline.hpp"
#include "ConverterFactory.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    // Check for help/version flags first
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " <input> <output> <converter1> [converter2...] [--param value]...\n";
            std::cout << "\nExample: " << argv[0] << " input.obj output.svg obj2nmap visible_edges --focal 1000 --pos \"(10,10,10)\"\n";
            std::cout << "\nAvailable converters:\n";
            std::cout << "  obj2nmap      - Convert OBJ to normal map\n";
            std::cout << "  visible_edges - Extract visible edges from OBJ model as SVG\n";
            std::cout << "  svg2roughjs   - Convert SVG to rough (hand-drawn) sketch\n";
            std::cout << "  nmap2surfaces - Extract surfaces from normal map\n";
            std::cout << "\nCommon parameters:\n";
            std::cout << "  --focal <value>     - Focal length for perspective projection (default: 0 = orthographic)\n";
            std::cout << "  --pos \"(x,y,z)\"     - Camera position\n";
            std::cout << "  --dir \"(x,y,z)\"     - View direction\n";
            std::cout << "  --W <width>         - Output width (default: 1024)\n";
            std::cout << "  --H <height>        - Output height (default: 1024)\n";
            std::cout << "  --roughness <value> - Roughness for SVG2RoughJS (default: 1.0)\n";
            return 0;
        } else if (arg == "--version" || arg == "-v") {
            std::cout << "Sketch2CAD v1.0.0\n";
            return 0;
        }
    }
    
    // Parse arguments: converter_cli input.obj output.svg converter1 converter2 [--param value]...
    if (argc < 4) { 
        std::cerr << "Usage: " << argv[0] << " <input> <output> <converter1> [converter2...] [--param value]...\n";
        std::cerr << "Example: " << argv[0] << " input.obj output.svg obj2nmap visible_edges --focal 1000 --pos \"(10,10,10)\"\n";
        std::cerr << "Use --help for more information.\n";
        return 1; 
    }

    std::string in  = argv[1];
    std::string out = argv[2];
    converter_lib::Options opts;
    
    // Parse converters and parameters
    std::vector<std::string> converters;
    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.substr(0, 2) == "--") {
            // Parameter: --key value
            if (i + 1 < argc) {
                std::string key = arg.substr(2);
                std::string value = argv[i + 1];
                opts.params[key] = value;
                ++i; // Skip the value in next iteration
            } else {
                std::cerr << "Error: Parameter " << arg << " requires a value\n";
                return 1;
            }
        } else {
            // Converter name
            converters.push_back(arg);
        }
    }

    if (converters.empty()) {
        std::cerr << "Error: No converters specified\n";
        return 1;
    }

    converter_lib::Pipeline pipeline;
    for (const auto& converter : converters)
        pipeline.addStep(converter_lib::ConverterFactory::create(converter));

    pipeline.run(in, out, opts);
    return 0;
}