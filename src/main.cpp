#include "Pipeline.hpp"
#include "ConverterFactory.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    // простая разборка аргументов (можно заменить на CLI-библиотеку)
    // Например: converter_pipeline input.obj output.svg obj2nmap visible_edges svg2roughjs
    if (argc < 5) { std::cerr<<"Usage: ...\n"; return 1; }

    std::string in  = argv[1];
    std::string out = argv[2];
    converter_lib::Options opts;
    // … заполнить opts из argc/argv …

    converter_lib::Pipeline pipeline;
    for (int i = 3; i < argc; ++i)
        pipeline.addStep(converter_lib::ConverterFactory::create(argv[i]));

    pipeline.run(in, out, opts);
    return 0;
}