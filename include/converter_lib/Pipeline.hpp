#pragma once
#include "Converter.hpp"
#include <vector>
#include <memory>

namespace converter_lib {

class Pipeline {
    std::vector<std::unique_ptr<IConverter>> steps_;
public:
    // добавляем шаг конвертации
    void addStep(std::unique_ptr<IConverter> step) {
        steps_.push_back(std::move(step));
    }

    // запускаем цепочку: выход каждого шага — вход следующего
    void run(const std::string& input, const std::string& finalOutput, Options opts) {
        std::string curInput = input;
        for (size_t i = 0; i < steps_.size(); ++i) {
            std::string outPath = (i+1==steps_.size())
                ? finalOutput
                : curInput + ".tmp" + std::to_string(i);
            steps_[i]->convert(curInput, outPath, opts);
            curInput = outPath;
        }
    }
};

} // namespace converter_lib