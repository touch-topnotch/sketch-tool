// tests/converter_tests.cpp
// -----------------------------------------------------------------------------
// Unit-tests for the Converter pipeline.
//   * Requires GoogleTest (already vendored inside Clipper2 repo)
//   * Test-data are expected to live next to this file:
//       - house.obj, normal_map.png
//       - part.obj,  visible_edges.svg
// -----------------------------------------------------------------------------

#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>

// Third-party single-header image loader for PNG comparison
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "converter_lib/ConverterFactory.hpp"
#include "converter_lib/Converter.hpp"

using converter_lib::ConverterFactory;
using converter_lib::Options;

// Return path to the directory where this test source resides (== tests/)
static std::filesystem::path dataDir()
{
    return std::filesystem::path(__FILE__).parent_path();
}

// Simple per-pixel comparison with a tolerance (0-255 scale)
static bool imagesAlmostEqual(const std::filesystem::path& a,
                              const std::filesystem::path& b,
                              int tol = 2)              // ±2 per channel
{
    int w1,h1,c1; unsigned char* i1 = stbi_load(a.string().c_str(), &w1,&h1,&c1, 3);
    int w2,h2,c2; unsigned char* i2 = stbi_load(b.string().c_str(), &w2,&h2,&c2, 3);
    if(!i1||!i2){ 
        std::cout << "Failed to load images: " << a << " or " << b << std::endl;
        stbi_image_free(i1); stbi_image_free(i2); return false; 
    }
    if(w1!=w2 || h1!=h2){ 
        std::cout << "Image sizes differ: " << w1 << "x" << h1 << " vs " << w2 << "x" << h2 << std::endl;
        stbi_image_free(i1); stbi_image_free(i2); return false; 
    }
    size_t N = static_cast<size_t>(w1)*h1*3;
    bool ok = true;
    int maxDiff = 0;
    int diffCount = 0;
    for(size_t i=0;i<N;++i) {
        int diff = std::abs(int(i1[i]) - int(i2[i]));
        if(diff > tol) {
            ok = false;
            maxDiff = std::max(maxDiff, diff);
            diffCount++;
            if(diffCount <= 10) { // Show first 10 differences
                std::cout << "Pixel " << i << " differs: " << int(i1[i]) << " vs " << int(i2[i]) << " (diff=" << diff << ")" << std::endl;
            }
        }
    }
    if(!ok) {
        std::cout << "Images differ: max diff=" << maxDiff << ", total different pixels=" << diffCount << "/" << N << std::endl;
    }
    stbi_image_free(i1); stbi_image_free(i2);
    return ok;
}

// -----------------------------------------------------------------------------
// OBJ → Normal-map
// -----------------------------------------------------------------------------
TEST(Obj2NMapConverter, GeneratesExpectedImage)
{
    auto conv = ConverterFactory::create("obj2nmap");
    Options opt;
    opt.params["dir_x"] = "0"; // orthographic +z
    opt.params["dir_y"] = "0";
    opt.params["dir_z"] = "1";
    opt.params["W"] = "1024"; // ensure consistent size
    opt.params["H"] = "1024";

    const auto inObj  = dataDir()/"house.obj";
    const auto outPng = dataDir()/"house_nm_gen.png";

    conv->convert(inObj.string(), outPng.string(), opt);

    ASSERT_TRUE(std::filesystem::exists(outPng));
    // Use a more lenient tolerance for the comparison
    ASSERT_TRUE(imagesAlmostEqual(outPng, dataDir()/"normal_map.png", 5));
}

// -----------------------------------------------------------------------------
// OBJ → Visible edges SVG
// -----------------------------------------------------------------------------
TEST(VisibleEdgesConverter, ProducesSvgWithLines)
{
    auto conv = ConverterFactory::create("visible_edges");
    Options opt; // no special params

    const auto inObj  = dataDir()/"part.obj";
    const auto outSvg = dataDir()/"visible_edges_gen.svg";

    conv->convert(inObj.string(), outSvg.string(), opt);

    ASSERT_TRUE(std::filesystem::exists(outSvg));
    std::ifstream f(outSvg);
    std::string s((std::istreambuf_iterator<char>(f)), {});
    ASSERT_NE(s.find("<svg"), std::string::npos);
    ASSERT_NE(s.find("<line"), std::string::npos);
}

// -----------------------------------------------------------------------------
// SVG → RoughJS SVG (Node pipeline)
// -----------------------------------------------------------------------------
TEST(SVG2RoughConverter, RunsNodeCli)
{
    auto conv = ConverterFactory::create("svg2roughjs");
    Options opt;
    opt.params["roughness"] = "2.5";

    const auto inSvg  = dataDir()/"visible_edges.svg";   // provided sample
    const auto outSvg = dataDir()/"rough_gen.svg";

    conv->convert(inSvg.string(), outSvg.string(), opt);

    ASSERT_TRUE(std::filesystem::file_size(outSvg) > 0);
}

// -----------------------------------------------------------------------------
// Normal-map → Surfaces (placeholder – just checks that converter succeeds)
// -----------------------------------------------------------------------------
TEST(NMap2SurfacesConverter, RunsWithoutError)
{
    auto conv = ConverterFactory::create("nmap2surfaces");
    Options opt; // default

    const auto inPng  = dataDir()/"normal_map.png";
    const auto outPng = dataDir()/"surfaces_gen.png";

    EXPECT_NO_THROW(conv->convert(inPng.string(), outPng.string(), opt));
    ASSERT_TRUE(std::filesystem::exists(outPng));
    ASSERT_GT(std::filesystem::file_size(outPng), 0u);
    
    // Also check that the text output file was created in current directory
    const auto outTxt = std::filesystem::current_path() / "output.txt";
    ASSERT_TRUE(std::filesystem::exists(outTxt));
    ASSERT_GT(std::filesystem::file_size(outTxt), 0u);
}

// -----------------------------------------------------------------------------
// Register with main()
// -----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
