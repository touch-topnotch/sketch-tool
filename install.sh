#!/bin/bash

# =============================================================================
# Sketch2CAD Installation Script
# =============================================================================
# This script installs all dependencies required for the Sketch2CAD project.
# 
# Prerequisites:
#   - macOS/Linux with bash
#   - Git
#   - Node.js (v16+) and npm
#   - C++ compiler (gcc/clang)
#   - CMake (v3.10+)
#   - pkg-config
#
# Dependencies installed:
#   - OpenCV 4.x
#   - Google Test
#   - Clipper2
#   - svg2roughjs (submodule)
# =============================================================================

set -euo pipefail  # Exit on error, undefined vars, pipe failures

# Colors for output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m' # No Color

# Script configuration
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
readonly LOG_FILE="$SCRIPT_DIR/install.log"

# Dependency versions
readonly OPENCV_VERSION="4.9.0"
readonly GTEST_VERSION="1.14.0"
readonly NODE_VERSION="18"
readonly EMBREE_VERSION="4.4.0"
readonly TBB_VERSION="2021.12.0"
readonly METAL_VERSION="3.0"

# Function to log messages
log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo -e "${timestamp} [${level}] ${message}" | tee -a "$LOG_FILE"
}

info() { log "INFO" "${BLUE}$*${NC}"; }
success() { log "SUCCESS" "${GREEN}$*${NC}"; }
warning() { log "WARNING" "${YELLOW}$*${NC}"; }
error() { log "ERROR" "${RED}$*${NC}"; }

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check OS
get_os() {
    case "$(uname -s)" in
        Darwin*) echo "macos";;
        Linux*) echo "linux";;
        CYGWIN*|MINGW*|MSYS*) echo "windows";;
        *) echo "unknown";;
    esac
}

# Function to check architecture
get_arch() {
    case "$(uname -m)" in
        x86_64) echo "x64";;
        arm64|aarch64) echo "arm64";;
        *) echo "unknown";;
    esac
}

# Function to print banner
print_banner() {
    cat << 'EOF'
╔═════════════════════════════════════════════════════════════╗
║                    Sketch2CAD Installer                     ║
║                                                             ║
║  A professional tool for converting 3D models to sketches   ║
║                                                             ║
║  This script will install all required dependencies:        ║
║  • OpenCV 4.x (Computer Vision)                             ║
║  • Google Test (Testing Framework)                          ║
║  • Clipper2 (Geometry Processing)                           ║
║  • Node.js dependencies (svg2roughjs)                       ║
║  • C++ build tools                                          ║
║  • Embree 4.x (Ray Tracing Acceleration)                    ║
║  • TBB (Threading Building Blocks)                          ║
║  • OpenMP (Parallel Processing)                             ║
║  • Metal Framework (GPU Acceleration)                       ║
╚═════════════════════════════════════════════════════════════╝
EOF
}

# Function to check prerequisites
check_prerequisites() {
    info "Checking prerequisites..."
    
    local missing_deps=()
    
    # Check essential tools
    if ! command_exists git; then
        missing_deps+=("git")
    fi
    
    if ! command_exists cmake; then
        missing_deps+=("cmake")
    fi
    
    if ! command_exists pkg-config; then
        missing_deps+=("pkg-config")
    fi
    
    # Check C++ compiler
    if ! command_exists g++ && ! command_exists clang++; then
        missing_deps+=("C++ compiler (g++ or clang++)")
    fi
    
    # Check Node.js
    if ! command_exists node; then
        missing_deps+=("Node.js")
    else
        local node_version=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
        if [ "$node_version" -lt 16 ]; then
            missing_deps+=("Node.js v16+ (current: v$(node --version))")
        fi
    fi
    
    # Check npm
    if ! command_exists npm; then
        missing_deps+=("npm")
    fi
    
    # Check Homebrew on macOS
    if [ "$(get_os)" = "macos" ] && ! command_exists brew; then
        missing_deps+=("Homebrew (for macOS optimizations)")
    fi
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        error "Missing prerequisites:"
        printf '%s\n' "${missing_deps[@]}" | sed 's/^/  - /'
        echo
        error "Please install the missing dependencies and run this script again."
        error "For macOS users: brew install git cmake pkg-config node opencv googletest"
        error "For macOS optimizations: brew install embree tbb libomp"
        error "For Ubuntu/Debian: sudo apt-get install git cmake pkg-config nodejs npm"
        exit 1
    fi
    
    success "All prerequisites are satisfied!"
}

# Function to install dependencies based on OS
install_system_dependencies() {
    local os=$(get_os)
    
    info "Installing system dependencies for $os..."
    
    case "$os" in
        "macos")
                    if command_exists brew; then
            info "Installing dependencies via Homebrew..."
            brew update
            
            # Install basic dependencies
            brew install opencv googletest
            
            # Install optimization dependencies for macOS
            info "Installing optimization dependencies..."
            brew install embree tbb libomp
            
                    # Check if we're on Apple Silicon and install additional optimizations
        if [ "$(get_arch)" = "arm64" ]; then
            info "Apple Silicon detected - installing additional optimizations..."
            brew install llvm  # For better compiler support
            
            # Install Metal development tools
            info "Installing Metal development tools..."
            xcode-select --install 2>/dev/null || true
            
            # Check if Metal is available
            if [ -d "/System/Library/Frameworks/Metal.framework" ]; then
                success "Metal framework found"
            else
                warning "Metal framework not found - GPU acceleration will be disabled"
            fi
        fi
            
            success "System dependencies installed via Homebrew"
        else
            warning "Homebrew not found. Please install dependencies manually:"
            warning "  brew install opencv googletest embree tbb libomp"
            warning "Continuing with manual installation..."
        fi
            ;;
        "linux")
            if command_exists apt-get; then
                info "Installing dependencies via apt-get..."
                sudo apt-get update
                sudo apt-get install -y libopencv-dev libgtest-dev
                success "System dependencies installed via apt-get"
            elif command_exists yum; then
                info "Installing dependencies via yum..."
                sudo yum install -y opencv-devel gtest-devel
                success "System dependencies installed via yum"
            else
                warning "No supported package manager found. Please install OpenCV and Google Test manually."
            fi
            ;;
        *)
            warning "Unsupported OS: $os"
            warning "Please install OpenCV and Google Test manually."
            ;;
    esac
}

# Function to verify OpenCV installation
verify_opencv() {
    info "Verifying OpenCV installation..."
    
    if pkg-config --exists opencv4; then
        local opencv_version=$(pkg-config --modversion opencv4)
        success "OpenCV $opencv_version found"
        return 0
    elif pkg-config --exists opencv; then
        local opencv_version=$(pkg-config --modversion opencv)
        success "OpenCV $opencv_version found"
        return 0
    else
        error "OpenCV not found via pkg-config"
        return 1
    fi
}

# Function to verify Google Test installation
verify_gtest() {
    info "Verifying Google Test installation..."
    
    if pkg-config --exists gtest; then
        local gtest_version=$(pkg-config --modversion gtest)
        success "Google Test $gtest_version found"
        return 0
    elif pkg-config --exists googletest; then
        local gtest_version=$(pkg-config --modversion googletest)
        success "Google Test $gtest_version found"
        return 0
    else
        warning "Google Test not found via pkg-config, will use bundled version"
        return 1
    fi
}

# Function to verify optimization libraries installation
verify_optimization_libs() {
    local os=$(get_os)
    
    if [ "$os" = "macos" ]; then
        info "Verifying optimization libraries installation..."
        
        # Check Embree
        if pkg-config --exists embree4; then
            local embree_version=$(pkg-config --modversion embree4)
            success "Embree $embree_version found"
        elif [ -f "/opt/homebrew/Cellar/embree/4.4.0/include/embree4/rtcore.h" ]; then
            success "Embree 4.4.0 found in Homebrew"
        else
            warning "Embree not found via pkg-config or Homebrew"
        fi
        
        # Check TBB
        if pkg-config --exists tbb; then
            local tbb_version=$(pkg-config --modversion tbb)
            success "TBB $tbb_version found"
        else
            warning "TBB not found via pkg-config"
        fi
        
        # Check OpenMP
        if command_exists ompi_info || pkg-config --exists ompi; then
            success "OpenMP found"
        else
            warning "OpenMP not found - parallel processing may be limited"
        fi
        
        # Check Metal framework
        if [ -d "/System/Library/Frameworks/Metal.framework" ]; then
            success "Metal framework found"
        else
            warning "Metal framework not found - GPU acceleration will be disabled"
        fi
        
        # Check Metal headers
        if [ -f "/System/Library/Frameworks/Metal.framework/Headers/Metal.h" ]; then
            success "Metal headers found"
        else
            warning "Metal headers not found - GPU acceleration will be disabled"
        fi
        
        # Check if we can compile with optimizations
        info "Testing optimization compilation..."
        local test_file="/tmp/opt_test.cpp"
        cat > "$test_file" << 'EOF'
#include <iostream>
#ifdef __APPLE__
#include <embree4/rtcore.h>
#endif
#ifdef _OPENMP
#include <omp.h>
#endif
#ifdef __APPLE__
#include <Metal/Metal.h>
#endif
int main() {
#ifdef __APPLE__
    std::cout << "Embree available" << std::endl;
    std::cout << "Metal available" << std::endl;
#endif
#ifdef _OPENMP
    std::cout << "OpenMP available with " << omp_get_max_threads() << " threads" << std::endl;
#endif
    return 0;
}
EOF
        
        if [ "$(get_arch)" = "arm64" ]; then
            # Apple Silicon
            if clang++ -std=c++20 -mcpu=apple-m2 -Xpreprocessor -fopenmp -lomp -I/opt/homebrew/include -L/opt/homebrew/lib -lembree4 -ltbb -framework Metal -framework MetalKit -framework Foundation "$test_file" -o /tmp/opt_test 2>/dev/null; then
                success "Optimization compilation test passed for Apple Silicon"
                /tmp/opt_test
            else
                warning "Optimization compilation test failed for Apple Silicon"
            fi
        else
            # Intel Mac
            if clang++ -std=c++20 -Xpreprocessor -fopenmp -lomp -I/usr/local/include -L/usr/local/lib -lembree4 -ltbb -framework Metal -framework MetalKit -framework Foundation "$test_file" -o /tmp/opt_test 2>/dev/null; then
                success "Optimization compilation test passed for Intel Mac"
                /tmp/opt_test
            else
                warning "Optimization compilation test failed for Intel Mac"
            fi
        fi
        
        rm -f "$test_file" /tmp/opt_test
    fi
}

# Function to initialize and update git submodules
setup_git_submodules() {
    info "Setting up Git submodules..."
    
    if [ ! -d ".git" ]; then
        error "Not a Git repository. Please clone the repository first."
        exit 1
    fi
    
    git submodule update --init --recursive
    success "Git submodules initialized and updated"
}

# Function to install Metal development tools
install_metal_tools() {
    local os=$(get_os)
    
    if [ "$os" = "macos" ]; then
        info "Installing Metal development tools..."
        
        # Check if Xcode Command Line Tools are installed
        if ! xcode-select -p >/dev/null 2>&1; then
            info "Installing Xcode Command Line Tools..."
            xcode-select --install
            info "Please complete the Xcode Command Line Tools installation and run this script again."
            exit 0
        fi
        
        # Check if Metal framework is available
        if [ -d "/System/Library/Frameworks/Metal.framework" ]; then
            success "Metal framework is available"
        else
            warning "Metal framework not found - GPU acceleration will be disabled"
        fi
        
        # Check if Metal headers are available
        if [ -f "/System/Library/Frameworks/Metal.framework/Headers/Metal.h" ]; then
            success "Metal headers are available"
        else
            warning "Metal headers not found - GPU acceleration will be disabled"
        fi
    fi
}

# Function to install Node.js dependencies
install_node_dependencies() {
    info "Installing Node.js dependencies..."
    
    # Install svg2roughjs dependencies
    if [ -d "external/svg2roughjs" ]; then
        info "Installing svg2roughjs dependencies..."
        cd external/svg2roughjs
        npm install
        cd "$SCRIPT_DIR"
        
        # Install nodejs-cli dependencies
        if [ -d "external/svg2roughjs/nodejs-cli" ]; then
            info "Installing nodejs-cli dependencies..."
            cd external/svg2roughjs/nodejs-cli
            npm install
            cd "$SCRIPT_DIR"
        fi
        
        success "Node.js dependencies installed"
    else
        error "svg2roughjs submodule not found"
        exit 1
    fi
}

# Function to build the project
build_project() {
    info "Building the project..."
    
    # Clean previous builds
    if [ -d "build" ]; then
        info "Cleaning previous build..."
        rm -rf build
    fi
    
    if [ -d "bin" ]; then
        info "Cleaning previous binaries..."
        rm -rf bin
    fi
    
    # Build using Makefile
    info "Building with Makefile..."
    make clean
    make
    
    # Build optimized version on macOS
    local os=$(get_os)
    if [ "$os" = "macos" ]; then
        info "Building optimized version for macOS..."
        
        # Create optimized build directory
        mkdir -p build_optimized
        cd build_optimized
        
        # Configure with CMake for optimized build
        if [ "$(get_arch)" = "arm64" ]; then
            # Apple Silicon
            cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-mcpu=apple-m2 -O3"
        else
            # Intel Mac
            cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3"
        fi
        
        # Build optimized version
        make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
        
        # Copy optimized binaries
        if [ -f "converter_cli_optimized" ]; then
            cp converter_cli_optimized ../bin/
            success "Optimized version built successfully"
        fi
        
        cd ..
    fi
    
    success "Project built successfully!"
}

# Function to run tests
run_tests() {
    info "Running tests..."
    
    if [ -f "bin/converter_tests" ]; then
        if ./bin/converter_tests --gtest_brief=1; then
            success "All tests passed!"
        else
            warning "Some tests failed. Check the output above for details."
        fi
    else
        error "Test executable not found. Build may have failed."
        exit 1
    fi
}

# Function to test converter functionality
test_converter_functionality() {
    info "Testing converter functionality..."
    
    # Create a simple test OBJ file if it doesn't exist
    if [ ! -f "input.obj" ]; then
        cat > input.obj << 'EOF'
# Simple cube
v -1 -1 -1
v  1 -1 -1
v  1  1 -1
v -1  1 -1
v -1 -1  1
v  1 -1  1
v  1  1  1
v -1  1  1
f 1 2 3 4
f 5 8 7 6
f 1 5 6 2
f 2 6 7 3
f 3 7 8 4
f 5 1 4 8
EOF
    fi
    
    # Test basic functionality
    if [ -f "bin/converter_cli" ]; then
        info "Testing basic converter functionality..."
        ./bin/converter_cli input.obj output.png obj2nmap
        ./bin/converter_cli input.obj output_perspective.png obj2nmap --focal 1000 --pos "(10,10,10)"
        ./bin/converter_cli input.obj output_edges.svg visible_edges --focal 800
        success "Converter functionality tests completed!"
        
        # Test optimized version on macOS
        local os=$(get_os)
        if [ "$os" = "macos" ] && [ -f "bin/converter_cli_optimized" ]; then
            info "Testing optimized converter functionality..."
            ./bin/converter_cli_optimized input.obj output_edges_optimized.svg visible_edges --embree --openmp --focal 800
            success "Optimized converter functionality tests completed!"
        fi
        
        # Test GPU-accelerated version on macOS
        if [ "$os" = "macos" ] && [ -f "bin/converter_cli" ]; then
            info "Testing GPU-accelerated converter functionality..."
            ./bin/converter_cli input.obj output_edges_gpu.svg visible_edges --gpu --focal 800
            success "GPU-accelerated converter functionality tests completed!"
        fi
    else
        error "Converter CLI not found. Build may have failed."
        exit 1
    fi
}

# Function to create environment setup script
create_env_script() {
    info "Creating environment setup script..."
    
    cat > setup_env.sh << 'EOF'
#!/bin/bash
# Environment setup script for Sketch2CAD
# Source this script to set up the environment

export SKETCH2CAD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="$SKETCH2CAD_ROOT/bin:$PATH"

echo "Sketch2CAD environment set up!"
echo "Project root: $SKETCH2CAD_ROOT"
echo "Binary path: $SKETCH2CAD_ROOT/bin"
EOF
    
    chmod +x setup_env.sh
    success "Environment setup script created: setup_env.sh"
}

# Function to print usage instructions
print_usage_instructions() {
    cat << 'EOF'

╔══════════════════════════════════════════════════════════════╗
║                    Installation Complete!                    ║
╚══════════════════════════════════════════════════════════════╝

🎉 Sketch2CAD has been successfully installed!

📁 Project Structure:
   • Source code: src/
   • Headers: include/
   • Binaries: bin/
   • Tests: tests/
   • External dependencies: external/

🚀 Quick Start:
   1. Set up environment:
      source setup_env.sh
   
   2. Run the converter:
      ./bin/converter_cli input.obj output.png obj2nmap
   
   3. Run GPU-accelerated edge detection (macOS):
      ./bin/converter_cli input.obj output.svg visible_edges --gpu
   
   4. Run tests:
      make tests

📚 Available Converters:
   • obj2nmap - Convert OBJ to normal map
   • visible_edges - Extract visible edges (CPU/GPU accelerated)
   • svg2roughjs - Convert SVG to rough sketch
   • nmap2surfaces - Extract surfaces from normal map

🔧 Build Commands:
   • make - Build the project
   • make clean - Clean build artifacts
   • make tests - Run all tests

📖 For more information, see README.md

EOF
}

# Function to handle errors
error_handler() {
    local exit_code=$?
    error "Installation failed with exit code $exit_code"
    error "Check the log file: $LOG_FILE"
    exit $exit_code
}

# Main installation function
main() {
    # Set up error handling
    trap error_handler ERR
    
    # Initialize log file
    > "$LOG_FILE"
    
    # Print banner
    print_banner
    
    # Check if running as root
    if [ "$EUID" -eq 0 ]; then
        error "Please do not run this script as root"
        exit 1
    fi
    
    # Check prerequisites
    check_prerequisites
    
    # Install system dependencies
    install_system_dependencies
    
    # Verify installations
    verify_opencv || {
        error "OpenCV installation verification failed"
        exit 1
    }
    
    verify_gtest || {
        warning "Google Test verification failed, continuing with bundled version"
    }
    
    # Verify optimization libraries on macOS
    verify_optimization_libs
    
    # Setup git submodules
    setup_git_submodules
    
    # Install Metal development tools
    install_metal_tools
    
    # Install Node.js dependencies
    install_node_dependencies
    
    # Build the project
    build_project
    
    # Run tests
    run_tests
    
    # Test converter functionality
    test_converter_functionality
    
    # Create environment setup script
    create_env_script
    
    # Print usage instructions
    print_usage_instructions
    
    success "Installation completed successfully!"
    info "Log file: $LOG_FILE"
}

# Run main function
main "$@" 


./bin/converter_cli tests/house.obj edges.svg visible_edges --pos "(10,10,10)" --fov 10 --w 1024 --h 1024

./bin/converter_cli tests/house.obj nm.png obj2nmap --pos "(10,10,10)" --fov 10 --w 1024 --h 1024

./bin/converter_cli edges.svg draft.svg svg2roughjs --roughness 2.5 --bowing 1