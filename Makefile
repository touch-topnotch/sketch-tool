
# ==============================================================
#  Makefile  —  sketch2cad
#  cross-platform (Linux/macOS) build:
#    • libconverter.a          — статическая библиотека
#    • bin/converter_cli       — CLI
#    • bin/converter_tests     — Google-testы (если gtest установлен)
#
#  Требования: g++17, pkg-config, (опц.) gtest, (опц.) opencv4.
# ==============================================================

# ---------- инструменты ---------------------------------------
CXX       := g++
OBJCXX    := g++
AR        := ar
RM        := rm -rf
MKDIR_P   := mkdir -p

# ---------- флаги компилятора ---------------------------------
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra               \
            -Iinclude                                  \
            -Iinclude/converter_lib                    \
            -Iexternal                                 \
            -IClipper2/CPP/Clipper2Lib/include

# ---------- macOS оптимизации ---------------------------------
ifeq ($(shell uname -s),Darwin)
    # Apple Silicon optimizations
    ifeq ($(shell uname -m),arm64)
        CXXFLAGS += -mcpu=apple-m2
    endif
    
    # Metal framework for GPU acceleration (link only)
    METAL_FRAMEWORKS := -framework Metal -framework Foundation
    
    # Embree paths
    EMBREE_INCLUDE := /opt/homebrew/Cellar/embree/4.4.0/include
    EMBREE_LIB := /opt/homebrew/Cellar/embree/4.4.0/lib
    
    # TBB paths
    TBB_INCLUDE := /opt/homebrew/Cellar/tbb/2022.2.0/include
    TBB_LIB := /opt/homebrew/Cellar/tbb/2022.2.0/lib
    
    # OpenMP paths (Homebrew libomp 21.1.0)
    OPENMP_INCLUDE := /opt/homebrew/Cellar/libomp/21.1.0/include
    OPENMP_LIB := /opt/homebrew/Cellar/libomp/21.1.0/lib
    
    CXXFLAGS += -I$(EMBREE_INCLUDE) -I$(TBB_INCLUDE) -I$(OPENMP_INCLUDE) -DEMBREE_STATIC_LIB
    LDFLAGS += -L$(EMBREE_LIB) -L$(TBB_LIB) -L$(OPENMP_LIB) -lembree4 -ltbb -lomp -lpthread
endif

# ---------- внешние библиотеки через pkg-config --------------
GTEST_CFLAGS  := $(shell pkg-config --cflags gtest 2>/dev/null)
GTEST_LIBS    := $(shell pkg-config --libs   gtest 2>/dev/null)

# OpenCV paths for Homebrew
OPENCV_INCLUDE := /opt/homebrew/opt/opencv/include/opencv4
OPENCV_LIB     := /opt/homebrew/opt/opencv/lib

CXXFLAGS += -I$(OPENCV_INCLUDE)
LDFLAGS  := -L$(OPENCV_LIB) -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui

# ---------- исходники -----------------------------------------
# собственные *.cpp (кроме main)
LIB_SRC := $(filter-out src/main.cpp, $(wildcard src/*.cpp))
# исходники Clipper2
LIB_SRC += $(wildcard Clipper2/CPP/Clipper2Lib/src/*.cpp)

# главная точка входа
MAIN_SRC := src/main.cpp

# файлы тестов
TEST_SRC := $(wildcard tests/*.cpp)

# ---------- объектные файлы -----------------------------------
LIB_OBJ  := $(patsubst %.cpp, build/%.o, $(LIB_SRC))
LIB_OBJ  += $(patsubst %.mm, build/%.o, $(wildcard src/*.mm))
MAIN_OBJ := $(patsubst %.cpp, build/%.o, $(MAIN_SRC))
TEST_OBJ := $(patsubst %.cpp, build/%.o, $(TEST_SRC))

# ---------- выходные артефакты --------------------------------
BIN_DIR   := bin
EXEC      := $(BIN_DIR)/converter_cli
TEST_EXE  := $(BIN_DIR)/converter_tests
LIB_OUT   := build/libconverter.a

# ---------- vpath, чтобы make находил .cpp и .mm где бы они ни были -
vpath %.cpp src
vpath %.mm src
vpath %.cpp Clipper2/CPP/Clipper2Lib/src
vpath %.cpp tests

# ---------- цели по умолчанию --------------------------------
.PHONY: all tests clean
all: $(EXEC)

# ---------- правило сборки .o (универсальное) -----------------
build/%.o: %.cpp
	@$(MKDIR_P) $(dir $@)
	$(CXX) $(CXXFLAGS) $(GTEST_CFLAGS) -c $< -o $@

build/%.o: %.mm
	@$(MKDIR_P) $(dir $@)
	$(OBJCXX) $(CXXFLAGS) $(GTEST_CFLAGS) -c $< -o $@

# ---------- библиотека ----------------------------------------
$(LIB_OUT): $(LIB_OBJ)
	@$(MKDIR_P) $(dir $@)
	$(AR) rcs $@ $^

# ---------- executable ----------------------------------------
$(EXEC): $(LIB_OUT) $(MAIN_OBJ)
	@$(MKDIR_P) $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(METAL_FRAMEWORKS) -L/opt/homebrew/Cellar/embree/4.4.0/lib -L/opt/homebrew/Cellar/tbb/2022.2.0/lib -L/opt/homebrew/Cellar/libomp/21.1.0/lib -lembree4 -ltbb -lomp

# ---------- тестовый бинарь -----------------------------------
$(TEST_EXE): $(LIB_OUT) $(TEST_OBJ)
	@$(MKDIR_P) $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(GTEST_LIBS) -pthread $(LDFLAGS) $(METAL_FRAMEWORKS) -L/opt/homebrew/Cellar/embree/4.4.0/lib -L/opt/homebrew/Cellar/tbb/2022.2.0/lib -L/opt/homebrew/Cellar/libomp/21.1.0/lib -lembree4 -ltbb -lomp

# ---------- запуск всех тестов --------------------------------
tests: $(TEST_EXE)
	./$(TEST_EXE)

# ---------- очистка -------------------------------------------
clean:
	$(RM) build $(BIN_DIR)