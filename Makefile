
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
AR        := ar
RM        := rm -rf
MKDIR_P   := mkdir -p

# ---------- флаги компилятора ---------------------------------
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra               \
            -Iinclude                                  \
            -Iinclude/converter_lib                    \
            -Iexternal                                 \
            -IClipper2/CPP/Clipper2Lib/include

# ---------- внешние библиотеки через pkg-config --------------
GTEST_CFLAGS  := $(shell pkg-config --cflags gtest 2>/dev/null)
GTEST_LIBS    := $(shell pkg-config --libs   gtest 2>/dev/null)

OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs   opencv4 2>/dev/null)

CXXFLAGS += $(OPENCV_CFLAGS)
LDFLAGS  := $(OPENCV_LIBS)

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
MAIN_OBJ := $(patsubst %.cpp, build/%.o, $(MAIN_SRC))
TEST_OBJ := $(patsubst %.cpp, build/%.o, $(TEST_SRC))

# ---------- выходные артефакты --------------------------------
BIN_DIR   := bin
EXEC      := $(BIN_DIR)/converter_cli
TEST_EXE  := $(BIN_DIR)/converter_tests
LIB_OUT   := build/libconverter.a

# ---------- vpath, чтобы make находил .cpp где бы они ни были -
vpath %.cpp src
vpath %.cpp Clipper2/CPP/Clipper2Lib/src
vpath %.cpp tests

# ---------- цели по умолчанию --------------------------------
.PHONY: all tests clean
all: $(EXEC)

# ---------- правило сборки .o (универсальное) -----------------
build/%.o: %.cpp
	@$(MKDIR_P) $(dir $@)
	$(CXX) $(CXXFLAGS) $(GTEST_CFLAGS) -c $< -o $@

# ---------- библиотека ----------------------------------------
$(LIB_OUT): $(LIB_OBJ)
	@$(MKDIR_P) $(dir $@)
	$(AR) rcs $@ $^

# ---------- executable ----------------------------------------
$(EXEC): $(LIB_OUT) $(MAIN_OBJ)
	@$(MKDIR_P) $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# ---------- тестовый бинарь -----------------------------------
$(TEST_EXE): $(LIB_OUT) $(TEST_OBJ)
	@$(MKDIR_P) $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(GTEST_LIBS) -pthread $(LDFLAGS)

# ---------- запуск всех тестов --------------------------------
tests: $(TEST_EXE)
	./$(TEST_EXE)

# ---------- очистка -------------------------------------------
clean:
	$(RM) build $(BIN_DIR)