###############################################################################
# Raylib AMCL simulator: builds raylib (if missing), builds the sim, and runs
###############################################################################

# C++ compiler (use existing value if already set)
CXX ?= g++

# Simulator source (change if your file name is different)
SIM_SRCS := $(SRCDIR)/amcl_sim.cpp
SIM_TARGET := $(BINDIR)/amcl_sim

# Raylib location (expects you put raylib source under external/raylib)
RAYLIB_DIR := external/raylib
RAYLIB_INC := -I$(RAYLIB_DIR)/src -I$(RAYLIB_DIR)/include
RAYLIB_LIB := $(RAYLIB_DIR)/src/libraylib.a

# macOS frameworks needed by raylib (kept for mac builds)
FRAMEWORKS := -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo

# Default runtime arguments (image then map)
SIM_ARGS ?= Maps/PBField.png Maps/map.txt

.PHONY: raylib-build raylib-sim run-raylib-sim clean-sim

# ---------------------------------------------------------------------------
# Build raylib static lib if it's missing or out-of-date relative to its source
# ---------------------------------------------------------------------------
$(RAYLIB_LIB):
	@echo ">>> Raylib static lib not found or out-of-date. Building raylib..."
	@$(MAKE) -C $(RAYLIB_DIR)/src PLATFORM=PLATFORM_DESKTOP
	@echo ">>> Raylib build finished."

raylib-build: $(RAYLIB_LIB)

# ---------------------------------------------------------------------------
# Build only (creates $(SIM_TARGET))
# ---------------------------------------------------------------------------
raylib-sim: $(SIM_TARGET)
	@echo ">>> Built simulator: $(SIM_TARGET)"
	@echo ">>> To run: make run-raylib-sim or ./$(SIM_TARGET) $(SIM_ARGS)"

# Link step: depends on simulator source and raylib static lib
$(SIM_TARGET): $(SIM_SRCS) $(RAYLIB_LIB)
	@mkdir -p $(BINDIR)
	@echo ">>> Compiling simulator..."
	$(CXX) -std=c++17 -O2 $(SIM_SRCS) -o $(SIM_TARGET) $(RAYLIB_INC) $(RAYLIB_LIB) $(FRAMEWORKS)
	@chmod +x $(SIM_TARGET)

# ---------------------------------------------------------------------------
# Build then run (use this most often)
# ---------------------------------------------------------------------------
run-raylib-sim: $(SIM_TARGET)
	@echo ">>> Running simulator: ./$(SIM_TARGET) $(SIM_ARGS)"
	./$(SIM_TARGET) $(SIM_ARGS)

# ---------------------------------------------------------------------------
# Clean simulator artifacts
# ---------------------------------------------------------------------------
clean-sim:
	@echo ">>> Removing simulator $(SIM_TARGET)"
	rm -f $(SIM_TARGET)
