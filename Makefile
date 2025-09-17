# =========================================================
# Project Makefile for PROS + Raylib Simulation
# =========================================================

# ---- Project settings ----
PROJECT_NAME = mcl
SIM_NAME = mcl_sim

# ---- Compiler ----
CC = clang

# ---- Directories ----
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
EXTERNAL_DIR = external
RAYLIB_DIR = $(EXTERNAL_DIR)/raylib

# ---- Flags ----
CFLAGS = -I$(RAYLIB_DIR)/src -I$(SRC_DIR) -Wall -std=c99
LDFLAGS = -L$(RAYLIB_DIR)/src -lraylib -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo

# ---- Sources ----
SOURCES = $(wildcard $(SRC_DIR)/*.c)
OBJECTS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SOURCES))
TARGET = $(BIN_DIR)/$(PROJECT_NAME)
SIM_TARGET = $(BIN_DIR)/$(SIM_NAME)

# =========================================================
# Rules
# =========================================================

all: pros

# ---- PROS Build ----
pros:
	pros make

flash:
	pros upload

# ---- Simulation Build ----
sim: $(SIM_TARGET)

$(SIM_TARGET): $(OBJECTS) | $(BIN_DIR) raylib
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# ---- Run simulation ----
run: sim
	./$(SIM_TARGET)

# ---- Build Raylib if not built yet ----
raylib:
	$(MAKE) -C $(RAYLIB_DIR)/src PLATFORM=PLATFORM_DESKTOP

# ---- Cleanup ----
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)
	$(MAKE) -C $(RAYLIB_DIR)/src clean


