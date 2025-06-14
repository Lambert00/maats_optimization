PROJ_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

# ───── Library locations (defaults) ────────────────────────────────────────────────
SNOPT_ROOT ?= $(HOME)/SNOPT7
VRPN_ROOT  ?= /usr/local
EIGEN_ROOT ?= $(PROJ_ROOT)/eigen

# ───── Compiler & flags ────────────────────────────────────────────────────────────
CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pthread -fPIC

SNOPT_INC := -I$(SNOPT_ROOT)
SNOPT_LIB := -L$(SNOPT_ROOT) -lsnopt7_cpp -lgfortran

VRPN_INC  := -I$(VRPN_ROOT)/include
VRPN_LIB  := -L$(VRPN_ROOT)/lib -lvrpn -lquat

EIGEN_INC := -I$(EIGEN_ROOT)

LDFLAGS   := $(SNOPT_LIB) $(VRPN_LIB) -pthread -Wl,-rpath,$(SNOPT_ROOT)

# ───── Sources & target ────────────────────────────────────────────────────────────
TARGET := MAATSTensionOptimizer
TARGET_SIM := MAATSTensionOptimizerSimulator
SRC    := MAATSTensionOptimizer.cpp
SRC_SIM := MAATSTensionOptimizerSimulator.cpp
OBJ    := $(SRC:.cpp=.o)
OBJ_SIM := $(SRC_SIM:.cpp=.o)

# ───── Build rules ────────────────────────────────────────────────────────────────
all: $(TARGET) $(TARGET_SIM)

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $@ $(LDFLAGS)

$(TARGET_SIM): $(OBJ_SIM)
	$(CXX) $(OBJ_SIM) -o $@ $(LDFLAGS)

$(OBJ): $(SRC)
	$(CXX) $(CXXFLAGS) $(SNOPT_INC) $(VRPN_INC) $(EIGEN_INC) -c $< -o $@

$(OBJ_SIM): $(SRC_SIM)
	$(CXX) $(CXXFLAGS) $(SNOPT_INC) $(VRPN_INC) $(EIGEN_INC) -c $< -o $@

clean:
	rm -f $(OBJ) $(OBJ_SIM) $(TARGET) $(TARGET_SIM)

install: $(TARGET)
	install -Dm755 $(TARGET) $(DESTDIR)/usr/local/bin/$(TARGET)

.PHONY: all clean install
