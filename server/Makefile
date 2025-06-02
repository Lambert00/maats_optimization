# Compiler settings
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wno-unused-parameter -Wno-deprecated-copy -pthread
CXXFLAGS_REAL = -std=c++17 -O2 -Wall -Wextra -pthread -fPIC

# SNOPT settings
SNOPT_INCLUDE = -I/home/user/optimization-libraries/SNOPT7
SNOPT_LIBS = -L/home/user/optimization-libraries/SNOPT7 -lsnopt7_cpp -lgfortran

# VRPN settings
VRPN_INCLUDE = -I/usr/local/include
VRPN_LIBS = -L/usr/local/lib -lvrpn -lquat

# Eigen settings
EIGEN_INCLUDE = -I/home/user/maats_optimization/server/eigen

MAATS_TARGET = MAATSTensionOptimizer
MAATS_SRC = MAATSTensionOptimizer.cpp

MAATS_OBJ = $(MAATS_SRC:.cpp=.o)

all: maats

maats: $(MAATS_TARGET)

$(MAATS_TARGET): $(MAATS_OBJ)
	$(CXX) $(CXXFLAGS_REAL) $^ -o $@ $(SNOPT_LIBS) $(VRPN_LIBS) -pthread -Wl,-rpath,/home/user/optimization-libraries/SNOPT7

$(MAATS_OBJ): $(MAATS_SRC)
	$(CXX) $(CXXFLAGS_REAL) $(SNOPT_INCLUDE) $(VRPN_INCLUDE) $(EIGEN_INCLUDE) -c $< -o $@

clean:
	rm -f $(MAATS_OBJ) $(MAATS_TARGET)

install: $(MAATS_TARGET)
	mkdir -p $(DESTDIR)/usr/local/bin
	cp $(MAATS_TARGET) $(DESTDIR)/usr/local/bin/

.PHONY: all maats clean install run-maats