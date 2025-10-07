CXX = g++
CXXFLAGS = -std=c++14

# Include directories
INCLUDES = \
    -I/usr/include/pcl-1.14 \
    -I/usr/include/eigen3 \
    -I/usr/include/vtk-9.1 \
    -I/usr/include/opencv4

# Libraries
LIBS = \
    `pkg-config --libs opencv4` \
    -lpcl_common -lpcl_io -lpcl_visualization -lpcl_filters \
    -lvtkCommonCore-9.1 -lvtkCommonDataModel-9.1 -lvtkFiltersCore-9.1 \
    -lvtkRenderingCore-9.1 -lvtkRenderingFreeType-9.1 -lvtkRenderingOpenGL2-9.1 \
    -lvtkRenderingVolume-9.1 -lvtkInteractionStyle-9.1 -lvtkIOCore-9.1 \
    -lvtkIOGeometry-9.1 -lvtkIOLegacy-9.1 -lvtkCommonMath-9.1

# Source files
SRC = $(wildcard src/*.cpp)

# Output file
TARGET = main

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(INCLUDES) $(LIBS)

clean:
	rm -f $(TARGET)