CXX ?= g++
NAVIO = ../../Navio2/C++/Navio
INCLUDES = -I ../..

all:
	$(CXX) -std=gnu++11 $(INCLUDES) accelerometer-calibration.cpp $(NAVIO)/MPU9250.cpp $(NAVIO)/LSM9DS1.cpp $(NAVIO)/gpio.cpp -o accelerometer-calibration

clean:
	rm accelertometer-calibration
