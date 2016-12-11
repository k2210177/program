CXX ?= g++
NAVIO = ../Navio2/C++/Navio
INCLUDES = -I ../..

all:
	$(CXX) -std=gnu++11 $(INCLUDES) imuloop.cpp $(NAVIO)/MPU9250.cpp $(NAVIO)/LSM9DS1.cpp $(NAVIO)/gpio.cpp $(NAVIO)/RGBled.cpp $(NAVIO)/PWM.cpp $(NAVIO)/Util.cpp -o imuloop

clean:
	rm imuloop
