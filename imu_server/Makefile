# Make the IMU Server
CC=g++
CXXFLAGS=-Wall -g -std=c++0x
LFLAGS=-lpthread -std=c++0x

# Objects
OBJECTS=serialstream.o socket.o serversocket.o imu_server.o

# All
all: $(OBJECTS) serialstream.h socket.hpp serversocket.hpp
	$(CC) $(OBJECTS) $(LFLAGS) -o imu_server

clean:
	rm -rf $(OBJECTS) imu_server
