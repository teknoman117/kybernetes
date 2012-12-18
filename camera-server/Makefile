# Make the IMU Server
CC=g++
CXXFLAGS=-Wall -std=c++0x
LFLAGS=-lpthread -lboost_date_time -lboost_thread -lxmlrpc++ -lxmlrpc_server++ -lxmlrpc_server_abyss++ -std=c++0x

# Objects
OBJECTS=framegrabber_uvc.o node.o

# All
all: $(OBJECTS) framegrabber_uvc.h
	$(CC) $(OBJECTS) $(LFLAGS) -o camera_server
