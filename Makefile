CC=g++
CFLAGS=-std=c++11 -g -I/home/pi/websocketpp-master/ -I/home/pi/src/psmoveapi/include -I/home/pi/src/psmoveapi/src/../external/hidapi/hidapi -I/home/pi/src/psmoveapi/src/../external/glm -I/home/pi/src/psmoveapi/src/../include -I/home/pi/src/psmoveapi/src/utils/../../src -I/src/psmoveapi/external/opencv -I/home/pi/src/psmoveapi/external/SDL2/include
LDFLAGS=-L/home/pi/boost_1_61_0/stage/lib -L/home/pi/src/psmoveapi/build -L/usr/local/lib -Wall -lboost_system -lboost_random -lboost_thread -lpthread -lboost_timer -lboost_chrono -lrt -lpsmoveapi -lpsmoveapi_tracker -lopencv_core -lopencv_highgui -lSDL2 -lGL -lglut -lGLU

SOURCES=$(wildcard ./*.cpp)
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=psmove_send_poll_data

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(EXECUTABLE) $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) -c -o $@ $<

clean:
	rm *.o

