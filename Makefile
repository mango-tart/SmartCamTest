CC = g++
CFLAGS = -Iinclude
LDFLAGS = -lwiringPi -lpthread
main: src/main.cpp src/MotorController.cpp
	$(CC) $(CFLAGS) -o main src/main.cpp src/MotorController.cpp $(LDFLAGS)
clean:
	rm -f main
