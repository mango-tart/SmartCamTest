CC = g++
CFLAGS = -Iinclude -I./mnn/include -I/usr/local/include -I/usr/include/jsoncpp -std=c++17
LDFLAGS += -lwiringPi -lpthread -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lcurl -ljsoncpp -L./mnn/lib -lMNN /usr/local/lib/libdrogon.a /usr/local/lib/libtrantor.a -lssl -lcrypto -luuid
RPATH = -Wl,-rpath,./mnn/lib

OPENCV_INCLUDE = -I/usr/include/opencv4
OPENCV_LIB = -L/usr/lib

main: LDFLAGS += -lz
main: src/main.cpp src/MotorController.cpp src/UltraFace.cpp
	$(CC) $(CFLAGS) $(OPENCV_INCLUDE) -o main src/main.cpp src/MotorController.cpp src/UltraFace.cpp $(LDFLAGS) $(OPENCV_LIB) $(RPATH)

clean:
	rm -f main
