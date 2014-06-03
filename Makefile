CC = g++
CFLAGS = -Wall -I/opt/local/include
LDFLAGS = -L/opt/local/lib \
	  -lopencv_core \
	  -lopencv_imgproc \
	  -lopencv_calib3d \
	  -lopencv_highgui

OBJ = main.o

PROG = cam-intrinsics-db

all: $(OBJ)
	$(CC) $(LDFLAGS) -o $(PROG) $(OBJ)
    
main.o: src/main.cpp
	$(CC) $(CFLAGS) -c src/main.cpp

clean:
	rm $(OBJ)
	rm $(PROG)
