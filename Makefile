RM = /bin/rm -f
BUILD = build
SRC = source
LIB = libraries
DEBUG = -DDEBUG

CIMG = -I$(LIB)/CImg/ -I/opt/X11/include -L/opt/X11/lib -lX11 -I/usr/local/bin/ -lpthread
EIGEN = -I$(LIB)/Eigen/
GLFW_COMP = -I$(LIB)/GLFW/include
GLFW_LINK = -L$(LIB)/GLFW/lib -lglfw3 -framework Cocoa -framework IOKit -framework CoreVideo
GLEW_COMP = -I$(LIB)/GLEW/include -DGLEW_STATIC
GLEW_LINK = -L$(LIB)/GLEW/lib -lGLEW

ifeq ($(shell sw_vers 2>/dev/null | grep Mac | awk '{ print $$2}'),Mac)
    CC = clang++ -std=c++11 -stdlib=libc++ -O3
	CFLAGS = -g -c -Wall -Wno-deprecated $(EIGEN) $(GLFW_COMP) $(GLEW_COMP) $(DEBUG)
	LFLAGS = -Wall $(GLFW_LINK) $(GLEW_LINK) -framework OpenGL -framework GLUT
else

endif

PRODUCT = IKRight
OBJS = $(BUILD)/IKRight.o $(BUILD)/utilities.o

IKRight: IKRight.o utilities.o
	$(CC) $(LFLAGS) $(OBJS) -o $(PRODUCT)

IKRight.o: $(SRC)/IKRight.cpp
	$(CC) $(CFLAGS) $(SRC)/IKRight.cpp -o $(BUILD)/IKRight.o

utilities.o: $(SRC)/utilities.cpp
	$(CC) $(CFLAGS) $(SRC)/utilities.cpp -o $(BUILD)/utilities.o

clean:
	$(RM) $(BUILD)/*.o $(PRODUCT)