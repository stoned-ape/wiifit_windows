all: a.out



a.out: main.cpp kinect_struct.cpp kinect_struct.h makefile
	clang++ -std=c++20 \
	`pkg-config --cflags --libs glm` \
	`pkg-config --cflags --libs glfw3` \
	-framework OpenGL \
	main.cpp kinect_struct.cpp



run: a.out
	./a.out 
	


