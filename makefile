all: a.exe

flags=-g -Wall -Wextra -std=gnu17
incs=/I "glfw-3.3.7.bin.WIN64/include/" \
	/I "C:\Program Files (x86)\Windows Kits\10\Include\10.0.19041.0\ucrt" \
	/I "C:\Program Files (x86)\Windows Kits\10\Include\10.0.19041.0\um" \
	/I "C:\Program Files (x86)\Windows Kits\10\Include\10.0.19041.0\shared" 

libs="glfw-3.3.7.bin.WIN64/lib-vc2019/glfw3_mt.lib" "C:\Program Files (x86)\Windows Kits\10\Lib\10.0.19041.0\ucrt\x64"

LIBDIR="C:/Program Files (x86)/Windows Kits/10/Lib/10.0.19041.0/um/x64/"

#build with clang
# a.exe: main.c plot_surface.h makefile
# 	clang $(flags) -I$(incs) $(libs) -L"$(LIBDIR)" main.c -o a.exe

#build with msvc
a.exe: main.c plot_surface.h makefile
	cl $(incs) $(libs)  main.c /link /out:a.exe /libpath:"$(LIBDIR)"


run: a.exe
	./a.exe 
	
clean:
	rm *.lib *.exe *.obj *.pdb *.exp *.ilk


