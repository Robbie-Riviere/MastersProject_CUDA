all: main ca
	
main:
	g++ -Isrc/include -Lsrc/lib -LCStuff -o main main.cpp -lmingw32 -lSDL2main -lSDL2 -lSDL2_image -lSDL2_ttf

ca:
	g++ -Isrc -o ca ca.cpp 
	
debug: main_debug ca_debug

main_debug:
	g++ -g -Isrc/include -Lsrc/lib -LCstuff -o main_debug main.cpp -lmingw32 -lSDL2main -lSDL2 -lSDL2_image -lSDL2_ttf

ca_debug:
	g++ -g -Isrc -o ca_debug ca.cpp 

clean:
	rm -f main.o main main.exe ca main_debug ca_debug ca_debug.exe ca.exe 

run: all
	./ca
	./main

run_debug: debug
	./ca_debug
	./main_debug