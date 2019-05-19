main: main.cpp utils.hpp scene.hpp
	g++ main.cpp -o main -fopenmp -std=c++11


test: main
	./main 800 600 test 50

run: main
	./main 800 600 img 5000

clear:
	rm main
