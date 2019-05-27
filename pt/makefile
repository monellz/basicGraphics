main: main.cpp utils.hpp scene.hpp
	g++ main.cpp ray.hpp -o main -fopenmp -std=c++11 -O3


test: main
	./main 800 600 test 50

run: main
	./main 800 600 img 5000

clear:
	rm main
