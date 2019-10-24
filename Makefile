CPPFLAGS = -Wall -std=c++14 -fPIC -shared -Ofast -mtune=native -march=native -I.
PYFLAGS = `python3 -m pybind11 --includes` `python3-config --ldflags`

cboids`python3-config --extension-suffix`: cboids.cpp boids.cpp
	g++ -o cboids`python3-config --extension-suffix` cboids.cpp boids.cpp $(CPPFLAGS) $(PYFLAGS)

boids: boids.cpp boids.hpp
	rm -f movie.xyz
	g++ -o boids boids.cpp $(CPPFLAGS)
