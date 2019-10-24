#include "boids.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

py::array_t<double> simulate_boids(int agent_number, double avoid_radius, int frames, int frames_eq=10000){
    int box = agent_number * floor(avoid_radius) * 5;
    System sim(agent_number, avoid_radius, box, false);

    auto result = py::array_t<double>(agent_number * frames * 6);
    auto buffer = result.request();
    double *ptr = (double *) buffer.ptr;
    int agent_idx = 0;

    for (int f = 0; f < frames_eq; f++) {
        sim.move();
    }

    for (int f = 0; f < frames; f++) {
        sim.move();
        agent_idx = 0;
        for (auto agent : sim.agents) {
            for (int d = 0; d < 3; d++){
                *ptr++ = agent.r[d];
            }
            for (int d = 0; d < 3; d++){
                *ptr++ = agent.v[d];
            }
            agent_idx++;
        }
    }
    result.resize({frames, agent_number, 6});
    return result;
}


PYBIND11_MODULE(cboids, m){
    m.doc() = "simulate boids model";
    m.def("simulate_boids", &simulate_boids, "boids model simulation");
}
