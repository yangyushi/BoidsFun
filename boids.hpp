#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <string>
#include <cstdlib>


#ifndef _BOIS_HPP
#define _BOIS_HPP

using namespace std;
using Vec3 = array<double, 3>;

class Agent {
    public:
        int id;
        Vec3 r;
        Vec3 v;
    friend class System;
    Agent(int id, Vec3 positions, Vec3 velocity);
    Agent(int id, double size);  // distribute randomly in a cubic box with zero speed

};

class System {
    // system with PBC
    public:
        System(int number, double avoid_radius, double size, bool pbc);
        void move(); // move one frame
        void dump(string filename); // save all positions and velocities as xyz file
        vector<Agent> agents;

    private:
        double n;
        double r; // avoid radius
        double size;
        bool pbc;
        Vec3 get_v_attract(Agent& ai); // individual flying to centre of mass of the group
        Vec3 get_v_avoid(Agent& ai);
        Vec3 get_v_align(Agent& ai);
};

#endif
