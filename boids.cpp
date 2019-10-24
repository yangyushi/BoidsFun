#include "boids.hpp"

double norm(Vec3 & vec){
    double n = 0;
    for (int d = 0; d < 3; d++){
        n += vec[d] * vec[d];
    }
    return sqrt(n);
}


void normalise(Vec3 & vec) {
    double n = norm(vec);
    if (abs(n) < 1e-16) {return;} 
    //cout << vec[0] << ", " << vec[1] << ", " << vec[2] << "(" << n << ")" << endl;
    for (int d=0; d < 3; d++){
        vec[d] = vec[d] / n;
    }
}

Agent::Agent(int id, Vec3 positions, Vec3 velocity) :
    id(id), r(positions), v(velocity) {};


Agent::Agent(int id, double size):
    id(id){
        for (int d=0; d<3; d++) this->v[d] = 0;
        for (int d=0; d<3; d++) this->r[d] = double(rand()) / RAND_MAX * size;
    }


System::System(int number, double avoid_radius, double size, bool pbc = false):
    n(number), r(avoid_radius), size(size), pbc(pbc) {
        for (int id=0; id < n; id++){
            this->agents.push_back(Agent(id, this->size));
        }
};


Vec3 System::get_v_attract(Agent& ai){
    Vec3 v_attact {{0, 0, 0}};
    Vec3 com {{0, 0, 0}}; // centre of mass of the group without agent "ai"
    for (auto aj : this->agents){
        if (aj.id != ai.id) {
            for (int d = 0; d < 3; d++){
                com[d] += aj.r[d];
            }
        }
    }
    for (int d = 0; d < 3; d++){
        v_attact[d] = com[d] / (this->n - 1) - ai.r[d];
        v_attact[d] = v_attact[d];  // magical number to prevent agent fly directly to the group centre
    }
    normalise(v_attact);
    return v_attact;
}


Vec3 System::get_v_avoid(Agent& ai){
    Vec3 v_avoid {{0, 0, 0}};
    Vec3 shift {{0, 0, 0}};
    double dist_1d = 0;
    double dist_3d = 0; 
    for (auto & aj : this->agents){
        if (aj.id == ai.id) continue;

        for (int d = 0; d < 3; d++){
            dist_1d = ai.r[d] - aj.r[d];
            if (this->pbc){
                if (dist_1d > this->size / 2) dist_1d -= this->size;
                else if (dist_1d < -this->size / 2) dist_1d += this->size;
            }
            shift[d] = dist_1d;
        }

        for (int d = 0; d < 3; d++) {
            dist_3d += pow(shift[d], 2);
        }
        dist_3d = sqrt(dist_3d);


        if (dist_3d <= 2 * this->r)  {
            for (int d = 0; d < 3; d++){
                v_avoid[d] += shift[d] * (2 * this->r - dist_3d);
            }
        }
    }
    normalise(v_avoid);
    for (int d = 0; d < 3; d++) {v_avoid[d] *= this->r;}
    return v_avoid;
}


Vec3 System::get_v_align(Agent& ai){
    Vec3 v_align {{0, 0, 0}};
    for (auto & aj : this->agents){
        if (aj.id == ai.id) continue;
        for (int d = 0; d < 3; d++){
            v_align[d] += aj.v[d];
        }
    }
    for (int d = 0; d < 3; d++){
        v_align[d] = v_align[d] / (this->n - 1);
    }
    normalise(v_align);
    return v_align;
}


void System::move(){
    Vec3 v_attract;
    Vec3 v_avoid;
    Vec3 v_align;

    for (auto& agent : this->agents) {
        v_attract = this->get_v_attract(agent);
        v_avoid = this->get_v_avoid(agent);
        v_align = this->get_v_align(agent);

        for (int d = 0; d < 3; d++){
            agent.v[d] += v_attract[d] + v_avoid[d] + v_align[d];
        }
        normalise(agent.v);

        for (int d = 0; d < 3; d++){
            agent.r[d] += agent.v[d];
            if (this->pbc) {
                if (agent.r[d] < 0) {
                    agent.r[d] += this->size * (1 + floor(abs(agent.r[d]) / this->size));
                }
                else if (agent.r[d] > this->size) {
                    agent.r[d] -= this->size * floor(abs(agent.r[d]) / this->size);
                }
            }
        }
    }
}


void System::dump(string filename){
    ofstream f;
    f.open(filename, ios::out | ios::app);
    f << this->n << endl;
    f << "id, x, y, z, vx, vy, vz" << endl;
    for (auto a : this->agents) {
        f << a.id << " "
            << a.r[0] << " " <<  a.r[1] << " " << a.r[2] << " "
            << a.v[0] << " " <<  a.v[1] << " " << a.v[2] << endl;
    }
    f.close();
}


int main(){
    int number = 100;
    double avoid_radius = 5.0;
    int box = number * floor(avoid_radius) * 5;
    bool pbc = false;

    System sim(number, avoid_radius, box, pbc);

    int total_frame = 10000;
    for (int frame = 0; frame < total_frame; frame++) {
        sim.move();
    }

    total_frame = 1000;
    for (int frame = 0; frame < total_frame; frame++) {
        sim.move();
        sim.dump("movie_" + to_string(number) + "_" + to_string(avoid_radius) + ".xyz");
    }
    return 0;
}
