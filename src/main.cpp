#include "Simulator.h"

int main()
{
    Simulator sim(4, 2, 10000, 1e-4);
    sim.Optimize();
    return 0;
}
