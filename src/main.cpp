#include "hackflight/hackflight.hpp"
#include "hackflight/simboard.hpp"

int main()
{
    hf::Hackflight h;
    h.setup(new hf::SimBoard());

    while (true) {

    }
}