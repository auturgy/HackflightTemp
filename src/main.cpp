#include "firmware/hackflight.hpp"
#include "firmware/simboard.hpp"

int main()
{
    hf::Hackflight h;
    h.init(new hf::SimBoard());

    while (true) {

    }
}