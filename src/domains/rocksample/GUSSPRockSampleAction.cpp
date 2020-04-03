#include "../../../include/domains/rocksample/GUSSPRockSampleAction.h"
#include "../../../include/domains/rocksample/GUSSPRockSampleProblem.h"

std::ostream& GUSSPRockSampleAction::print(std::ostream& os) const
{

    if (dir_ == rocksample::UP)
        os << "up";
    if (dir_ == rocksample::DOWN)
        os << "down";
    if (dir_ == rocksample::LEFT)
        os << "left";
    if (dir_ == rocksample::RIGHT)
        os << "right";
    if (dir_ == rocksample::SAMPLE)
        os << "sample";
    return os;
}
