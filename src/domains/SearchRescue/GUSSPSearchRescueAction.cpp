#include "../../../include/domains/SearchRescue/GUSSPSearchRescueAction.h"
#include "../../../include/domains/SearchRescue/GUSSPSearchRescueProblem.h"

std::ostream& GUSSPSearchRescueAction::print(std::ostream& os) const
{
    os << "action ";
    if (dir_ == searchrescue::UP)
        os << "up";
    if (dir_ == searchrescue::DOWN)
        os << "down";
    if (dir_ == searchrescue::LEFT)
        os << "left";
    if (dir_ == searchrescue::RIGHT)
        os << "right";
    if (dir_ == searchrescue::SAVE)
        os << "save";
    return os;
}
