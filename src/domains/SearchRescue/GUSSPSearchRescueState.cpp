#include "../../../include/util/general.h"

#include "../../../include/domains/SearchRescue/GUSSPSearchRescueState.h"

GUSSPSearchRescueState::GUSSPSearchRescueState(
    mlcore::Problem* problem, int x, int y, int victims, std::vector<std::pair<std::pair<int, int>,double>> goalPos) :
                                    x_(x), y_(y), victims_(victims),goalPos_(goalPos)
{
     problem_ = problem;
     goalBelief_ =  goalPos_;
         /* Adding a successor entry for each action */
    for (int i = 0; i < 5; i++) {
        allSuccessors_.push_back(std::list<mlcore::Successor> ());
    }
}

std::ostream& GUSSPSearchRescueState::print(std::ostream& os) const
{
    os << "GUSSPSearchRescue : (" << x_ << "," << y_ << "," << victims_ << ",";
    for(int i = 0; i < goalPos_.size(); ++i){
        std::pair<std::pair<int, int>,double> pos = goalPos_[i];
        std::pair<int,int> locs = pos.first;
        os << "[" << locs.first << "," << locs.second << "," <<
              pos.second << "], ";
    }
    os << " )";
    return os;
}

bool GUSSPSearchRescueState::equals(mlcore::State* other) const
{
    GUSSPSearchRescueState* srs = static_cast<GUSSPSearchRescueState*> (other);
    return *this == *srs;
}

int GUSSPSearchRescueState::hashValue() const
{
    int hashVal = x_ + 31 * (y_ + 31 * victims_);
     for(int i = 0; i < goalPos_.size(); ++i){
        std::pair<std::pair<int, int>,double> pos = goalPos_[i];
        std::pair<int,int> locs = pos.first;
        hashVal += 31 * (locs.first + 31  * locs.second);
    }
    return hashVal;
}
