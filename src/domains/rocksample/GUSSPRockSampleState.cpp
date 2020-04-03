#include "../../../include/util/general.h"

#include "../../../include/domains/rocksample/GUSSPRockSampleState.h"

GUSSPRockSampleState::GUSSPRockSampleState(
    mlcore::Problem* problem, int x, int y, int sampledRocks, std::vector<std::pair<std::pair<int, int>,double>> goalPos) :
                                                                            x_(x), y_(y), sampledRocks_(sampledRocks), goalPos_(goalPos)
{
     problem_ = problem;
     goalBelief_ =  goalPos_;
      /* Adding a successor entry for each action */
    for (int i = 0; i < 5; i++) {
        allSuccessors_.push_back(std::list<mlcore::Successor> ());
    }
}

std::ostream& GUSSPRockSampleState::print(std::ostream& os) const
{
    os << "RockSample : (" << x_ << "," << y_ << "," << sampledRocks_ << ",";
    for(int i = 0; i < goalPos_.size(); ++i){
        std::pair<std::pair<int, int>,double> pos = goalPos_[i];
        std::pair<int,int> locs = pos.first;
        os << "[" << locs.first << "," << locs.second << "," <<
              pos.second << "], ";
    }
    os << " )";
    return os;
}

bool GUSSPRockSampleState::equals(mlcore::State* other) const
{
    GUSSPRockSampleState* rss = static_cast<GUSSPRockSampleState*> (other);
    return *this == *rss;
}

int GUSSPRockSampleState::hashValue() const
{
     int hashVal = x_ + 31 * (y_ + 31 * sampledRocks_);
     for(int i = 0; i < goalPos_.size(); ++i){
        std::pair<std::pair<int, int>,double> pos = goalPos_[i];
        std::pair<int,int> locs = pos.first;
        hashVal += 31 * (locs.first + 31  * locs.second);
    }
    return hashVal;
}
