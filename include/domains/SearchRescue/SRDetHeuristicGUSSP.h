#ifndef MDPLIB_SRDETHEURGUSSP_H
#define MDPLIB_SRDETHEURGUSSP_H

#include <algorithm>
#include <vector>

#include "../../util/general.h"

#include "../../Heuristic.h"
#include "../../State.h"

#include "GUSSPSearchRescueProblem.h"
#include "GUSSPSearchRescueState.h"


class SRDetHeuristicGUSSP : public mlcore::Heuristic
{
private:
    GUSSPSearchRescueProblem* problem_;

public:
     SRDetHeuristicGUSSP(GUSSPSearchRescueProblem* problem)
    {
        problem_ = problem;
    }

  virtual double cost(const mlcore::State* s){
        GUSSPSearchRescueState* srs = (GUSSPSearchRescueState*) s;
        if (srs->x() == -1) // absorbing dummy state
            return 0;
      //check if it saved all victims-- goal condition
      if (srs->victims() == problem_->maxVictims())
            return 0.0;

        double cost_ = mdplib::dead_end_cost;
        double value  = 0; // all goals have cost 0.
        for (auto const& element : srs->goalPos()) {
            std::pair<int,int> loc = element.first;
            double md = abs (loc.first -  srs->x()) +
                        abs(loc.second - srs->y());

             /** multiply cost by the probability of it not being a goal**/
            double goalCost = std::round((1 - element.second) * (problem_->getactioncost() * md + value));
     //       double goalCost = (problem_->getactioncost() * md + value);
            if(element.second == 0) //not a goal
                goalCost = cost_;

            if (goalCost < cost_)
                cost_ = goalCost;
            }
 //   std::cout << srs << " hval = " << cost_ << std::endl;
    return cost_;
    }
};

#endif // MDPLIB_SRDETHEURGUSSP_H

