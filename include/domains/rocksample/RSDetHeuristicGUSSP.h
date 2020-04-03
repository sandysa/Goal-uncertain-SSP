#ifndef MDPLIB_RSDETHEURGUSSP_H
#define MDPLIB_RSDETHEURGUSSP_H

#include "../../util/general.h"

#include "../../Heuristic.h"
#include "../../State.h"

#include "GUSSPRockSampleProblem.h"
#include "GUSSPRockSampleState.h"


class RSDetHeuristicGUSSP : public mlcore::Heuristic
{
private:
    GUSSPRockSampleProblem* problem_;

public:
     RSDetHeuristicGUSSP(GUSSPRockSampleProblem* problem)
    {
        problem_ = problem;
    }

  virtual double cost(const mlcore::State* s){

        GUSSPRockSampleState* rss = (GUSSPRockSampleState*) s;
        std::pair<int,int> pos (rss->x(),rss->y());
        if (rss->x() == -1) // absorbing dummy state
            return 0;
      //check if it has sampled rocks. GUSSPRSgoal
      if (rss->sampledRocks() == 1)
            return 0.0;

        double cost_ = mdplib::dead_end_cost;
        double value  = 0; // all goals have cost 0.
        for (auto const& element : rss->goalPos()) {
            std::pair<int,int> loc = element.first;
            double md = abs (loc.first -  rss->x()) +
                        abs(loc.second - rss->y());

             /** multiply cost by the probability of it not being a goal**/
            double goalCost = std::round((1 - element.second) * (problem_->getactioncost() * md + value));
 //           double goalCost =  (problem_->getactioncost() * md + value);
            if (goalCost < cost_)
                cost_ = goalCost;
            }
 //                                                                      std::cout << rss << " hval= " << cost_ << std::endl;
    return cost_;
    }
};

#endif // MDPLIB_RSDETHEURGUSSP_H

