#include "../../include/solvers/Solver.h"
#include "../../include/Heuristic.h"
#include "../../include/solvers/DeterministicGUSSPSolver.h"

#include "../../include/domains/rocksample/GUSSPRockSampleProblem.h"
#include "../../include/domains/SearchRescue/GUSSPSearchRescueProblem.h"

#include "../../include/util/general.h"

#include <ctime>
#include <queue>
#include <vector>

namespace mlsolvers
{
//figure out smarter way of doing this. Using this engineered solution due to lack of time for submission.
double DeterministicGUSSPSolver::getPGDistance(mlcore::State* s, std::pair<int,int> pg)
{
   if(problem_->getProblemName() == "GUSSPRockSample"){
        GUSSPRockSampleState* srs  =  static_cast<GUSSPRockSampleState*> (s);
            return (abs(srs->x() - pg.first) + abs(srs->y() - pg.second));
    }

    else if(problem_->getProblemName() == "GUSSPSearchRescue"){
        GUSSPSearchRescueState* srs  =  static_cast<GUSSPSearchRescueState*> (s);
         return (abs(srs->x() - pg.first) + abs(srs->y() - pg.second));
    }
    return 0;
}

void DeterministicGUSSPSolver::setTempGoal(mlcore::State* s)
{
    pg_active_.clear();
    std::vector<std::pair<std::pair<int,int>,double>> gp = s->getGoalBelief();
    for(auto it = gp.begin(); it != gp.end(); ++it){
        std::pair<std::pair<int,int>,double> val = *it;
        if(val.second > 0)
            pg_active_.push_back(*it);

    }
    //picks most likely goal and plans.
    if(choice_ == det_GUSSP_most_likely){
        double max_goal_prob = 0.0;
        for (int i = 0; i < pg_active_.size(); i++){
             std::pair<std::pair<int,int>, double> gp  = pg_active_.at(i);
             if(gp.second > max_goal_prob){
                max_goal_prob =  gp.second;
                temp_goal_ =  gp.first;
             }
        }
    }
    else if(choice_ == det_GUSSP_random){
        int index =  rand() % pg_active_.size();
        std::pair<std::pair<int,int>, double> gp  = pg_active_.at(index);
        temp_goal_ =  gp.first;
    }
    else if(choice_  == det_GUSSP_closest){
        double min_dist = 10000000.0;
         for (int i = 0; i < pg_active_.size(); i++){
             std::pair<std::pair<int,int>, double> gp  = pg_active_.at(i);
             double distance_pg  = getPGDistance(s, gp.first);
             if( distance_pg <  min_dist){
                min_dist =  distance_pg;
                temp_goal_ =  gp.first;
             }
        }
    }
//    std::cout << "Temp goal = " << temp_goal_.first << ", " << temp_goal_.second << std::endl;
}

bool DeterministicGUSSPSolver::tempGoal(mlcore::State* s)
{
    if(pg_active_.size() == 1) // only true goal is remaining
        return problem_->goal(s); //return true goal condition

    if(problem_->getProblemName() == "GUSSPRockSample"){
        GUSSPRockSampleState* srs  =  static_cast<GUSSPRockSampleState*> (s);
        if(temp_goal_.first == srs->x() && temp_goal_.second == srs->y())
                return true;
    }

    else if(problem_->getProblemName() == "GUSSPSearchRescue"){
        GUSSPSearchRescueState* srs  =  static_cast<GUSSPSearchRescueState*> (s);
        GUSSPSearchRescueProblem* srp = static_cast<GUSSPSearchRescueProblem*>(problem_);
       if(temp_goal_.first == srs->x() && temp_goal_.second == srs->y() && srs->victims() == srp->maxVictims())
                return true;
    }

    return false;
}

mlcore::Action* DeterministicGUSSPSolver::solve(mlcore::State* s0){
    setTempGoal(s0);
//    std::cout << " In detGUSSP solver with s0 = " << s0 << std::endl;
    clock_t startTime = clock();
    int totalExpanded = 0;
    int countExpanded = 0;
    double error = mdplib::dead_end_cost;
    while (true) {
        do {
            visited.clear();
            countExpanded = expand(s0);
            totalExpanded += countExpanded;
            if ((0.001 * (clock() - startTime)) /
                    CLOCKS_PER_SEC > timeLimit_)
                              return s0->bestAction();
        } while (countExpanded != 0);

        while (true) {
            if ((0.001 * (clock() - startTime)) /
                    CLOCKS_PER_SEC > timeLimit_)
                         return s0->bestAction();
            visited.clear();
            error = testConvergence(s0);
            if (error < epsilon_)
                         return s0->bestAction();
            if (error > mdplib::dead_end_cost) {
                break;  // BPSG changed, must expand tip nodes again
            }
        }
    }
}

int DeterministicGUSSPSolver::expand(mlcore::State* s)
{
    if (!visited.insert(s).second)  // state was already visited.
        return 0;
    if (s->deadEnd() || tempGoal(s) || problem_->goal(s))
        return 0;

    int cnt = 0;
    if (s->bestAction() == nullptr) {
        // state has not been expanded.
        GUSSP_bellmanUpdate(problem_, s);
        return 1;
    } else {
        mlcore::Action* a = s->bestAction();
        for (mlcore::Successor sccr : problem_->transition(s, a))
            cnt += expand(sccr.su_state);
    }
    GUSSP_bellmanUpdate(problem_, s);
    return cnt;
}

double DeterministicGUSSPSolver::testConvergence(mlcore::State* s)
{
    double error = 0.0;

    if (s->deadEnd() || tempGoal(s) || problem_->goal(s))
        return 0.0;

    if (!visited.insert(s).second)
        return 0.0;

    mlcore::Action* prevAction = s->bestAction();
    if (prevAction == nullptr) {
        // if it reaches this point it hasn't converged yet.
        return mdplib::dead_end_cost + 1;
    } else {
        for (mlcore::Successor sccr : problem_->transition(s, prevAction))
            error =  std::max(error, testConvergence(sccr.su_state));
    }

    error = std::max(error, GUSSP_bellmanUpdate(problem_, s));
    if (prevAction == s->bestAction())
        return error;
    // it hasn't converged because the best action changed.
    return mdplib::dead_end_cost + 1;
}

double DeterministicGUSSPSolver::GUSSP_bellmanUpdate(mlcore::Problem* problem, mlcore::State* s)
{
    std::pair<double, mlcore::Action*> best = GUSSP_bellmanBackup(problem, s);
    double residual = s->cost() - best.bb_cost;
    bellman_mutex.lock();
    s->setCost(best.bb_cost);
    s->setBestAction(best.bb_action);
    bellman_mutex.unlock();
    return fabs(residual);
}

double DeterministicGUSSPSolver::GUSSP_qvalue(mlcore::Problem* problem, mlcore::State* s, mlcore::Action* a)
{
    double qAction = 0.0;
    for (const mlcore::Successor& su : problem->transition(s,a)) {
        qAction += su.su_prob * su.su_state->cost();
    }
    qAction = (qAction * problem->gamma()) + problem->cost(s, a);
     return qAction;
}

std::pair<double, mlcore::Action*> DeterministicGUSSPSolver::GUSSP_bellmanBackup(mlcore::Problem* problem,
                                                 mlcore::State* s)
{
    double bestQ = mdplib::dead_end_cost;
    if(problem->goal(s) || tempGoal(s))
        bestQ = 0.0;

    bool hasAction = false;
    mlcore::Action* bestAction = nullptr;
    for (mlcore::Action* a : problem->actions()) {
        if (!problem->applicable(s, a))
             continue;
        hasAction = true;
        double qAction = std::min(mdplib::dead_end_cost, GUSSP_qvalue(problem, s, a));
        if (qAction <= bestQ) {
            bestQ = qAction;
            bestAction = a;
        }
    }

    if (!hasAction && bestQ >= mdplib::dead_end_cost)
        s->markDeadEnd();


    return std::make_pair(bestQ, bestAction);
}
mlcore::Action* DeterministicGUSSPSolver::GUSSP_greedyAction(mlcore::Problem* problem, mlcore::State* s)
{
    if (s->bestAction() != nullptr)
        return s->bestAction();

    mlcore::Action* bestAction = nullptr;
    double bestQ = mdplib::dead_end_cost;
    bool hasAction = false;
    for (mlcore::Action* a : problem->actions()) {
        if (!problem->applicable(s, a))
      //    if(!isApplicable(s,a,problem))
            continue;
        hasAction = true;
        double qAction = std::min(mdplib::dead_end_cost, GUSSP_qvalue(problem, s, a));
        if (qAction <= bestQ) {
            bestQ = qAction;
            bestAction = a;
        }
    }
    if (!hasAction)
        s->markDeadEnd();

    return bestAction;
}

}
