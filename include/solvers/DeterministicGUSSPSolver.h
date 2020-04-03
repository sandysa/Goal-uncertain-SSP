#ifndef MDPLIB_DETERMINISTICGUSSPSOLVER_H
#define MDPLIB_DETERMINISTICGUSSPSOLVER_H

#include "../Action.h"
#include "../State.h"

#include "Solver.h"

#include <climits>
#include <cmath>
#include <ctime>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <vector>
namespace mlsolvers
{

const int det_GUSSP_random = 0;
const int det_GUSSP_most_likely = 1;
const int det_GUSSP_closest = 2;


/**
 * Implements a deterministic solver for the GUSSP. It simplifies the problem
 * by considering and planning for one potential goal.
 * The goal selection mechanism (either closest or most likely) is specified in
 * the constructor.
 */
class DeterministicGUSSPSolver : public Solver
{
private:

    /**
     * Indicates the choice of deterministic outcome
     * (e.g. most-likely, closest). If there are ties in
     * most-likely, then ties are resolved in favor of the closest.
     * Similarly, if there are ties in the closest, ties are resolved
     * in favor of the more likely goal.
     * Default is most likely.
     **/
    int choice_ = det_GUSSP_most_likely;

    /*
     * The heuristic to use.
     */
    mlcore::Heuristic* heuristic_ = nullptr;

    /* The problem to solve. */
    mlcore::Problem* problem_ = nullptr;

     mlcore::StateSet visited;

    std::pair<int,int> temp_goal_ ;

   //denotes active potential goals.
    std::vector<std::pair<std::pair<int,int>, double>> pg_active_;

    /* Error tolerance */
    double epsilon_ = 1.0e-6;

    /* Weight for the Bellman backup */
    double weight_ = 1.0;

    /* Time limit for LAO* in milliseconds */
    int timeLimit_ = 1000000;

    /*
     * Expands the BPSG rooted at state s and returns the
     * number of states expanded.
     */
    int expand(mlcore::State* s);

    /* Test if the BPSG rooted at state s has converged */
    double testConvergence(mlcore::State* s);


public:

    DeterministicGUSSPSolver() {}

    /**
     * Creates a deterministic solver with the given choice for determinization
     * and heuristic to use for solving using LAO*.
     *
     * Available options the determinization are:
     *
     *      - mlsolvers::det_GUSSP_most_likely (Most likely potential goal)
     *      - mlsolvers::det_GUSSP_closest (Closest potential goal)
     */
    DeterministicGUSSPSolver(mlcore::Problem* problem, int choice,
                            mlcore::Heuristic* heuristic = nullptr):
                            problem_(problem), choice_(choice), heuristic_(heuristic){}

    virtual ~DeterministicGUSSPSolver() {}

    virtual void setTempGoal(mlcore::State* s);

    virtual bool tempGoal(mlcore::State* s);

    virtual double getPGDistance(mlcore::State* s, std::pair<int,int> pg);
    /**
     * Solves the associated problem using the LAO* algorithm.
     *
     * @param s0 The state to start the search at.
     */
    virtual mlcore::Action* solve(mlcore::State* s0);

    virtual double GUSSP_qvalue(mlcore::Problem* problem, mlcore::State* s, mlcore::Action* a);

    virtual std::pair<double, mlcore::Action*> GUSSP_bellmanBackup(mlcore::Problem* problem,
                                                 mlcore::State* s);

    virtual double GUSSP_bellmanUpdate(mlcore::Problem* problem, mlcore::State* s);

    virtual mlcore::Action* GUSSP_greedyAction(mlcore::Problem* problem, mlcore::State* s);


};

}
#endif // MDPLIB_DETERMINISTICGUSSPSOLVER_H
