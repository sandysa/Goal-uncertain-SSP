#ifndef MPDLIB_GUSSPROCKSAMPLEPROBLEM_H
#define MPDLIB_GUSSPROCKSAMPLEPROBLEM_H

#include <unordered_set>
#include <map>
#include <cassert>
#include <utility>
#include <vector>

#include "GUSSPRockSampleState.h"
#include "GUSSPRockSampleAction.h"

#include "../../Problem.h"
#include "../../State.h"
#include "../../Action.h"
#include "../../util/general.h"

class RSDetHeuristicGUSSP;

namespace rocksample
{
    const unsigned char UP = 0;
    const unsigned char DOWN = 1;
    const unsigned char LEFT = 2;
    const unsigned char RIGHT = 3;
    const unsigned char SAMPLE = 4;

    const int UNKNOWN = 2;
    const int GOOD = 1;
    const int BAD = 0;
}

/**
 * A class representing a modified ROCKSAMPLEproblem to be an SSP. (https://arxiv.org/pdf/1207.4166.pdf)
 * Problems can be read from a file with the following notation:
 *
 *   - A '.' character represents an empty cell.
 *   - A 'x' character represents a wall (can't be traversed).
 *   - A '@' character represents a hole (can be traversed at a larger cost).
 *   - A 'D' character represents a dead-end.
 *   - A 'S' character represents the start cell.
 *   - A 'G' character represents a potential goal cell (multiple goals are allowed).
 */
class GUSSPRockSampleProblem : public mlcore::Problem
{
    friend class RSDetHeuristicGUSSP;

private:
    int width_;
    int height_;
    int x0_;
    int y0_;
    int sampledRocks0_;
    std::vector<std::pair<std::pair<int, int>,double>> goalPos0_;
    double actionCost_;
    double holeCost_;
    bool allDirections_;
    mlcore::State* absorbing;
    IntPairSet walls;
    IntPairSet holes;
    IntPairSet dead_ends;
    PairDoubleMap* goals_;

    void addSuccessor(GUSSPRockSampleState* state,
                      std::list<mlcore::Successor>& successors,
                      int val,
                      int limit,
                      int newx,
                      int newy,
                      int newsamples,
                      std::vector<std::pair<std::pair<int, int>,double>> newgoalPos,
                      double prob);

    void addSuccessor(GUSSPRockSampleState* state,
                      std::vector<mlcore::SuccessorsList>* allSuccessors,
                      int idAction,
                      int val,
                      int limit,
                      int newx,
                      int newy,
                      int newsamples,
                      std::vector<std::pair<std::pair<int, int>,double>> newgoalPos,
                      double prob);

    void addAllActions();

    bool GUSSPRockSampleGoal(GUSSPRockSampleState* s) const;
public:
    /**
     * Default constructor.
     */
    GUSSPRockSampleProblem();
    std::vector<std::pair<int,int>> potential_goals;
    /**
     * Constructs a rocksample world from a string file representation
     * stored at the given filename. The constructor also receives
     * the cost of the actions.
     */
    GUSSPRockSampleProblem(const char* filename,
                     double actionCost = 1.0,
                     double holeCost = 100.0,
                     bool allDirections = false,
                     std::string goal_dist = "uniform");


   ~GUSSPRockSampleProblem()
    {
        delete goals_;
    }

    /**
     * Overrides method from Problem.
     */
    virtual bool goal(mlcore::State* s) const;
    /**
     * Overrides method from Problem.
     */
    virtual std::list<mlcore::Successor> transition(mlcore::State* s,
                                                    mlcore::Action* a);

    /**
     * Overrides method from Problem.
     */
    virtual double cost(mlcore::State* s, mlcore::Action* a) const;

    /**
     * Overrides method from Problem.
     */
    virtual bool applicable(mlcore::State* s, mlcore::Action* a) const;

    /** set the true goal in GUSSp
    ** this is set to generate observations accordingly.
    ** One state is randomly chosen from the set of potential goals and set as the true goal.
    **/
    virtual void setTrueGoal(std::vector<std::pair<int,int>> potential_goals);

      /** sets initial belief **/
    virtual void setGoalProb(std::vector<std::pair<int,int>> potential_goals, std::string goal_dist);

    /** Returns 1 if the state is a true goal. Returns 0 otherwise.
     **/
    virtual int getObservation(GUSSPRockSampleState* s);
    virtual int getObservation(int x, int y);

    virtual std::vector<std::pair<std::pair<int,int>, double>> updateBelief(
                                                std::vector<std::pair<std::pair<int,int>, double>> curr_belief);

    /** Returns 1 if the state is a potential goal. Returns 0 otherwise.
     **/
    virtual bool isPotentialGoal(GUSSPRockSampleState* s);
    virtual bool isPotentialGoal(int x, int y);

    virtual bool GUSSPRSGoal(GUSSPRockSampleState* s) const;

    double getactioncost(){return actionCost_;}

    PairDoubleMap* getGoals(){return goals_;}
};

#endif // MPDLIB_GUSSPROCKSAMPLEPROBLEM_H
