#ifndef MPDLIB_SEARCHRESCUEPROBLEM_H
#define MPDLIB_SEARCHRESCUEPROBLEM_H

#include <unordered_set>
#include <map>
#include <cassert>
#include <utility>
#include <vector>

#include "GUSSPSearchRescueState.h"
#include "GUSSPSearchRescueAction.h"

#include "../../Problem.h"
#include "../../State.h"
#include "../../Action.h"
#include "../../util/general.h"

class SRDetHeuristicGUSSP;

namespace searchrescue
{
    const unsigned char UP = 0;
    const unsigned char DOWN = 1;
    const unsigned char LEFT = 2;
    const unsigned char RIGHT = 3;
    const unsigned char SAVE = 4;

    const int UNKNOWN = 2;
    const int TRUE = 1;
    const int FALSE = 0;
}

/**
 * A class representing a search and rescue problem.
 * Each potential goal lcoation may or may not have victims.
 * It is currently restricted to max 1 victim per location.
 * The agent's goal is to save all victims.
 * Problems can be read from a file with the following notation:
 *
 *   - A '.' character represents an empty cell.
 *   - A 'x' character represents a wall (can't be traversed).
 *   - A '@' character represents a hole (can be traversed at a larger cost).
 *   - A 'D' character represents a dead-end.
 *   - A 'S' character represents the start cell.
 *   - A 'G' character represents a potential goal cell (multiple goals are allowed).
 */
class GUSSPSearchRescueProblem : public mlcore::Problem
{
    friend class SRDetHeuristicGUSSP;

private:
    int width_;
    int height_;
    int x0_;
    int y0_;
    int victims0_;
    int maxVictims_;
    std::vector<std::pair<std::pair<int, int>,double>> goalPos0_;
    double actionCost_;
    double holeCost_;
    bool allDirections_;
    mlcore::State* absorbing;
    IntPairSet walls;
    IntPairSet holes;
    IntPairSet dead_ends;
    std::vector<std::pair<int,int>> potential_goals;
    std::vector<std::pair<std::pair<int,int>, int>> victimLocations;
    void addSuccessor(GUSSPSearchRescueState* state,
                      std::list<mlcore::Successor>& successors,
                      int val,
                      int limit,
                      int newx,
                      int newy,
                      int newvictims,
                      std::vector<std::pair<std::pair<int, int>,double>> newgoalPos,
                      double prob);

    void addSuccessor(GUSSPSearchRescueState* state,
                      std::vector<mlcore::SuccessorsList>* allSuccessors,
                      int idAction,
                      int val,
                      int limit,
                      int newx,
                      int newy,
                      int newvictims,
                      std::vector<std::pair<std::pair<int, int>,double>> newgoalPos,
                      double prob);

    void addAllActions();

    bool GUSSPSearchRescueGoal(GUSSPSearchRescueState* s) const;
public:
    /**
     * Default constructor.
     */
    GUSSPSearchRescueProblem();

    /**
     * Constructs a search and rescue world from a string file representation
     * stored at the given filename. The constructor also receives
     * the cost of the actions.
     */
    GUSSPSearchRescueProblem(const char* filename,
                     double actionCost = 1.0,
                     double holeCost = 100.0,
                     bool allDirections = false,
                     bool uniform_goal_dist = true);

    ~GUSSPSearchRescueProblem()
    {
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
    virtual void setGoalProb(std::vector<std::pair<int,int>> potential_goals, bool uniform_goal_dist);

    /** Returns 1 if the state is a true goal. Returns 0 otherwise.
     **/
    virtual int getObservation(GUSSPSearchRescueState* s);
    virtual int getObservation(int x, int y);

    virtual std::vector<std::pair<std::pair<int,int>, double>> updateBelief(
                                        std::vector<std::pair<std::pair<int,int>, double>> curr_belief);

    /** Returns 1 if the state is a potential goal. Returns 0 otherwise.
     **/
    virtual bool isPotentialGoal(GUSSPSearchRescueState* s);
    virtual bool isPotentialGoal(int x, int y);

    virtual bool GUSSPSRGoal(GUSSPSearchRescueState* s) const;

    double getactioncost(){return actionCost_;}

    int maxVictims() {return maxVictims_;}

};

#endif // MPDLIB_SEARCHRESCUEPROBLEM_H
