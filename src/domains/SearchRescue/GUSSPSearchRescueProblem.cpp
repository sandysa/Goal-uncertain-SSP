#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <vector>

#include "../../../include/Problem.h"
#include "../../../include/domains/SearchRescue/GUSSPSearchRescueProblem.h"
#include "../../../include/domains/SearchRescue/GUSSPSearchRescueState.h"
#include "../../../include/domains/SearchRescue/GUSSPSearchRescueAction.h"

using namespace std;

void GUSSPSearchRescueProblem::addAllActions()
{
    mlcore::Action* up = new GUSSPSearchRescueAction(searchrescue::UP);
    mlcore::Action* down = new GUSSPSearchRescueAction(searchrescue::DOWN);
    mlcore::Action* left = new GUSSPSearchRescueAction(searchrescue::LEFT);
    mlcore::Action* right = new GUSSPSearchRescueAction(searchrescue::RIGHT);
    mlcore::Action* save = new GUSSPSearchRescueAction(searchrescue::SAVE);
    actions_.push_front(up);
    actions_.push_front(down);
    actions_.push_front(left);
    actions_.push_front(right);
    actions_.push_front(save);
}


GUSSPSearchRescueProblem::GUSSPSearchRescueProblem() :
                    width_(0), height_(0), x0_(0), y0_(0), victims0_(0),
                    actionCost_(0.03)
{
    std::pair<int, int> pos (-1,-1);
    goalPos0_.push_back(std::make_pair(pos,1.0));
    absorbing = new GUSSPSearchRescueState(this, -1,-1, -1, goalPos0_);
    addAllActions();
}

void GUSSPSearchRescueProblem::setGoalProb(std::vector<std::pair<int,int>> potential_goals,
                                bool uniform_dist)
{
    if (uniform_dist){
     for (int i = 0; i < potential_goals.size(); ++i){
            std::pair<int,int> pos (potential_goals.at(i));
            goalPos0_.push_back(std::make_pair(pos, (1.0/potential_goals.size())));
        }
     }
}


GUSSPSearchRescueProblem::GUSSPSearchRescueProblem(const char* filename,
                                   double actionCost,
                                   double holeCost,
                                   bool allDirections,
                                   bool uniform_dist)
{
    std::ifstream myfile (filename);
    // Once the file is finished parsing, these will have correct values
    width_ = 0, height_ = 0;
    potential_goals.clear();
    if (myfile.is_open()) {
        std::string line;
        while ( std::getline (myfile, line) ) {
            for (width_ = 0; width_ < line.size(); width_++) {
                if (line.at(width_) == 'x') {
                    walls.insert(std::pair<int, int>(width_, height_));
                } else if (line.at(width_) == '@') {
                    holes.insert(std::pair<int, int>(width_, height_));
                } else if (line.at(width_) == 'G') {
                       potential_goals.push_back(std::pair<int, int>(width_, height_));
                } else if (line.at(width_) == 'D') {
                    dead_ends.insert(std::pair<int, int> (width_, height_));
                } else if (line.at(width_) == 'S') {
                    x0_ = width_;
                    y0_ = height_;
                  } else {
                    assert(line.at(width_) == '.');
                }
            }
            height_++;
        }
        myfile.close();
    } else {
        std::cerr << "Invalid file " << filename << std::endl;
        exit(-1);
    }
    actionCost_ = actionCost;
    holeCost_ = holeCost;
    allDirections_ = allDirections;
    setGoalProb(potential_goals, uniform_dist);

    s0 = new GUSSPSearchRescueState(this, x0_, y0_, 0, goalPos0_); //no victims at start state
    absorbing = new GUSSPSearchRescueState(this, -1, -1, -1, goalPos0_);
    this->addState(s0);
    this->addState(absorbing);
    addAllActions();
    setTrueGoal(potential_goals);

}

/** randomly sets victims in potential goals **/
void GUSSPSearchRescueProblem::setTrueGoal(std::vector<std::pair<int,int>> potential_goals)
{
    if (potential_goals.size() == 1){
        victimLocations.push_back(std::make_pair(potential_goals.at(0), 1));
        maxVictims_ = 1;
        }
    else {
        int totalVictims = 0;
        for(int i = 0; i < potential_goals.size(); i++){
            int index = rand() %2;
            if(index > 0)
                victimLocations.push_back(std::make_pair(potential_goals.at(i), index));
            totalVictims += index;
         }
        maxVictims_ = totalVictims;
                                                                                            std::cout << "potential goal size = " << potential_goals.size()
                                                                                        << " totalVictims = " << maxVictims_ << endl;
        if(maxVictims_ == 0)
        {
            victimLocations.push_back(std::make_pair(potential_goals.at(0), 1));
            maxVictims_ = 1;
        }
    }
//    std::cout << "Victim locations:" << std::endl;
//    for(auto it = victimLocations.begin(); it != victimLocations.end(); ++it){
//        std::pair<std::pair<int,int>,double> val = *it;
//        std::cout << val.first.first << "," << val.first.second << " : " << val.second << std::endl;
//    }

}

bool GUSSPSearchRescueProblem::GUSSPSRGoal(GUSSPSearchRescueState* srs) const
{
   return srs->victims() == maxVictims_;
}

bool GUSSPSearchRescueProblem::goal(mlcore::State* s) const
{
    return s == absorbing;
}

int GUSSPSearchRescueProblem::getObservation(GUSSPSearchRescueState* srs){
    if (isPotentialGoal(srs))
    {
        for (int i = 0; i < victimLocations.size(); i++)
        {
            std::pair<std::pair<int, int>, int> vloc = victimLocations.at(i);
            std::pair<int,int> pos = vloc.first;
            if (pos.first == srs->x() && pos.second == srs->y())
                return vloc.second;
        }
    }
    return 0;
}

int GUSSPSearchRescueProblem::getObservation(int x, int y){
    if (isPotentialGoal(x,y))
    {
        for (int i = 0; i < victimLocations.size(); i++)
        {
            std::pair<std::pair<int, int>, int> vloc = victimLocations.at(i);
            std::pair<int,int> pos = vloc.first;
            if (pos.first == x && pos.second == y)
                return vloc.second;
        }
    }
    return 0;
}

bool GUSSPSearchRescueProblem::isPotentialGoal(GUSSPSearchRescueState* srs){
     std::pair<int,int> pos(srs->x(),srs->y());
     return (std::find(potential_goals.begin(), potential_goals.end(), pos) != potential_goals.end());
}

bool GUSSPSearchRescueProblem::isPotentialGoal(int x, int y){
     std::pair<int,int> pos(x,y);
     return (std::find(potential_goals.begin(), potential_goals.end(), pos) != potential_goals.end());
}

/** normalizes belief based on observation **/
std::vector<std::pair<std::pair<int,int>, double>> GUSSPSearchRescueProblem::updateBelief
                                                    (std::vector<std::pair<std::pair<int,int>, double>> curr_belief)
{
     double total = 0;
//    int goal_index  = -1; //indicates if the true goal has been observed.
    std::vector<std::pair<std::pair<int,int>, double>> new_belief;
    for (int i = 0; i < curr_belief.size(); i++)
    {
        std::pair<std::pair<int,int>, double> pos =  curr_belief.at(i);
        total+= pos.second;
    }
    for (int i = 0; i < curr_belief.size(); i++)
    {
        std::pair<std::pair<int,int>, double> pos = curr_belief.at(i);
        new_belief.push_back(std::make_pair(pos.first, (pos.second/total)));
    }
    return new_belief;
}

std::list<mlcore::Successor>
GUSSPSearchRescueProblem::transition(mlcore::State *s, mlcore::Action *a)
{
    GUSSPSearchRescueState* state = static_cast<GUSSPSearchRescueState *> (s);
    GUSSPSearchRescueAction* action = static_cast<GUSSPSearchRescueAction *> (a);
    std::vector<std::pair<std::pair<int,int>, double>> goalPos = state->goalPos();
    std::list<mlcore::Successor> successors;

    if (s == absorbing || GUSSPSRGoal(state)) {
        successors.push_front(mlcore::Successor(absorbing, 1.0));
        return successors;
    }
    if (dead_ends.count(std::pair<int, int>(state->x(), state->y()))) {
        s->markDeadEnd();
        successors.push_front(mlcore::Successor(s, 1.0));
        return successors;
    }
 /** Otherwise check observation to update belief over goal configurations **/
    int obs =  getObservation(state);

    /** return if the transition is found in the cache **/
        int idAction = action->hashValue();
        std::vector<mlcore::SuccessorsList>* allSuccessors = state->allSuccessors();
        if (!allSuccessors->at(idAction).empty()) {
            return allSuccessors->at(idAction);
        }

     double curr_bel = 0;
     for(auto it = goalPos.begin(); it != goalPos.end(); ++it){
        std::pair<std::pair<int,int>,double> val = *it;
        if(val.first == std::make_pair(state->x(),state->y()))
            curr_bel = val.second;
     }

  /** SAVE action: **/
    if (action->dir() == searchrescue::SAVE && obs > 0)
    {
//                                                                                        std::cout<< " saving at " << state <<std::endl;
       if(curr_bel > 0 ) {
            addSuccessor(state, allSuccessors, idAction, 100, 0,
                             state->x(), state->y(), (state->victims() + obs), goalPos, 1.0);
        }
        else
            addSuccessor(state, allSuccessors, idAction, 100, 0,
                             state->x(), state->y(), state->victims(), goalPos, 1.0);
        return successors;
    }


    /** SSP state transition **/
    double probForward = 0.8;
    int numSuccessors = allDirections_ ? 3 : 2;
    double probSides = 0.2 / numSuccessors;

    if (action->dir() == searchrescue::DOWN) {
       addSuccessor(state, allSuccessors, idAction, height_ - 1, state->y(),
                     state->x(), state->y() + 1, state->victims(),
                     state->goalPos(), probForward);

        addSuccessor(state, allSuccessors, idAction, state->x(), 0,
                     state->x() - 1, state->y(), state->victims(),
                     state->goalPos(), probSides);

        addSuccessor(state, allSuccessors, idAction, width_ - 1, state->x(),
                     state->x() + 1, state->y(), state->victims(),
                     state->goalPos(), probSides);

        if (allDirections_) {
            addSuccessor(state, allSuccessors, idAction, state->y(), 0,
                         state->x(), state->y() - 1, state->victims(),
                         state->goalPos(), probSides);
        }
    } else if (action->dir() == searchrescue::UP) {
        addSuccessor(state, allSuccessors,idAction, state->y(), 0,
                     state->x(), state->y() - 1, state->victims(),
                     state->goalPos(), probForward);

        addSuccessor(state, allSuccessors,idAction, state->x(), 0,
                     state->x() - 1, state->y(), state->victims(),
                     state->goalPos(), probSides);

        addSuccessor(state, allSuccessors,idAction, width_ - 1, state->x(),
                     state->x() + 1, state->y(), state->victims(),
                     state->goalPos(), probSides);

        if (allDirections_) {
                addSuccessor(state, allSuccessors,idAction, height_ - 1, state->y(),
                         state->x(), state->y() + 1, state->victims(),
                         state->goalPos(), probSides);
        }
    } else if (action->dir() == searchrescue::LEFT) {
        addSuccessor(state, allSuccessors, idAction, state->x(), 0,
                     state->x() - 1, state->y(), state->victims(),
                     state->goalPos(), probForward);

        addSuccessor(state, allSuccessors, idAction, state->y(), 0,
                     state->x(), state->y() - 1, state->victims(),
                     state->goalPos(), probSides);

        addSuccessor(state, allSuccessors, idAction, height_ - 1, state->y(),
                     state->x(), state->y() + 1, state->victims(),
                     state->goalPos(), probSides);

        if (allDirections_) {
            addSuccessor(state, allSuccessors, idAction, width_ - 1, state->x(),
                         state->x() + 1, state->y(), state->victims(),
                         state->goalPos(), probSides);
        }
    } else if (action->dir() == searchrescue::RIGHT) {
        addSuccessor(state, allSuccessors, idAction, width_ - 1, state->x(),
                     state->x() + 1, state->y(), state->victims(),
                     state->goalPos(), probForward);

        addSuccessor(state, allSuccessors, idAction, state->y(), 0,
                     state->x(), state->y() - 1, state->victims(),
                     state->goalPos(), probSides);

        addSuccessor(state, allSuccessors, idAction, height_ - 1, state->y(),
                     state->x(), state->y() + 1, state->victims(),
                     state->goalPos(), probSides);

        if (allDirections_) {
            addSuccessor(state, allSuccessors, idAction, state->x(), 0,
                         state->x() - 1, state->y(), state->victims(),
                         state->goalPos(), probSides);
        }

    }
    return allSuccessors->at(idAction);
}

double GUSSPSearchRescueProblem::cost(mlcore::State* s, mlcore::Action* a) const
{
    if (s == absorbing)
        return 0.0;

    GUSSPSearchRescueAction* sra = static_cast<GUSSPSearchRescueAction *> (a);
    GUSSPSearchRescueState* srs = static_cast<GUSSPSearchRescueState *> (s);
    std::pair<int,int> pos (srs->x(), srs->y());
    bool hasVictims = false;

    for (int i = 0; i < victimLocations.size(); i++)
        {
            std::pair<std::pair<int, int>, int> vloc = victimLocations.at(i);
            std::pair<int,int> pos = vloc.first;
            if (pos.first == srs->x() && pos.second == srs->y() && vloc.second > 0)
                hasVictims = true;
        }
    if (sra->dir() == searchrescue::SAVE && hasVictims)
        return 2.0 * actionCost_;
    if (sra->dir() == searchrescue::SAVE && !hasVictims)
        return 10.0 * actionCost_;

    if (GUSSPSRGoal(srs)) {
        return 0;
    }

   if (holes.count(std::pair<int, int> (srs->x(), srs->y())) != 0)
        return holeCost_ * actionCost_;

    return actionCost_;
}


bool GUSSPSearchRescueProblem::applicable(mlcore::State* s, mlcore::Action* a) const
{
    GUSSPSearchRescueState* srs = static_cast<GUSSPSearchRescueState *> (s);
    GUSSPSearchRescueProblem* srp = const_cast<GUSSPSearchRescueProblem*> (this);
    GUSSPSearchRescueAction* sra = static_cast<GUSSPSearchRescueAction *> (a);
    if (sra->dir() == searchrescue::SAVE) // applicable in potential goals
    {
       if(srp->isPotentialGoal(srs) && srp->getObservation(srs->x(), srs->y()) > 0 )
            return true;

        return false;
    }
    // move action is always applicable
        return true;
}

void GUSSPSearchRescueProblem::addSuccessor(
    GUSSPSearchRescueState* state, std::list<mlcore::Successor>& successors,
    int val, int limit, int newx, int newy, int newvictims, std::vector<std::pair<std::pair<int, int>,double>> newgoalPos, double prob)
{
    bool isWall = (walls.count(std::pair<int, int> (newx, newy)) != 0);
    if (val > limit && !isWall) {
            GUSSPSearchRescueState *next = new GUSSPSearchRescueState(this, newx, newy, newvictims, newgoalPos);
            successors.push_front(mlcore::Successor(this->addState(next), prob));
    } else {
        successors.push_front(mlcore::Successor(state, prob));
    }
}
std::vector<std::pair<std::pair<int,int>,double>> belCollapse(std::vector<std::pair<std::pair<int,int>,double>> curr_bel, std::pair<int,int> comp)
{
    std::vector<std::pair<std::pair<int,int>,double>> to_ret;
    for (int i = 0; i < curr_bel.size(); i++)
    {
        std::pair<std::pair<int,int>, double> pos = curr_bel.at(i);
        if(comp == pos.first)
                to_ret.push_back(std::make_pair(pos.first, 1.0));
        else
                to_ret.push_back(std::make_pair(pos.first, 0.0));
    }

    return to_ret;

}


void GUSSPSearchRescueProblem::addSuccessor(
    GUSSPSearchRescueState* state, std::vector<mlcore::SuccessorsList>* allSuccessors, int idAction,
    int val, int limit, int newx, int newy, int newvictims, std::vector<std::pair<std::pair<int, int>,double>> newgoalPos, double prob)
{
    /** check observation to update belief over goal configurations **/
    int obs =  getObservation(newx, newy);
    std::vector<std::pair<std::pair<int, int>,double>> newgoalPoscp  = newgoalPos;

   // if(newvictims > state->victims() && newvictims != maxVictims_ && idAction == searchrescue::SAVE){
    if(newvictims > state->victims() && newvictims < maxVictims_)
        obs = 0;


    /** If potential goal, update beliefs based on observation **/
    if (isPotentialGoal(newx, newy) && obs == 0)//&&obs == 0
    {
        for (auto i = newgoalPoscp.begin(); i != newgoalPoscp.end(); ++i){
            std::pair<std::pair<int,int>, double> pos (*i);
            std::pair<int,int> locs = pos.first;
            if (locs.first == newx && locs.second == newy && pos.second !=obs)
            {
                newgoalPoscp.erase(i);
                newgoalPoscp.push_back(std::make_pair(locs,obs));
//                for (int k = 0; k < victimLocations.size(); k++)
//                {
//                    std::pair<std::pair<int, int>, int> vloc = victimLocations.at(k);
//                    if (vloc.first.first == newx  && vloc.first.second == newy && state->victims()+vloc.second == maxVictims_ && obs == 1)
//                    {
//                        newgoalPos = belCollapse(newgoalPoscp, vloc.first);
//                        break;
//                    }
//                }
                newgoalPos = updateBelief(newgoalPoscp);
                break;
             }
        }
    }

    bool isWall = (walls.count(std::pair<int, int> (newx, newy)) != 0);
    if (val > limit && !isWall) {
            GUSSPSearchRescueState *next = new GUSSPSearchRescueState(this, newx, newy, newvictims, newgoalPos);
            allSuccessors->at(idAction).push_back(mlcore::Successor(this->addState(next), prob));

     } else {
        GUSSPSearchRescueState *next = new GUSSPSearchRescueState(this, state->x(), state->y(), state->victims() , newgoalPos);
        allSuccessors->at(idAction).push_back(mlcore::Successor(this->addState(next), prob)); //remains in same state with updated beliefs
     }
}


