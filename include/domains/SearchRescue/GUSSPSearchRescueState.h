#ifndef MPDLIB_GUSSPSEARCHRESCUESTATE_H
#define MPDLIB_GUSSPSEARCHRESCUESTATE_H

#include <iostream>
#include <vector>
#include <map>
#include <cassert>
#include "../../State.h"

/**
 * A class representing a grid world state as described in AIAMA 3rd Edition.
 */
class GUSSPSearchRescueState : public mlcore::State
{
private:
    int x_;
    int y_;
    int victims_;
    std::vector<std::pair<std::pair<int, int>,double>> goalPos_; //goalPos <int,int>, 1 occurs only when all victims have been saved.

    virtual std::ostream& print(std::ostream& os) const;

        /* A cache of all successors (for all actions) of this state */
    std::vector<mlcore::SuccessorsList> allSuccessors_;

public:

    /**
     * Constructs a SearchRescueState representing grid position (x,y)
     * on the problem
     * given as a first parameter.
     */
    GUSSPSearchRescueState(mlcore::Problem* problem, int x, int y, int victims,
                            std::vector<std::pair<std::pair<int, int>,double>> goalPos);

    /**
     * Copy constructor. The resulting state represents the same position as the
     * state passed as parameter.
     */
    GUSSPSearchRescueState(const GUSSPSearchRescueState& srs) : x_(srs.x_), y_(srs.y_),
                                                                victims_(srs.victims_),
                                                                goalPos_(srs.goalPos_){}

     /**
     * Returns a pointer to the successor cache of this state.
     */
    std::vector<mlcore::SuccessorsList>* allSuccessors()
    {
        return &allSuccessors_;
    }

    virtual mlcore::State& operator=(const mlcore::State& rhs)
    {
        if (this == &rhs)
            return *this;

        GUSSPSearchRescueState* srs = (GUSSPSearchRescueState *) &rhs;
        x_ =  srs->x_;
        y_ =  srs->y_;
        victims_ = srs->victims_;
        goalPos_ =  srs->goalPos_;
        return *this;
    }

    virtual bool operator==(const mlcore::State& rhs) const
    {
        GUSSPSearchRescueState* srs = (GUSSPSearchRescueState *) &rhs;
        return x_ == srs->x_ && y_ == srs->y_ && victims_ == srs->victims_ && goalPos_ == srs->goalPos_;
    }

    virtual bool equals(mlcore::State* other) const;
    virtual int hashValue() const;


    int x() {return x_;}

    int y() {return y_;}

    int victims() {return victims_;}

    std::vector<std::pair<std::pair<int, int>,double>>goalPos() {return goalPos_;}

};

#endif // MPDLIB_GUSSPSEARCHRESCUESTATE_H
