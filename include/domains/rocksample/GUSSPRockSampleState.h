#ifndef MPDLIB_GUSSPROCKSAMPLESTATE_H
#define MPDLIB_GUSSPROCKSAMPLESTATE_H

#include <iostream>
#include <vector>
#include <map>
#include <cassert>
#include "../../State.h"

/**
 * A class representing a grid world state as described in AIAMA 3rd Edition.
 */
class GUSSPRockSampleState : public mlcore::State
{
private:
    int x_;
    int y_;
    int sampledRocks_; // 0 indicates nothing sampled yet. 1 indicates successfully sampled.
    std::vector<std::pair<std::pair<int, int>,double>> goalPos_;


    virtual std::ostream& print(std::ostream& os) const;

    /* A cache of all successors (for all actions) of this state */
    std::vector<mlcore::SuccessorsList> allSuccessors_;

public:

    /**
     * Constructs a RockSampleState representing grid position (x,y)
     * on the problem
     * given as a first parameter.
     */
    GUSSPRockSampleState(mlcore::Problem* problem, int x, int y, int sampledRocks,
                        std::vector<std::pair<std::pair<int, int>,double>> goalbel);

    /**
     * Copy constructor. The resulting state represents the same position as the
     * state passed as parameter.
     */
    GUSSPRockSampleState(const GUSSPRockSampleState& rss) : x_(rss.x_), y_(rss.y_),sampledRocks_(rss.sampledRocks_),
                                                                                    goalPos_(rss.goalPos_){}

     /**
     * Returns a pointer to the successor cache of this state.
     */
    std::vector<mlcore::SuccessorsList>* allSuccessors()
    {
        return &allSuccessors_;
    }
//    template<>
//    std::vector<std::pair<std::pair<int, int>,double>>  getGoalPos(){ return goalPos_; }

    virtual mlcore::State& operator=(const mlcore::State& rhs)
    {
        if (this == &rhs)
            return *this;

        GUSSPRockSampleState* rss = (GUSSPRockSampleState *) &rhs;
        x_ =  rss->x_;
        y_=  rss->y_;
        sampledRocks_ = rss->sampledRocks_;
        goalPos_ =  rss->goalPos_;
        return *this;
    }

    virtual bool operator==(const mlcore::State& rhs) const
    {
        GUSSPRockSampleState* rss = (GUSSPRockSampleState *) &rhs;
        return x_ == rss->x_ && y_ == rss->y_ && sampledRocks_ == rss->sampledRocks_  && goalPos_ == rss->goalPos_;
    }

    virtual bool equals(mlcore::State* other) const;
    virtual int hashValue() const;


    int x() {return x_;}

    int y() {return y_;}

    int sampledRocks() {return sampledRocks_;}

    std::vector<std::pair<std::pair<int, int>,double>>goalPos() {return goalPos_;}

};

#endif // MPDLIB_GUSSPROCKSAMPLESTATE_H
