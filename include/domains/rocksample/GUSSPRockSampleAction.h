#ifndef MPDLIB_GUSSPROCKSAMPLEACTION_H
#define MPDLIB_GUSSPROCKSAMPLEACTION_H

#include "../../Action.h"

class GUSSPRockSampleAction : public mlcore::Action
{
private:
    unsigned char dir_;
    virtual std::ostream& print(std::ostream& os) const;

public:
    GUSSPRockSampleAction() : dir_(-1) {}

    GUSSPRockSampleAction(const unsigned char dir) : dir_(dir) {}

    /**
     * Overriding method from Action.
     */
    virtual mlcore::Action& operator=(const mlcore::Action& rhs)
    {
        if (this == &rhs)
            return *this;

        GUSSPRockSampleAction* action = (GUSSPRockSampleAction*)  & rhs;
        dir_ =  action->dir_;
        return *this;
    }

    /**
     * Overriding method from Action.
     */
    virtual int hashValue() const
    {
        return (int) dir_;
    }

    unsigned char dir() const
    {
        return dir_;
    }
};

#endif // MPDLIB_GUSSPROCKSAMPLEACTION_H
