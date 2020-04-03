#ifndef MPDLIB_GUSSPSEARCHRESCUEACTION_H
#define MPDLIB_GUSSPSEARCHRESCUEACTION_H

#include "../../Action.h"

class GUSSPSearchRescueAction : public mlcore::Action
{
private:
    unsigned char dir_;
    virtual std::ostream& print(std::ostream& os) const;

public:
    GUSSPSearchRescueAction() : dir_(-1) {}

    GUSSPSearchRescueAction(const unsigned char dir) : dir_(dir) {}

    /**
     * Overriding method from Action.
     */
    virtual mlcore::Action& operator=(const mlcore::Action& rhs)
    {
        if (this == &rhs)
            return *this;

        GUSSPSearchRescueAction* action = (GUSSPSearchRescueAction*)  & rhs;
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

#endif // MPDLIB_GUSSPSEARCHRESCUEACTION_H
