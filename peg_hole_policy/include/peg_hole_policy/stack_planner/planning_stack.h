#ifndef PEG_HOLE_PLANNING_STACK_H_
#define PEG_HOLE_PLANNING_STACK_H_

#include "peg_hole_policy/stack_planner/state_machine.h"

namespace ph_policy{

typedef enum actions{
    NONE,
    FIND_TABLE,
    GO_TO_EDGE,
    GET_BACK_ONTO_TABLE,
    GO_TO_SOCKET,
    FIND_SOCKET_HOLE,
    INSERT
}actions;

inline std::string actions2str(actions action){
    switch(action){
    case NONE:
        return "NONE";
    case FIND_TABLE:
        return "FIND_TABLE";
    case GO_TO_EDGE:
        return "GO_TO_EDGE";
    case GO_TO_SOCKET:
        return "GO_TO_SOCKET";
    case FIND_SOCKET_HOLE:
        return "FIND_SOCKET_HOLE";
    case INSERT:
        return "INSERT";
    case GET_BACK_ONTO_TABLE:
        return "GET_BACK_ONTO_TABLE";
    }
}


class Planning_stack{

public:

    Planning_stack();

    void update(const std::vector<STATES>& states);

    actions get_action() const;

    void reset();

    const void print(double seconds=0) const;

    static bool has_state(const STATES state, const std::vector<STATES>& states);

    void set_action(actions action);

private:

    void next_action();

private:

    std::vector<actions> stack;
    actions              current_action;
    std::size_t          index;
    bool                 bFrist;
};

}

#endif
