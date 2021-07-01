#ifndef MACHINE_STATE_HPP
#define MACHINE_STATE_HPP

#include <std_msgs/Int32.h>

namespace MachineState {
    enum class State : int {
        SEARCH_OBJ = 0,
        HOLDING_OBJ, // 1
        CARRY_OBJ,   // 2
        INTO_GOAL,   // 3
        EXIT_GOAL,   // 4
        UNKNOWN      //5
    };

    class StateTransition {
      private:
        State prev_;
        State current_;
        State next_;

      public:
        StateTransition();

        virtual ~StateTransition();

        State prev();
        State current();
        State next();

        bool set_next(State next);

        void transition();

        std_msgs::Int32 current_as_msg();
    };

    int to_int(State state);

    std_msgs::Int32 to_msg(State state);

    State from_int(int state_int);

    State from_msg(const std_msgs::Int32ConstPtr msg);
} // namespace MachineState

#endif // MACHINE_STATE_HPP
