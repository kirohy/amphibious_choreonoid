#include <amphibious_choreonoid_motions/machine_state.hpp>

namespace MachineState {
    StateTransition::StateTransition() {
        prev_ = current_ = next_ = State::SEARCH_OBJ;
    }

    StateTransition::~StateTransition() {}

    State StateTransition::prev() {
        return prev_;
    }

    State StateTransition::current() {
        return current_;
    }

    State StateTransition::next() {
        return next_;
    }

    bool StateTransition::set_next(State next) {
        if (next == State::UNKNOWN) {
            return false;
        }
        next_ = next;
        return true;
    }

    void StateTransition::transition() {
        prev_ = current_;
        current_ = next_;
    }

    std_msgs::Int32 StateTransition::current_as_msg() {
        return to_msg(current_);
    }

    int to_int(State state) {
        return static_cast<int>(state);
    }

    std_msgs::Int32 to_msg(State state) {
        std_msgs::Int32 msg;
        msg.data = to_int(state);
        return msg;
    }

    State from_int(int state_int) {
        State res;
        switch (state_int) {
        case 0:
            res = State::SEARCH_OBJ;
            break;
        case 1:
            res = State::HOLDING_OBJ;
            break;
        case 2:
            res = State::CARRY_OBJ;
            break;
        case 3:
            res = State::INTO_GOAL;
            break;
        case 4:
            res = State::EXIT_GOAL;
            break;
        default:
            res = State::UNKNOWN;
            break;
        }
        return res;
    }

    State from_msg(const std_msgs::Int32ConstPtr msg) {
        return from_int(msg->data);
    }
} // namespace MachineState
