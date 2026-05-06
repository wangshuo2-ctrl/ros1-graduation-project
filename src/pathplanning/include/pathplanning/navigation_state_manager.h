#ifndef F25581A1_952D_4F51_82DD_93D924C6A435
#define F25581A1_952D_4F51_82DD_93D924C6A435
#ifndef NAVIGATION_STATE_MANAGER_H
#define NAVIGATION_STATE_MANAGER_H

#include <ros/ros.h>
#include <string>
#include <atomic>

class NavigationStateManager {
public:
    enum State {
        IDLE = 0,
        GLOBAL_PLANNING,
        LOCAL_EXECUTION, 
        GOAL_REACHED,
        FAILED
    };

    NavigationStateManager() : current_state_(IDLE) {
        state_names_ = {
            "IDLE",
            "GLOBAL_PLANNING", 
            "LOCAL_EXECUTION",
            "GOAL_REACHED",
            "FAILED"
        };
    }

    bool transitionTo(State new_state) {
        State old_state = current_state_;
        
        // 状态转换检查
        if (!isValidTransition(old_state, new_state)) {
            ROS_WARN("Invalid state transition: %s -> %s", 
                     state_names_[old_state].c_str(), state_names_[new_state].c_str());
            return false;
        }

        current_state_ = new_state;
        ROS_INFO("State transition: %s -> %s", 
                 state_names_[old_state].c_str(), state_names_[new_state].c_str());
        return true;
    }

    void reset() {
        current_state_ = IDLE;
        ROS_INFO("Navigation state reset to IDLE");
    }

    State getState() const { return current_state_; }
    std::string getStateString() const { return state_names_[current_state_]; }

    bool isIdle() const { return current_state_ == IDLE; }
    bool isPlanning() const { return current_state_ == GLOBAL_PLANNING; }
    bool isExecuting() const { return current_state_ == LOCAL_EXECUTION; }
    bool isGoalReached() const { return current_state_ == GOAL_REACHED; }
    bool isFailed() const { return current_state_ == FAILED; }

private:
    bool isValidTransition(State from, State to) {
        switch (from) {
            case IDLE:
                return to == GLOBAL_PLANNING;
            case GLOBAL_PLANNING:
                return to == LOCAL_EXECUTION || to == FAILED;
            case LOCAL_EXECUTION:
                return to == GOAL_REACHED || to == FAILED || to == IDLE;
            case GOAL_REACHED:
                return to == IDLE;
            case FAILED:
                return to == IDLE;
            default:
                return false;
        }
    }

private:
    std::atomic<State> current_state_;
    std::vector<std::string> state_names_;
};

#endif


#endif /* F25581A1_952D_4F51_82DD_93D924C6A435 */
