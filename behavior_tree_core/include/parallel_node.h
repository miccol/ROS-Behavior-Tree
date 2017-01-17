#ifndef PARALLELNODE_H
#define PARALLELNODE_H

#include <control_node.h>

#include <limits>

namespace BT
{
    class ParallelNode : public ControlNode
    {
    private:
        // N threshold
        unsigned int threshold_;

        // Number of returned Success, Failure and Running states
        unsigned int n_of_successes_;
        unsigned int n_of_failures_;
        unsigned int n_of_runnings_;

        // Update states vector
        std::vector<bool> is_children_states_updated_vtr_;

        // state_ update
        bool is_state_updated_;
    public:
        // Constructor
        ParallelNode(std::string name);
        ~ParallelNode();

        // the method used to set threshold_
        void SetThreshold(unsigned int new_threshold);
        // Method that retuns the type
        int DrawType();
        // The method that is going to be executed by the thread
        void Exec();
    };
}

#endif
