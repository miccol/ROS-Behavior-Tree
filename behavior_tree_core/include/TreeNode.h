#ifndef BEHAVIORTREECORE_TREENODE_H
#define BEHAVIORTREECORE_TREENODE_H

#include <iostream>
#include <unistd.h>

#include <string>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <TickEngine.h>
#include <Exceptions.h>

namespace BT
{
    // Enumerates the possible types of a node, for drawinf we have do discriminate whoich control node it is:

    enum NodeType {ACTION_NODE, CONDITION_NODE, CONTROL_NODE};
    enum DrawNodeType {PARALLEL, SELECTOR, SEQUENCE, SEQUENCESTAR, SELECTORSTAR, ACTION, CONDITION,DECORATOR};
    // Enumerates the states every node can be in after execution during a particular
    // time step:
    // - "Success" indicates that the node has completed running during this time step;
    // - "Failure" indicates that the node has determined it will not be able to complete
    //   its task;
    // - "Running" indicates that the node has successfully moved forward during this
    //   time step, but the task is not yet complete;
    // - "Idle" indicates that the node hasn't run yet.
    // - "Halted" indicates that the node has been halted by its father.
    enum NodeState {SUCCESS, FAILURE, RUNNING, IDLE, HALTED, EXIT};

    // Enumerates the options for when a parallel node is considered to have failed:
    // - "FAIL_ON_ONE" indicates that the node will return failure as soon as one of
    //   its children fails;
    // - "FAIL_ON_ALL" indicates that all of the node's children must fail before it
    //   returns failure.
    enum FailurePolicy {FAIL_ON_ONE, FAIL_ON_ALL};

    // Enumerates the options for when a parallel node is considered to have succeeded:
    // - "SUCCEED_ON_ONE" indicates that the node will return success as soon as one
    //   of its children succeeds;
    // - "BT::SUCCEED_ON_ALL" indicates that all of the node's children must succeed before
    //   it returns success.
    enum SuccessPolicy {SUCCEED_ON_ONE, SUCCEED_ON_ALL};

    // If "BT::FAIL_ON_ONE" and "BT::SUCCEED_ON_ONE" are both active and are both trigerred in the
    // same time step, failure will take precedence.

    // Abstract base class for Behavior Tree Nodes
    class TreeNode
    {
    private:
        // Node name
        std::string name_;



    protected:
        // The node state that must be treated in a thread-safe way
        bool is_state_updated_;
        NodeState state_;
        NodeState color_state_;
        boost::mutex state_mutex_;
        boost::mutex color_state_mutex_;
        boost::condition_variable state_condition_variable_;
        // Node type
        NodeType type_;
        //position and offset for horizontal positioning when drawing
        float x_shift_, x_pose_;

    public:


        // The thread that will execute the node
        boost::thread thread_;

        // Node semaphore to simulate the tick
        // (and to synchronize fathers and children)
        TickEngine tick_engine;




        // The constructor and the distructor
        TreeNode(std::string name);
        ~TreeNode();

        // The method that is going to be executed by the thread
        virtual void Exec() = 0;

        // The method used to interrupt the execution of the node
        virtual bool Halt() = 0;

        // The method that retrive the state of the node
        // (conditional waiting and mutual access)
        NodeState GetNodeState();
        void SetNodeState(NodeState StateToBeSet);
        void SetColorState(NodeState ColorStateToBeSet);

        // Methods used to access the node state without the
        // conditional waiting (only mutual access)
        NodeState ReadState();
        NodeState ReadColorState();
        virtual int DrawType() = 0;
        virtual bool WriteState(NodeState StateToBeSet) = 0;
        virtual void ResetColorState() = 0;
        virtual int Depth() = 0;


        //Getters and setters
        void set_x_pose(float x_pose);
        float get_x_pose();

        void set_x_shift(float x_shift);
        float get_x_shift();

        std::string get_name();
        NodeType get_type();


    };
}

#endif
