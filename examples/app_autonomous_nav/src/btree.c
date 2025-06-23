#include <stdio.h>
#include <stdlib.h>
#include "btree.h"

#define DEBUG_MODULE "BTREE"
#include "debug.h"

#define TURN_RATE 0.5f
#define FWD_VEL 0.3f



// ========= Composite Nodes ===========

BTStatus executeBTSequence(BTNode *node, BTBlackboard *bb)
{
    while (node->composite.current_child < node->composite.child_count) {
            BTNode *child = node->composite.children[node->composite.current_child];
            BTStatus status = child->execute(child, bb);
            if (status == BT_RUNNING) {
                return BT_RUNNING;
            }
            if (status == BT_FAILURE) {
                node->composite.current_child = 0;
                return BT_FAILURE;
            }
            node->composite.current_child++;
        }
        node->composite.current_child = 0;
        return BT_SUCCESS;
}


BTStatus executeBTSelector(BTNode *node, BTBlackboard *bb)
{
    while (node->composite.current_child < node->composite.child_count) {
            BTNode *child = node->composite.children[node->composite.current_child];
            BTStatus status = child->execute(child, bb);
            if (status == BT_RUNNING) {
                return BT_RUNNING;
            }
            if (status == BT_SUCCESS) {
                node->composite.current_child = 0;
                return BT_SUCCESS;
            }
            node->composite.current_child++;
        }
        node->composite.current_child = 0;
        return BT_FAILURE;
}



// ======== Leaf Node Functions =======


// Actions

BTStatus turnRightBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Action Turn Right");
    bb->r_cmd = TURN_RATE;
    return BT_SUCCESS;
}

BTStatus turnLeftBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Action Turn Left");
    bb->r_cmd = -TURN_RATE;
    return BT_SUCCESS;
}

BTStatus moveForwardBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Action Move Fwd");
    bb->vx_cmd = FWD_VEL;
    bb->r_cmd = 0.0f;
    return BT_SUCCESS;
}


// Conditions

BTStatus randomConditionBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Condition Random");
    if (rand() % 2 == 0)
    {
        return BT_SUCCESS;
    } 
    else
    {
        return BT_FAILURE;
    }
}

BTStatus pathClearBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Condition Path clear");
    if (bb->pathClear)
    { 
        return BT_SUCCESS;
    } 
    else
    {
        return BT_FAILURE;
    }
}

BTStatus leftOverRightBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Condition Left over right");
    if (bb->leftDist > bb->rightDist)
    {
        return BT_SUCCESS;
    }
    else
    {
        return BT_FAILURE;
    }
}







// ==== Usable Behaviour Trees ======


// Manually designed tree
static BTNode lr_random_node = {.type = BT_LEAF, .execute = randomConditionBT};
static BTNode pathclear_node = {.type = BT_LEAF, .execute = pathClearBT};
static BTNode turnleft_node = {.type = BT_LEAF, .execute = turnLeftBT};
static BTNode turnright_node = {.type = BT_LEAF, .execute = turnRightBT};
static BTNode moveforward_node = {.type = BT_LEAF, .execute = moveForwardBT};

static BTNode *lr_sel_children[] = {&lr_random_node, &turnleft_node};
BTNode lr_sel = {
    .type = BT_SELECTOR,
    .execute = executeBTSelector,
    .composite = {
        .children = lr_sel_children,
        .child_count = 2,
        .current_child = 0
    }
};

static BTNode *lr_seq_children[] = {&lr_sel, &turnright_node};
BTNode lr_seq = {
    .type = BT_SEQUENCE,
    .execute = executeBTSequence,
    .composite = {
        .children = lr_seq_children,
        .child_count = 2,
        .current_child = 0
    }
};


static BTNode *obs_avoidance_sel_children[] = {&pathclear_node, &lr_seq};
BTNode obs_avoidance_sel = {
    .type = BT_SELECTOR,
    .execute = executeBTSelector,
    .composite = {
        .children = obs_avoidance_sel_children,
        .child_count = 2,
        .current_child = 0
    }
};


static BTNode *ManualTree_children[] = {&obs_avoidance_sel, &moveforward_node};
BTNode ManualTree = {
    .type = BT_SEQUENCE,
    .execute = executeBTSequence,
    .composite = {
        .children = ManualTree_children,
        .child_count = 2,
        .current_child = 0
    }
};

// End manually designed tree