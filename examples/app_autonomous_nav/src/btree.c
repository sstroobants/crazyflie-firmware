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
    while (node->composite.current_child < node->composite.child_count)
    {
        BTNode *child = node->composite.children[node->composite.current_child];
        BTStatus status = child->execute(child, bb);
        // DEBUG_PRINT("A sequences's child returned %d\n", status);

        if (status == BT_RUNNING)
        {
            node->composite.current_child = 0;

            // DEBUG_PRINT("A sequence is running\n");
            return BT_RUNNING;
        }
        if (status == BT_FAILURE)
        {
            node->composite.current_child = 0;
            // DEBUG_PRINT("A sequence failed\n");

            return BT_FAILURE;
        }
        node->composite.current_child++;
    }
    node->composite.current_child = 0;
    // DEBUG_PRINT("A sequence succeeded\n");

    return BT_SUCCESS;
}

BTStatus executeBTSelector(BTNode *node, BTBlackboard *bb)
{
    while (node->composite.current_child < node->composite.child_count)
    {
        BTNode *child = node->composite.children[node->composite.current_child];
        BTStatus status = child->execute(child, bb);
        // DEBUG_PRINT("A selector's child returned %d\n", status);
        if (status == BT_RUNNING)
        {
            // DEBUG_PRINT("A selector is running\n");
            node->composite.current_child = 0;

            return BT_RUNNING;
        }
        if (status == BT_SUCCESS)
        {
            // DEBUG_PRINT("A selector succeeded\n");

            node->composite.current_child = 0;
            return BT_SUCCESS;
        }
        node->composite.current_child++;
    }
    node->composite.current_child = 0;
    // DEBUG_PRINT("A selector failed\n");

    return BT_FAILURE;
}

// ======== Leaf Node Functions =======

// Actions

BTStatus turnRightBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Action Turn Right\n");
    bb->r_cmd = -TURN_RATE;
    return BT_RUNNING;
}

BTStatus turnLeftBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Action Turn Left\n");
    bb->r_cmd = TURN_RATE;
    return BT_RUNNING;
}

BTStatus moveForwardBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Action Move Fwd\n");
    bb->vx_cmd = FWD_VEL;
    bb->r_cmd = 0.0f;
    return BT_RUNNING;
}

BTStatus turnLeftAndForwardBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Action Turn Left and Move Fwd\n");
    bb->vx_cmd = FWD_VEL;
    bb->r_cmd = TURN_RATE;
    return BT_RUNNING;
}





// Conditions

BTStatus randomConditionBT(BTNode *node, BTBlackboard *bb)
{
    DEBUG_PRINT("Condition Random\n");
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
    if (bb->pathClear)
    {
        return BT_SUCCESS;
    }
    else
    {
        DEBUG_PRINT("Path obstructed \n");

        return BT_FAILURE;
    }
}

BTStatus leftOverRightBT(BTNode *node, BTBlackboard *bb)
{
    if (bb->leftDist > bb->rightDist)
    {
        DEBUG_PRINT("Left is freer than right\n");
        return BT_SUCCESS;
    }
    else
    {
        DEBUG_PRINT("Right is freer than left\n");
        return BT_FAILURE;
    }
}

BTStatus withinPeerRange(BTNode *node, BTBlackboard *bb)
{
    // Check if the peer distance is within a certain range
    if (bb->peerDist < 1.5f) 
    {
        DEBUG_PRINT("Peer is within range\n");
        return BT_SUCCESS;
    }
    else
    {
        DEBUG_PRINT("Peer is out of range\n");
        return BT_FAILURE;
    }
}

// ==== Usable Behaviour Trees ======

// Manually designed tree
// static BTNode lr_random_node = {.type = BT_LEAF, .execute = randomConditionBT};
static BTNode lr_smart_node = {.type = BT_LEAF, .execute = leftOverRightBT};
static BTNode pathclear_node = {.type = BT_LEAF, .execute = pathClearBT};
static BTNode turnleft_node = {.type = BT_LEAF, .execute = turnLeftBT};
static BTNode turnright_node = {.type = BT_LEAF, .execute = turnRightBT};
static BTNode moveforward_node = {.type = BT_LEAF, .execute = moveForwardBT};
static BTNode fwd_left_node = {.type = BT_LEAF, .execute = turnLeftAndForwardBT};
static BTNode peersphere_node = {.type = BT_LEAF, .execute = withinPeerRange};


static BTNode *lr_sel_children[] = {&lr_smart_node, &turnright_node};
BTNode lr_sel = {
    .type = BT_SELECTOR,
    .execute = executeBTSelector,
    .composite = {
        .children = lr_sel_children,
        .child_count = 2,
        .current_child = 0}};

static BTNode *lr_seq_children[] = {&lr_sel, &turnleft_node};
BTNode lr_seq = {
    .type = BT_SEQUENCE,
    .execute = executeBTSequence,
    .composite = {
        .children = lr_seq_children,
        .child_count = 2,
        .current_child = 0}};

static BTNode *obs_avoidance_sel_children[] = {&pathclear_node, &lr_seq};
BTNode obs_avoidance_sel = {
    .type = BT_SELECTOR,
    .execute = executeBTSelector,
    .composite = {
        .children = obs_avoidance_sel_children,
        .child_count = 2,
        .current_child = 0}};

static BTNode *peer_sphere_sel_children[] = {&peersphere_node, &fwd_left_node};
BTNode peer_sphere_sel = {
    .type = BT_SELECTOR,
    .execute = executeBTSelector,
    .composite = {
        .children = peer_sphere_sel_children,
        .child_count = 2,
        .current_child = 0}};

static BTNode *ManualTree_children[] = {&peer_sphere_sel, &obs_avoidance_sel, &moveforward_node};
BTNode ManualTree = {
    .type = BT_SEQUENCE,
    .execute = executeBTSequence,
    .composite = {
        .children = ManualTree_children,
        .child_count = 3,
        .current_child = 0}};

// static BTNode *ManualTree_children[] = {&lr_random_node, &moveforward_node};
// BTNode ManualTree = {
//     .type = BT_SEQUENCE,
//     .execute = executeBTSequence,
//     .composite = {
//         .children = ManualTree_children,
//         .child_count = 2,
//         .current_child = 0
//     }
// };

// End manually designed tree