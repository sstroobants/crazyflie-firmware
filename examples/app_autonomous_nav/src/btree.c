#include <stdio.h>
#include "btree.h"


// ========= Composite Nodes ===========

BTStatus executeBTSequence(BTNode *node, void *context)
{
    while (node->composite.current_child < node->composite.child_count) {
            BTNode *child = node->composite.children[node->composite.current_child];
            BTStatus status = child->execute(child, context);
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


BTStatus executeBTSelector(BTNode *node, void *context)
{
    while (node->composite.current_child < node->composite.child_count) {
            BTNode *child = node->composite.children[node->composite.current_child];
            BTStatus status = child->execute(child, context);
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

BTStatus turnRightBT(BTNode *node, void *context)
{
    return BT_SUCCESS;
}

BTStatus turnLeftBT(BTNode *node, void *context)
{
    return BT_SUCCESS;
}

BTStatus moveForwardBT(BTNode *node, void *context)
{
    return BT_SUCCESS;
}


// Conditions

BTStatus randomConditionBT(BTNode *node, void *context)
{
    return BT_SUCCESS;
}

BTStatus pathClearBT(BTNode *node, void *context)
{
    return BT_SUCCESS;
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