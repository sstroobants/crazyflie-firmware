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

BTStatus turnRightBT(BTNode *node, void *context)
{
    return BT_SUCCESS;
}