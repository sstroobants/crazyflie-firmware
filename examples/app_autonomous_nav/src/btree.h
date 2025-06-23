#ifndef BTREE_H
#define BTREE_H

#include <stdbool.h>

// Return types
typedef enum {
    BT_SUCCESS,
    BT_FAILURE,
    BT_RUNNING
} BTStatus;

// Node types
typedef enum {
    BT_SEQUENCE,
    BT_SELECTOR,
    BT_LEAF
} BTNodeType;

typedef struct {
    bool pathClear;
} BTBlackboard;

struct BTNode;

// Node function type definition that returns a BT_STATUS from above enum
typedef BTStatus (*BTNodeFunc) (struct BTNode *node, BTBlackboard *bb);

typedef struct BTNode {
    BTNodeType type;
    BTNodeFunc execute;
    union {
        struct {
            struct BTNode **children;
            int child_count;
            int current_child;
        } composite;
        // Add custom leaf data if needed in future
    };
} BTNode;

BTStatus executeBTSequence(BTNode *node, BTBlackboard *bb);
BTStatus executeBTSelector(BTNode *node, BTBlackboard *bb);

// Expose pre-defined BTs
extern BTNode ManualTree;
#endif