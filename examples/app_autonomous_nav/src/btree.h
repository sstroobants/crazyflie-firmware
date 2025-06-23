#ifndef BTREE_H
#define BTREE_H

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

struct BTNode;

// Node function type definition that returns a BT_STATUS from above enum
typedef BTStatus (*BTNodeFunc) (struct BTNode *node, void *context);

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


BTStatus executeBTSequence(BTNode *node, void *context);
BTStatus executeBTSelector(BTNode *node, void *context);

// Expose pre-defined BTs
extern BTNode ManualTree;
#endif