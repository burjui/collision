typedef struct {
    double2 topleft;
    double2 bottomright;
} AABB;

typedef enum {
    TAG_LEAF,
    TAG_TREE
} NodeTag;

typedef struct Tree {
    uint left;
    uint right;
} Tree;

typedef union {
    uint leaf_object_index;
    Tree tree;
} NodeData;

typedef struct  {
    AABB aabb;
    NodeTag tag;
    NodeData data;
} Node;

#pragma(inline)
bool intersects(const AABB *a, const AABB *b) {
    return a->topleft.x <= b->bottomright.x
        && a->bottomright.x >= b->topleft.x
        && a->topleft.y <= b->bottomright.y
        && a->bottomright.y >= b->topleft.y;
}

#define MAX_CANDIDATES 16

#pragma(inline)
uint find_intersections_with(
    uint root,
    uint object1_index,
    global const Node *nodes,
    global const AABB *object_aabbs,
    global const double2 *positions,
    global const double *radii,
    uint2 *candidates,
    uint candidate_index
) {
    const AABB object_aabb = object_aabbs[object1_index];
    const double2 object1_position = positions[object1_index];
    const double object1_radius = radii[object1_index];

    #define STACK_SIZE 32
    uint stack[STACK_SIZE];
    int sp = 0;
    stack[sp++] = root;

    // TODO fetch N nodes into private memory

    while (sp > 0) {
        uint node_id = stack[--sp];
        const Node node = nodes[node_id];
        if (intersects(&object_aabb, &node.aabb)) {
            if (node.tag == TAG_LEAF) {
                uint object2_index = node.data.leaf_object_index;
                if (object2_index != object1_index) {
                    const double2 object2_position = positions[object2_index];
                    const double object2_radius = radii[object2_index];
                    const double d = distance(object1_position, object2_position);
                    const double collision_distance = object1_radius + object2_radius;
                    if ((d < collision_distance) && (candidate_index < MAX_CANDIDATES)) {
                        const uint min_i = min(object1_index, object2_index);
                        const uint max_i = max(object1_index, object2_index);
                        candidates[candidate_index] = (uint2)(min_i, max_i);
                        candidate_index += 1;
                    }
                }
            } else {
                stack[sp] = node.data.tree.left;
                stack[sp + 1] = node.data.tree.right;
                sp += 2;
            }
        }
    }
    return candidate_index;
}

kernel void bvh_find_candidates(
    const uint root,
    global const Node *nodes,
    global const AABB *object_aabbs,
    global const double2 *positions,
    global const double *radii,
    global uint2 *global_candidates,
    const uint max_candidates
) {
    uint object1_index = get_global_id(0);
    uint2 candidates[MAX_CANDIDATES] = { 0 };
    uint candidates_end = find_intersections_with(
        root,
        object1_index,
        nodes,
        object_aabbs,
        positions,
        radii,
        candidates,
        0
    );
    uint offset = object1_index * MAX_CANDIDATES;
    for (uint i = 0; i < candidates_end; ++i) {
        global_candidates[offset + i] = candidates[i];
    }
}
