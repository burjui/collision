typedef struct {
    double2 topleft;
    double2 bottomright;
} AABB;

typedef enum {
    TAG_LEAF,
    TAG_TREE
} NodeTag;


bool intersects(const AABB *a, const AABB *b) {
    return a->topleft.x <= b->bottomright.x
        && a->bottomright.x >= b->topleft.x
        && a->topleft.y <= b->bottomright.y
        && a->bottomright.y >= b->topleft.y;
}

#define MAX_CANDIDATES 16

uint find_intersections_with(
    uint root,
    uint object1_index,
    global const AABB *bvh_node_aabbs,
    global const NodeTag *bvh_node_tags,
    global const uint *bvh_node_leaf_indices,
    global const uint *bvh_node_tree_left,
    global const uint *bvh_node_tree_right,
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

    while (sp > 0) {
        uint node_id = stack[--sp];
        const AABB aabb = bvh_node_aabbs[node_id];
        if (intersects(&object_aabb, &aabb)) {
            if (bvh_node_tags[node_id] == TAG_LEAF) {
                uint object2_index = bvh_node_leaf_indices[node_id];
                if (object2_index != object1_index) {
                    const double2 object2_position = positions[object2_index];
                    const double object2_radius = radii[object2_index];
                    const double d = distance(object1_position, object2_position);
                    const double collision_distance = object1_radius + object2_radius;
                    if ((d < collision_distance) && (candidate_index < MAX_CANDIDATES)) {
                        uint min_i = min(object1_index, object2_index);
                        uint max_i = max(object1_index, object2_index);
                        candidates[candidate_index] = (uint2)(min_i, max_i);
                        candidate_index += 1;
                    }
                }
            } else {
                stack[sp] = bvh_node_tree_left[node_id];
                stack[sp + 1] = bvh_node_tree_right[node_id];
                sp += 2;
            }
        }
    }
    return candidate_index;
}

kernel void bvh_find_candidates(
    const uint root,
    global const AABB *bvh_node_aabbs,
    global const NodeTag *bvh_node_tags,
    global const uint *bvh_node_leaf_indices,
    global const uint *bvh_node_tree_left,
    global const uint *bvh_node_tree_right,
    global const AABB *object_aabbs,
    global const double2 *positions,
    global const double *radii,
    global uint2 *global_candidates,
    const uint max_candidates
) {
    uint object1_index = get_global_id(0);
    uint2 candidates[MAX_CANDIDATES];
    for (uint i = 0; i < MAX_CANDIDATES; ++i) {
        candidates[i] = (uint2)(0, 0);
    }
    uint candidates_end = find_intersections_with(
        root,
        object1_index,
        bvh_node_aabbs,
        bvh_node_tags,
        bvh_node_leaf_indices,
        bvh_node_tree_left,
        bvh_node_tree_right,
        object_aabbs,
        positions,
        radii,
        candidates,
        0
    );
    for (uint i = 0; i < candidates_end; ++i) {
        global_candidates[object1_index * max_candidates + i] = candidates[i];
    }
}
