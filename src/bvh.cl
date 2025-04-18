typedef struct {
    double2 topleft;
    double2 bottomright;
} AABB;

enum {
    TAG_LEAF,
    TAG_TREE
};

typedef struct {
    uint object2_index;
} Leaf;

typedef struct {
    uint left;
    uint right;
} Tree;

typedef struct {
    uint tag;
    union {
        Leaf leaf;
        Tree tree;
    };
} NodeKind;

typedef struct {
    AABB aabb;
    NodeKind kind;
} Node;

bool intersects(const AABB *a, const AABB *b) {
    return a->topleft.x <= b->bottomright.x
        && a->bottomright.x >= b->topleft.x
        && a->topleft.y <= b->bottomright.y
        && a->bottomright.y >= b->topleft.y;
}

#define MAX_CANDIDATES 10

uint find_intersections_with(
    uint root,
    uint object1_index,
    global const Node *bvh_nodes,
    global const AABB *object_aabbs,
    global const double2 *positions,
    global const double *radii,
    uint2 *candidates,
    uint candidate_index
) {
    const AABB object_aabb = object_aabbs[object1_index];
    const double2 object1_position = positions[object1_index];
    const double object1_radius = radii[object1_index];

    #define STACK_SIZE 64
    uint stack[STACK_SIZE];
    int sp = 0;
    stack[sp++] = root;

    while (sp > 0) {
        uint node_id = stack[--sp];
        const Node node = bvh_nodes[node_id];
        if (intersects(&object_aabb, &node.aabb)) {
            if (node.kind.tag == TAG_LEAF) {
                uint object2_index = node.kind.leaf.object2_index;
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
                stack[sp++] = node.kind.tree.left;
                stack[sp++] = node.kind.tree.right;
            }
        }
    }
    return candidate_index;
}

kernel void bvh_find_candidates(
    const uint root,
    global const Node *bvh_nodes,
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
    uint candidates_end = find_intersections_with(root, object1_index, bvh_nodes, object_aabbs, positions, radii, candidates, 0);
    for (uint i = 0; i < candidates_end; ++i) {
        global_candidates[object1_index * max_candidates + i] = candidates[i];
    }
}
