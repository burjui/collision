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

bool intersects(const AABB a, const AABB b) {
    return a.topleft.x <= b.bottomright.x
        && a.bottomright.x >= b.topleft.x
        && a.topleft.y <= b.bottomright.y
        && a.bottomright.y >= b.topleft.y;
}

#define MAX_CANDIDATES 16
#define STACK_SIZE 64
#define MAX_PRIVATE_AABBS ((uint) 3)

kernel void bvh_find_candidates(
    const uint root,
    global const Node *nodes,
    const uint node_count,
    global const AABB *object_aabbs,
    global const double2 *positions,
    global const double *radii,
    const uint object_count,
    global uint2 *candidates,
    volatile global uint *candidates_length
) {
    AABB private_node_aabbs[MAX_PRIVATE_AABBS];
    const uint private_node_aabb_count = min(MAX_PRIVATE_AABBS, node_count);
    for (uint i = 0; i < private_node_aabb_count; ++i) {
        private_node_aabbs[i] = nodes[i].aabb;
    }
    const uint object1_index = get_global_id(0);
    const AABB object_aabb = object_aabbs[object1_index];
    const double2 object1_position = positions[object1_index];
    const double object1_radius = radii[object1_index];

    uint stack[STACK_SIZE];
    int sp = 0;
    stack[sp++] = root;

    AABB aabb;
    while (sp > 0) {
        const uint node_id = stack[--sp];
        if (node_id < private_node_aabb_count) {
            aabb = private_node_aabbs[node_id];
        } else {
            aabb = nodes[node_id].aabb;
        }
        if (!intersects(object_aabb, aabb)) {
            continue;
        }

        const Node node = nodes[node_id];
        if (node.tag == TAG_LEAF) {
            const uint object2_index = node.data.leaf_object_index;
            if (object2_index != object1_index) {
                const double2 object2_position = positions[object2_index];
                const double object2_radius = radii[object2_index];
                const double d = distance(object1_position, object2_position);
                const double collision_distance = object1_radius + object2_radius;
                if (d < collision_distance) {
                    const uint min_i = min(object1_index, object2_index);
                    const uint max_i = max(object1_index, object2_index);
                    const uint index = atomic_add(candidates_length, 1);
                    if (index < object_count * MAX_CANDIDATES) {
                        candidates[index] = (uint2)(min_i, max_i);
                    }
                }
            }
        } else {
            stack[sp] = node.data.tree.left;
            stack[sp + 1] = node.data.tree.right;
            sp += 2;
        }
    }
}
