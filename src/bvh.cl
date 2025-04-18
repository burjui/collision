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

void find_intersections_with(
    uint node_id,
    uint object1_index,
    global const Node *bvh_nodes,
    global const AABB *object_aabbs,
    global const double2 *positions,
    global const double *radii,
    uint2 *candidates,
    const uint2 *candidates_end
) {
    const Node node = bvh_nodes[node_id];
    const AABB object_aabb = object_aabbs[object1_index];
    const double2 object1_position = positions[object1_index];
    const double object1_radius = radii[object1_index];
    if (intersects(&object_aabb, &node.aabb)) {
        if (node.kind.tag == TAG_LEAF) {
            uint object2_index = node.kind.leaf.object2_index;
                if (object2_index != object1_index) {
                    const double2 object2_position = positions[object2_index];
                    const double object2_radius = radii[object2_index];
                    const double d = distance(object1_position, object2_position);
                    const double collision_distance = object1_radius + object2_radius;
                    if ((d < collision_distance) && (candidates < candidates_end)) {
                        if (object1_index < object2_index) {
                            *candidates = (uint2)(object1_index, object2_index);
                        } else {
                            *candidates = (uint2)(object2_index, object1_index);
                        }
                        candidates += 1;
                    }
                }
        } else {
            find_intersections_with(node.kind.tree.left, object1_index, bvh_nodes, object_aabbs, positions, radii, candidates, candidates_end);
            find_intersections_with(node.kind.tree.right, object1_index, bvh_nodes, object_aabbs, positions, radii, candidates, candidates_end);
        }
    }
}

kernel void bvh_find_candidates(
    const uint root,
    global const Node *bvh_nodes,
    global const AABB *object_aabbs,
    global const double2 *positions,
    global const double *radii,
    global uint2 *candidates,
    const uint max_candidates
) {
    uint object1_index = get_global_id(0);
    uint candidates_count = 0;
    uint2 private_candidates[10];
    for (uint i = 0; i < 10; ++i) {
        private_candidates[i] = (uint2)(0, 0);
    }
    find_intersections_with(root, object1_index, bvh_nodes, object_aabbs, positions, radii, private_candidates, private_candidates + candidates_count);
    for (uint i = 0; i < 10; ++i) {
        candidates[object1_index * max_candidates + i] = private_candidates[i];
    }
}
