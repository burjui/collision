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
    uchar tag;
    union {
        Leaf leaf;
        Tree tree;
    };
} NodeKind;

typedef struct {
    AABB aabb;
    NodeKind kind;
} Node;

bool intersects(AABB a, AABB b) {
    return a.topleft.x <= b.bottomright.x
        && a.bottomright.x >= b.topleft.x
        && a.topleft.y <= b.bottomright.y
        && a.bottomright.y >= b.topleft.y;
}

void find_intersections_with(
    uint object1_index,
    uint node_id,
    global const Node *bvh_nodes,
    global const AABB *object_aabbs,
    global double2 *positions,
    global double *radii,
    global uint2 *candidates,
    const uint max_candidates,
    uint candidates_count
) {
    AABB object_aabb = object_aabbs[object1_index];
    Node node = bvh_nodes[node_id];
    if (intersects(object_aabb, node.aabb)) {
        uint candidates_offset = object1_index * max_candidates + candidates_count;
        if (node.kind.tag == TAG_LEAF) {
            uint object2_index = node.kind.leaf.object2_index;
                if (object2_index != object1_index) {
                    const double2 object1_position = positions[object1_index];
                    const double2 object2_position = positions[object2_index];
                    const double object1_radius = radii[object1_index];
                    const double object2_radius = radii[object2_index];
                    const double d = distance(object1_position, object2_position);
                    const double collision_distance = object1_radius + object2_radius;
                    if ((d < collision_distance) && (candidates_count < max_candidates)) {
                        if (object1_index > object2_index) {
                            candidates[candidates_offset] = (uint2)(object2_index, object1_index);
                        } else {
                            candidates[candidates_offset] = (uint2)(object1_index, object2_index);
                        }
                        candidates_count += 1;
                    }
                }
        } else {
            find_intersections_with(object1_index, node.kind.tree.left, bvh_nodes, object_aabbs, positions, radii, candidates, max_candidates, candidates_offset);
            find_intersections_with(object1_index, node.kind.tree.right, bvh_nodes, object_aabbs, positions, radii, candidates, max_candidates, candidates_offset);
        }
    }
}

kernel void bvh_find_candidates(
    global const Node *bvh_nodes,
    const uint bvh_node_count,
    global const AABB *object_aabbs,
    global double2 *positions,
    global double *radii,
    global uint2 *candidates,
    const uint max_candidates
) {
    uint object1_index = get_global_id(0);
    uint candidates_offset = object1_index * max_candidates;
    uint root = bvh_node_count - 1;
    find_intersections_with(object1_index, root, bvh_nodes, object_aabbs, positions, radii, candidates, max_candidates, candidates_offset);
}
