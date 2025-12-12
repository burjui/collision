typedef struct __attribute__((__packed__)) {
    float2 topleft;
    float2 bottomright;
} AABB;

typedef enum {
    TAG_LEAF = 0,
    TAG_TREE = 1
} NodeTag;

typedef struct __attribute__((__packed__))  {
    uint left;
    uint right;
} Tree;

typedef union __attribute__((__packed__)) {
    uint leaf_object_index;
    Tree tree;
} NodeData;

typedef struct __attribute__((__packed__)) {
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

void print_node(global const Node *node) {
    printf("[OpenCL node] tag: %d, aabb: %f %f %f %f\n", node->tag,
        node->aabb.topleft.x, node->aabb.topleft.y,
        node->aabb.bottomright.x, node->aabb.bottomright.y
    );
}

#define MAX_CANDIDATES 16
#define STACK_SIZE 64

kernel void bvh_find_candidates(
    const uint root,
    global const Node *nodes,
    const uint node_count,
    global const float2 *positions,
    global const float *radii,
    const uint object_count,
    global uint2 *candidates,
    volatile global uint *candidates_length,
    volatile global uint *errors
) {
    const uint object1_index = get_global_id(0);
    if (object1_index == 0) {
        print_node(&nodes[0]);
        print_node(&nodes[1]);
        printf("[OpenCL] sizeof(Node): %d\n", sizeof(Node));
        printf("[OpenCL] sizeof(NodeTag): %d\n", sizeof(NodeTag));
        printf("[OpenCL] sizeof(NodeData): %d\n", sizeof(NodeData));
        printf("[OpenCL] sizeof(Tree): %d\n", sizeof(Tree));
        printf("[OpenCL] sizeof(AABB): %d\n", sizeof(AABB));
    }

    const AABB object_aabb = nodes[object1_index].aabb;
    const float2 object1_position = positions[object1_index];
    const float object1_radius = radii[object1_index];

    uint stack[STACK_SIZE];
    int sp = 0;
    stack[sp++] = root;

    AABB aabb;
    while (sp > 0) {
        const uint node_id = stack[--sp];
        const Node node = nodes[node_id];
        if (!intersects(object_aabb, node.aabb)) {
            continue;
        }

        if (node.tag == TAG_LEAF) {
            const uint object2_index = node.data.leaf_object_index;
            if (object2_index != object1_index) {
                const float2 object2_position = positions[object2_index];
                const float object2_radius = radii[object2_index];
                const float d = distance(object1_position, object2_position);
                const float collision_distance = object1_radius + object2_radius;
                if (object1_index == 0) {
                    printf("[OpenCL] d: %f, collision_distance: %f\n", d, collision_distance);
                }
                if (d < collision_distance) {
                    const uint min_i = min(object1_index, object2_index);
                    const uint max_i = max(object1_index, object2_index);
                    const uint index = atomic_add(candidates_length, 1);
                    if (index < object_count * MAX_CANDIDATES) {
                        candidates[index] = (uint2)(min_i, max_i);
                    } else {
                        atomic_add(errors, 1);
                    }
                }
            }
        } else {
            if (object1_index == 0) {
                printf("[OpenCL] left: %d, right: %d\n", node.data.tree.left, node.data.tree.right);
            }
            if (sp + 2 < STACK_SIZE) {
                stack[sp] = node.data.tree.left;
                stack[sp + 1] = node.data.tree.right;
                sp += 2;
            } else {
                atomic_add(errors, 1);
            }
        }
    }
}
