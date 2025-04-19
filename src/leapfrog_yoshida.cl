#define C1 0.675603595979829
#define C2 -0.1756035959798291
#define C3 -0.1756035959798291
#define C4 0.675603595979829
#define D1 1.351207191959658
#define D2 -1.7024143839193162
#define D3 1.351207191959658

double2 gravity_acceleration(
    uint object_index,
    const double2 position,
    const double2 global_gravity,
    global const double2 *restrict positions,
    constant double *restrict planet_masses,
    const uint planet_count,
    const double gravitational_constant
) {
    double2 gravity = global_gravity;
    for (uint planet_index = 0; planet_index < planet_count; ++planet_index) {
        const double2 planet_position = positions[planet_index];
        const double planet_mass = planet_masses[planet_index];
        if (planet_index != object_index) {
            const double2 delta = planet_position - position;
            const double r2 = dot(delta, delta);
            const double inv_r = native_rsqrt(r2);
            const double2 direction = delta * inv_r;
            const double factor = gravitational_constant * planet_mass / r2;
            gravity = fma(direction, (double2)(factor, factor), gravity);
        }
    }
    return gravity;
}

kernel void leapfrog_yoshida(
    global double2 *restrict positions,
    global double2 *restrict velocities,
    const uint object_count,
    const double dt,
    const double2 global_gravity,
    constant double *restrict planet_masses,
    const uint planet_count,
    const double gravitational_constant
) {
    const uint object_index = get_global_id(0);
    const double2 x0 = positions[object_index];
    const double2 v0 = velocities[object_index];
    const double2 x1 = fma(v0, C1 * dt, x0);
    const double2 a1 = gravity_acceleration(
        object_index, x1, global_gravity, positions, planet_masses, planet_count, gravitational_constant);
    const double2 v1 = fma(a1, D1 * dt, v0);
    const double2 x2 = fma(v1, C2 * dt, x0);
    const double2 a2 = gravity_acceleration(
        object_index, x2, global_gravity, positions, planet_masses, planet_count, gravitational_constant);
    const double2 v2 = fma(a2, D2 * dt, v0);
    const double2 x3 = fma(v2, C3 * dt, x0);
    const double2 a3 = gravity_acceleration(
        object_index, x3, global_gravity, positions, planet_masses, planet_count, gravitational_constant);
    const double2 v3 = fma(a3, D3 * dt, v0);
    positions[object_index] = fma(v3, C4 * dt, x0);
    velocities[object_index] = v3;
}
