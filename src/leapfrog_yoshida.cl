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
    const double2 *planet_positions,
    const double *planet_masses,
    const uint planet_count,
    const double gravitational_constant
) {
    double2 gravity = global_gravity;
    for (uint planet_index = 0; planet_index < planet_count; ++planet_index) {
        if (planet_index != object_index) {
            const double2 to_planet = planet_positions[planet_index] - position;
            const double2 coords_squared = to_planet * to_planet;
            const double distance_squared = coords_squared.x + coords_squared.y;
            const double2 direction = to_planet / sqrt(distance_squared);
            const double planet_mass = planet_masses[planet_index];
            const double factor =
                gravitational_constant * planet_mass / distance_squared;
                gravity += direction * (double2)(factor, factor);
        }
    }
    return gravity;
}

kernel void leapfrog_yoshida(
    global double2 *positions,
    global double2 *velocities,
    const uint object_count,
    const double dt,
    const double2 global_gravity,
    global const double *global_planet_masses,
    const uint planet_count,
    const double gravitational_constant
) {
    #define MAX_PLANETS 2

    const uint object_index = get_global_id(0);
    double2 planet_positions[MAX_PLANETS];
    double planet_masses[MAX_PLANETS];
    for (uint planet_index = 0; planet_index < planet_count; ++planet_index) {
        planet_positions[planet_index] = positions[planet_index];
        planet_masses[planet_index] = global_planet_masses[planet_index];
    }
    const double2 c1dt = (double2)(C1 * dt, C1 * dt);
    const double2 c2dt = (double2)(C2 * dt, C2 * dt);
    const double2 c3dt = (double2)(C3 * dt, C3 * dt);
    const double2 c4dt = (double2)(C4 * dt, C4 * dt);
    const double2 d1dt = (double2)(D1 * dt, D1 * dt);
    const double2 d2dt = (double2)(D2 * dt, D2 * dt);
    const double2 d3dt = (double2)(D3 * dt, D3 * dt);
    const double2 x0 = positions[object_index];
    const double2 v0 = velocities[object_index];
    const double2 x1 = x0 + v0 * c1dt;
    const double2 a1 = gravity_acceleration(
        object_index, x1, global_gravity, planet_positions, planet_masses, planet_count, gravitational_constant);
    const double2 v1 = v0 + a1 * d1dt;
    const double2 x2 = x0 + v1 * c2dt;
    const double2 a2 = gravity_acceleration(
        object_index, x2, global_gravity, planet_positions, planet_masses, planet_count, gravitational_constant);
    const double2 v2 = v0 + a2 * d2dt;
    const double2 x3 = x0 + v2 * c3dt;
    const double2 a3 = gravity_acceleration(
        object_index, x3, global_gravity, planet_positions, planet_masses, planet_count, gravitational_constant);
    const double2 v3 = v0 + a3 * d3dt;
    positions[object_index] = x0 + v3 * c4dt;
    velocities[object_index] = v3;
}
