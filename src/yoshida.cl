#define C1 0.675603595979829
#define C2 -0.1756035959798291
#define C3 -0.1756035959798291
#define C4 0.675603595979829
#define D1 1.351207191959658
#define D2 -1.7024143839193162
#define D3 1.351207191959658

double2 gravity_acceleration(size_t object_index, const double2 position,
                             global const double2 *positions,
                             global const double2 *velocities,
                             const double2 global_gravity,
                             constant double *planet_masses,
                             const size_t planets_count,
                             const double gravitational_constant) {
  double2 gravity = global_gravity;
  for (size_t planet_index = 0; planet_index < planets_count; ++planet_index) {
    if (planet_index != object_index) {
      const double2 to_planet =
          positions[planet_index] - positions[object_index];
      const double distance_squared_unchecked =
          to_planet.x * to_planet.x + to_planet.y * to_planet.y;
      const double distance_squared =
          fmax(distance_squared_unchecked, (double)(0.000001));
      const double2 direction = to_planet / sqrt(distance_squared);
      const double planet_mass = planet_masses[planet_index];
      const double factor =
          gravitational_constant * planet_mass / distance_squared;
      gravity += direction * (double2)(factor, factor);
    }
  }

  return gravity;
}

kernel void yoshida_no_planets(global double2 *positions,
                               global double2 *velocities, const double dt,
                               const double2 global_gravity) {
  const int object_index = get_global_id(0);
  const double2 x0 = positions[object_index];
  const double2 v0 = velocities[object_index];
  const double2 x1 = x0 + v0 * (double2)(C1 * dt, C1 * dt);
  const double2 v1 = v0 + global_gravity * (double2)(D1 * dt, D1 * dt);
  const double2 x2 = x0 + v1 * (double2)(C2 * dt, C2 * dt);
  const double2 v2 = v0 + global_gravity * (double2)(D2 * dt, D2 * dt);
  const double2 x3 = x0 + v2 * (double2)(C3 * dt, C3 * dt);
  const double2 v3 = v0 + global_gravity * (double2)(D3 * dt, D3 * dt);
  positions[object_index] = x0 + v3 * (double2)(C4 * dt, C4 * dt);
  velocities[object_index] = v3;
}

kernel void yoshida(global double2 *positions, global double2 *velocities,
                    const double dt, const double2 global_gravity,
                    constant double *planet_masses, const size_t planets_count,
                    const double gravitational_constant) {
  const int object_index = get_global_id(0);
  const double2 x0 = positions[object_index];
  const double2 v0 = velocities[object_index];
  const double2 x1 = x0 + v0 * (double2)(C1 * dt, C1 * dt);
  const double2 a1 = gravity_acceleration(
      object_index, x1, positions, velocities, global_gravity, planet_masses,
      planets_count, gravitational_constant);
  const double2 v1 = v0 + a1 * (double2)(D1 * dt, D1 * dt);
  const double2 x2 = x0 + v1 * (double2)(C2 * dt, C2 * dt);
  const double2 a2 = gravity_acceleration(
      object_index, x2, positions, velocities, global_gravity, planet_masses,
      planets_count, gravitational_constant);
  const double2 v2 = v0 + a2 * (double2)(D2 * dt, D2 * dt);
  const double2 x3 = x0 + v2 * (double2)(C3 * dt, C3 * dt);
  const double2 a3 = gravity_acceleration(
      object_index, x3, positions, velocities, global_gravity, planet_masses,
      planets_count, gravitational_constant);
  const double2 v3 = v0 + a3 * (double2)(D3 * dt, D3 * dt);
  positions[object_index] = x0 + v3 * (double2)(C4 * dt, C4 * dt);
  velocities[object_index] = v3;
}
