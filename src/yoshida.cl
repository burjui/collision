#define C1 0.6756036
#define C2 -0.17560363
#define C3 -0.17560363
#define C4 0.6756036
#define D1 1.3512073
#define D2 -1.7024145
#define D3 1.3512073

float2 gravity_acceleration(size_t object_index, const float2 position,
                            global const float2 *positions,
                            global const float2 *velocities,
                            global const float *planet_masses,
                            const size_t planets_count,
                            const float gravitational_constant) {
  float2 gravity = (float2)(0.0, 0.0);
  for (size_t planet_index = 0; planet_index < planets_count; ++planet_index) {
    if (planet_index != object_index) {
      const float2 to_planet =
          positions[planet_index] - positions[object_index];
      const float distance_squared_unchecked =
          to_planet.x * to_planet.x + to_planet.y * to_planet.y;
      const float distance_squared =
          fmax(distance_squared_unchecked, (float)(0.000001));
      const float2 direction = to_planet / sqrt(distance_squared);
      const float planet_mass = planet_masses[planet_index];
      const float factor =
          gravitational_constant * planet_mass / distance_squared;
      gravity += direction * (float2)(factor, factor);
    }
  }

  return gravity;
}

kernel void yoshida(global float2 *positions, global float2 *velocities,
                    global const float *planet_masses,
                    const size_t planets_count,
                    const float gravitational_constant, const float dt) {
  const int object_index = get_global_id(0);
  const float2 x0 = positions[object_index];
  const float2 v0 = velocities[object_index];
  const float2 x1 = x0 + v0 * (float2)(C1 * dt, C1 * dt);
  const float2 a1 = gravity_acceleration(object_index, x1, positions,
                                         velocities, planet_masses,
                                         planets_count, gravitational_constant);
  const float2 v1 = v0 + a1 * (float2)(D1 * dt, D1 * dt);
  const float2 x2 = x0 + v1 * (float2)(C2 * dt, C2 * dt);
  const float2 a2 = gravity_acceleration(object_index, x2, positions,
                                         velocities, planet_masses,
                                         planets_count, gravitational_constant);
  const float2 v2 = v0 + a2 * (float2)(D2 * dt, D2 * dt);
  const float2 x3 = x0 + v2 * (float2)(C3 * dt, C3 * dt);
  const float2 a3 = gravity_acceleration(object_index, x3, positions,
                                         velocities, planet_masses,
                                         planets_count, gravitational_constant);
  const float2 v3 = v0 + a3 * (float2)(D3 * dt, D3 * dt);
  positions[object_index] = x0 + v3 * (float2)(C4 * dt, C4 * dt);
  velocities[object_index] = v3;
}
