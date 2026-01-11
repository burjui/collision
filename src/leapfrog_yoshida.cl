#define C1 (float)(0.675603595979829)
#define C2 (float)(-0.1756035959798291)
#define C3 (float)(-0.1756035959798291)
#define C4 (float)(0.675603595979829)
#define D1 (float)(1.351207191959658)
#define D2 (float)(-1.7024143839193162)
#define D3 (float)(1.351207191959658)

#pragma(inline)
float2 gravity_acceleration(uint object_index, const float2 position,
                            const float2 global_gravity,
                            global const float2 *restrict positions,
                            constant float *restrict planet_masses,
                            const uint planet_count,
                            const float gravitational_constant) {
  float2 gravity = global_gravity;
  for (uint planet_index = 0; planet_index < planet_count; ++planet_index) {
    const float2 planet_position = positions[planet_index];
    const float planet_mass = planet_masses[planet_index];
    if (planet_index != object_index) {
      const float2 delta = planet_position - position;
      const float r2 = dot(delta, delta);
      const float inv_r = native_rsqrt(r2);
      const float2 direction = delta * inv_r;
      const float inv_r2 = inv_r * inv_r;
      const float factor = gravitational_constant * planet_mass * inv_r2;
      gravity = fma(direction, (float2)(factor, factor), gravity);
    }
  }
  return gravity;
}

kernel void leapfrog_yoshida(global float2 *restrict positions,
                             global float2 *restrict velocities,
                             const uint object_count, const float dt,
                             const float2 global_gravity,
                             constant float *restrict planet_masses,
                             const uint planet_count,
                             const float gravitational_constant) {
  const uint object_index = get_global_id(0);
  const float2 x0 = positions[object_index];
  const float2 v0 = velocities[object_index];
  const float2 x1 = fma(v0, C1 * dt, x0);
  const float2 a1 =
      gravity_acceleration(object_index, x1, global_gravity, positions,
                           planet_masses, planet_count, gravitational_constant);
  const float2 v1 = fma(a1, D1 * dt, v0);
  const float2 x2 = fma(v1, C2 * dt, x1);
  const float2 a2 =
      gravity_acceleration(object_index, x2, global_gravity, positions,
                           planet_masses, planet_count, gravitational_constant);
  const float2 v2 = fma(a2, D2 * dt, v1);
  const float2 x3 = fma(v2, C3 * dt, x2);
  const float2 a3 =
      gravity_acceleration(object_index, x3, global_gravity, positions,
                           planet_masses, planet_count, gravitational_constant);
  const float2 v3 = fma(a3, D3 * dt, v0);
  positions[object_index] = fma(v3, C4 * dt, x3);
  velocities[object_index] = v3;
}
