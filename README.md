An experiment in physics particle system simulation. Uses 4th order Leapfrog
integrator with Yoshida coefficients and preserves momentum on collisions if
`restitution_coefficient = 1` (see config.toml). Integration and broad phase
collision detection are implemented both on CPU and GPU (OpenCL); the
implementations can be switched at runtime using keys. GPU-accelerated rendering
is powered by [`vello`](https://github.com/linebender/vello).

This project is practically abandoned in favor of
[`collision2`](https://github.com/burjui/collision2), which is a complete
rewrite.
