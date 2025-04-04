<blockquote><sub>"Collision! My misssion!"<br/>Faith No More</sub></blockquote>

This is an experiment in particle system simulation.
Currently uses 4th order Leapfrog integrator with Yoshida coefficients and preserves momentum on collisions if `restitution_coefficient = 1` (see config.toml). Runs on CPU, can use OpenCL for Leapfrog (collisions — not yet). Rendering is done on GPU using [`vello`](https://github.com/linebender/vello). 
