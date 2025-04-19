use std::{
    iter::zip,
    time::{Duration, Instant},
};

use anyhow::Context;
use opencl3::kernel::{ExecuteKernel, Kernel};
use rand::{rng, seq::SliceRandom};
use rayon::{
    ThreadPool, ThreadPoolBuilder,
    iter::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator},
    slice::ParallelSliceMut,
};

use crate::{
    app_config::{CONFIG, DtSource},
    bvh::{AABB, Bvh, NodeId, NodeTag},
    gpu::{
        GPU,
        GpuBufferAccessMode::{self, ReadOnly, ReadWrite, WriteOnly},
        GpuDeviceBuffer,
    },
    object::{ObjectPrototype, ObjectSoa},
    ring_buffer::RingBuffer,
    vector2::Vector2,
};

pub struct PhysicsEngine {
    pub enable_constraint_bouncing: bool,
    objects: ObjectSoa,
    bvh: Bvh,
    candidates: Vec<Vec<NormalizedCollisionPair>>,
    candidates_flat: Vec<NormalizedCollisionPair>,
    time: f64,
    constraints: AABB,
    stats: Stats,
    restitution_coefficient: f64,
    global_gravity: Vector2<f64>,
    gravitational_constant: f64,
    gpu_integration_kernel: Kernel,
    gpu_leapfrog_positions: Option<GpuDeviceBuffer<Vector2<f64>>>,
    gpu_leapfrog_velocities: Option<GpuDeviceBuffer<Vector2<f64>>>,
    gpu_leapfrog_planet_masses: Option<GpuDeviceBuffer<f64>>,
    gpu_planet_masses_tmp: Vec<f64>,
    gpu_compute_options: GpuComputeOptions,
    thread_pool: ThreadPool,
    max_candidates_per_object: usize,
    gpu_bvh_kernel: Kernel,
    gpu_bvh_node_aabbs: Option<GpuDeviceBuffer<AABB>>,
    gpu_bvh_node_tags: Option<GpuDeviceBuffer<NodeTag>>,
    gpu_bvh_node_leaf_indices: Option<GpuDeviceBuffer<u32>>,
    gpu_bvh_node_tree_left: Option<GpuDeviceBuffer<NodeId>>,
    gpu_bvh_node_tree_right: Option<GpuDeviceBuffer<NodeId>>,
    gpu_bvh_object_aabbs: Option<GpuDeviceBuffer<AABB>>,
    gpu_bvh_object_positions: Option<GpuDeviceBuffer<Vector2<f64>>>,
    gpu_bvh_object_radii: Option<GpuDeviceBuffer<f64>>,
    gpu_bvh_object_candidates: Option<GpuDeviceBuffer<NormalizedCollisionPair>>,
}

impl PhysicsEngine {
    pub fn new() -> anyhow::Result<Self> {
        let constraints = AABB {
            topleft: Vector2::new(0.0, 0.0),
            bottomright: Vector2::new(f64::from(CONFIG.window.width), f64::from(CONFIG.window.height)),
        };
        let integration_program = GPU.build_program("src/leapfrog_yoshida.cl")?;
        let gpu_integration_kernel =
            Kernel::create(&integration_program, "leapfrog_yoshida").context("Failed to create kernel")?;
        let bvh_program = GPU.build_program("src/bvh.cl")?;
        let gpu_bvh_kernel = Kernel::create(&bvh_program, "bvh_find_candidates").context("Failed to create kernel")?;
        let thread_count = num_cpus::get();
        Ok(Self {
            enable_constraint_bouncing: true,
            objects: ObjectSoa::default(),
            bvh: Bvh::default(),
            candidates: vec![Vec::default(); thread_count],
            candidates_flat: Vec::default(),
            time: 0.0,
            constraints,
            stats: Stats::default(),
            restitution_coefficient: CONFIG.simulation.restitution_coefficient,
            global_gravity: Vector2::from(CONFIG.simulation.global_gravity),
            gravitational_constant: CONFIG.simulation.gravitational_constant,
            gpu_integration_kernel,
            gpu_leapfrog_positions: None,
            gpu_leapfrog_velocities: None,
            gpu_leapfrog_planet_masses: None,
            gpu_planet_masses_tmp: Vec::default(),
            gpu_compute_options: GpuComputeOptions::default(),
            thread_pool: ThreadPoolBuilder::new().num_threads(thread_count).build().unwrap(),
            max_candidates_per_object: 0,
            gpu_bvh_kernel,
            gpu_bvh_node_aabbs: None,
            gpu_bvh_node_tags: None,
            gpu_bvh_node_leaf_indices: None,
            gpu_bvh_node_tree_left: None,
            gpu_bvh_node_tree_right: None,
            gpu_bvh_object_aabbs: None,
            gpu_bvh_object_positions: None,
            gpu_bvh_object_radii: None,
            gpu_bvh_object_candidates: None,
        })
    }

    pub fn add(&mut self, object: ObjectPrototype) -> usize {
        self.objects.add(object)
    }

    #[must_use]
    pub fn objects(&self) -> &ObjectSoa {
        &self.objects
    }

    pub fn objects_mut(&mut self) -> &mut ObjectSoa {
        &mut self.objects
    }

    #[must_use]
    pub fn stats(&self) -> &Stats {
        &self.stats
    }

    #[must_use]
    pub fn stats_mut(&mut self) -> &mut Stats {
        &mut self.stats
    }

    #[must_use]
    pub fn time(&self) -> f64 {
        self.time
    }

    #[must_use]
    pub fn constraints(&self) -> AABB {
        self.constraints
    }

    pub fn bvh(&self) -> &Bvh {
        &self.bvh
    }

    pub fn bvh_mut(&mut self) -> &mut Bvh {
        &mut self.bvh
    }

    pub fn advance(&mut self, speed_factor: f64, gpu_compute_options: GpuComputeOptions) {
        if gpu_compute_options.integration != self.gpu_compute_options.integration {
            self.stats.integration_duration = DurationStat::default();
        }
        self.gpu_compute_options = gpu_compute_options;

        let start = Instant::now();
        let dt = match CONFIG.simulation.dt {
            DtSource::Auto => {
                // Using max_object_size instead of grid cell size to avoid an unnecessary grid update
                let (max_velocity_squared, min_object_size) =
                    zip(self.objects.velocities.iter(), self.objects.radii.iter()).fold(
                        (0.0_f64, f64::MAX),
                        |(max_velocity_squared, min_object_size), (velocity, radius)| {
                            (max_velocity_squared.max(velocity.magnitude_squared()), min_object_size.min(radius * 2.0))
                        },
                    );
                let velocity_factor = if min_object_size == f64::MAX {
                    1.0
                } else {
                    // Two times velocity because objects can collide head on
                    let current_velocity = max_velocity_squared.sqrt();
                    current_velocity * 2.0 / min_object_size
                };
                let max_gravity_squared = self.objects.positions.iter().enumerate().fold(
                    0.0_f64,
                    |max_gravity_squared, (object_index, &position)| {
                        let gravity_squared = Self::gravity_acceleration(
                            object_index,
                            position,
                            &self.objects.positions,
                            self.global_gravity,
                            self.gravitational_constant,
                            &self.objects.masses[self.objects.planet_range()],
                        )
                        .magnitude_squared();
                        max_gravity_squared.max(gravity_squared)
                    },
                );
                // Experimentally derived
                let gravity_factor =
                    max_gravity_squared.sqrt().sqrt().max(self.global_gravity.magnitude()) / min_object_size.sqrt();
                speed_factor / 2.0 * (1.0 / (velocity_factor + gravity_factor).max(1.0))
            }
            DtSource::Fixed(dt) => dt,
        };
        self.time += dt;
        self.update(dt, gpu_compute_options);

        self.stats.total_duration.update(start.elapsed());
        self.stats.sim_time = self.time;
        self.stats.object_count = self.objects.len();
    }

    fn update(&mut self, dt: f64, gpu_compute_options: GpuComputeOptions) {
        self.time += dt;

        let start = Instant::now();
        self.integrate(dt, gpu_compute_options);
        self.stats.integration_duration.update(start.elapsed());

        let start = Instant::now();
        self.bvh = Bvh::new(&self.objects.positions, &self.objects.radii);
        self.stats.bvh_duration.update(start.elapsed());

        let start = Instant::now();
        self.process_collisions();
        self.stats.collisions_duration.update(start.elapsed());

        let start = Instant::now();
        self.apply_constraints();
        self.stats.constraints_duration.update(start.elapsed());
    }

    fn integrate(&mut self, dt: f64, gpu_compute_options: GpuComputeOptions) {
        if gpu_compute_options.integration {
            self.integrate_gpu(dt);
        } else {
            self.integrate_cpu(dt);
        }
    }

    fn integrate_cpu(&mut self, dt: f64) {
        #[allow(clippy::unreadable_literal)]
        const CBRT2: f64 = 1.2599210498948732;
        const W0: f64 = -CBRT2 / (2.0 - CBRT2);
        const W1: f64 = 1.0 / (2.0 - CBRT2);
        const C1: f64 = 0.5 * W1;
        const C4: f64 = C1;
        const C2: f64 = 0.5 * (W0 + W1);
        const C3: f64 = C2;
        const D1: f64 = W1;
        const D3: f64 = D1;
        const D2: f64 = W0;

        let c1dt = C1 * dt;
        let c2dt = C2 * dt;
        let c3dt = C3 * dt;
        let c4dt = C4 * dt;
        let d1dt = D1 * dt;
        let d2dt = D2 * dt;
        let d3dt = D3 * dt;
        for object_index in 0..self.objects.len() {
            let x0 = self.objects.positions[object_index];
            let v0 = self.objects.velocities[object_index];
            let x1 = x0 + v0 * c1dt;
            let a1 = Self::gravity_acceleration(
                object_index,
                x1,
                &self.objects.positions,
                self.global_gravity,
                self.gravitational_constant,
                &self.objects.masses[self.objects.planet_range()],
            );
            let v1 = v0 + a1 * d1dt;
            let x2 = x0 + v1 * c2dt;
            let a2 = Self::gravity_acceleration(
                object_index,
                x2,
                &self.objects.positions,
                self.global_gravity,
                self.gravitational_constant,
                &self.objects.masses[self.objects.planet_range()],
            );
            let v2 = v0 + a2 * d2dt;
            let x3 = x0 + v2 * c3dt;
            let a3 = Self::gravity_acceleration(
                object_index,
                x3,
                &self.objects.positions,
                self.global_gravity,
                self.gravitational_constant,
                &self.objects.masses[self.objects.planet_range()],
            );
            let v3 = v0 + a3 * d3dt;
            self.objects.positions[object_index] = x0 + v3 * c4dt;
            self.objects.velocities[object_index] = v3;
        }
    }

    fn integrate_gpu(&mut self, dt: f64) {
        self.gpu_leapfrog_positions.init(self.objects.positions.len(), ReadWrite, "gpu_leapfrog_positions");
        self.gpu_leapfrog_velocities.init(self.objects.velocities.len(), ReadWrite, "gpu_leapfrog_velocities");
        self.gpu_planet_masses_tmp.resize(self.objects.planet_count + 1 /* cannot create an empty buffer */, 0.0);
        self.gpu_leapfrog_planet_masses.init(self.gpu_planet_masses_tmp.len(), ReadOnly, "gpu_leapfrog_planet_masses");
        self.execute_integration_kernel(dt);
    }

    fn execute_integration_kernel(&mut self, dt: f64) {
        self.write_integration_gpu_buffers();
        self.gpu_leapfrog_positions.as_mut().unwrap();
        self.gpu_leapfrog_velocities.as_mut().unwrap();
        self.gpu_leapfrog_planet_masses.as_mut().unwrap();
        let mut kernel = ExecuteKernel::new(&self.gpu_integration_kernel);
        kernel.set_global_work_size(self.objects.len());
        unsafe {
            self.gpu_leapfrog_positions.set_arg(&mut kernel);
            self.gpu_leapfrog_velocities.set_arg(&mut kernel);
            kernel.set_arg(&u32::try_from(self.objects.len()).unwrap());
            kernel.set_arg(&dt);
            kernel.set_arg(&self.global_gravity);
            self.gpu_leapfrog_planet_masses.set_arg(&mut kernel);
            kernel.set_arg(&u32::try_from(self.objects.planet_count).unwrap());
            kernel.set_arg(&self.gravitational_constant);
        }
        GPU.enqueue_execute_kernel(&mut kernel).context("Failed to execute kernel").unwrap();
        self.gpu_leapfrog_positions.enque_read(&mut self.objects.positions, "gpu_leapfrog_positions");
        self.gpu_leapfrog_velocities.enque_read(&mut self.objects.velocities, "gpu_leapfrog_velocities");
        GPU.wait_for_queue_completion().unwrap();
    }

    fn write_integration_gpu_buffers(&mut self) {
        self.gpu_leapfrog_positions.as_mut().unwrap();
        self.gpu_leapfrog_velocities.as_mut().unwrap();
        self.gpu_leapfrog_planet_masses.as_mut().unwrap();
        assert_eq!(self.gpu_leapfrog_positions.len(), self.objects.positions.len());
        assert_eq!(self.gpu_leapfrog_velocities.len(), self.objects.velocities.len());
        assert_eq!(self.gpu_leapfrog_planet_masses.len(), self.gpu_planet_masses_tmp.len());
        assert_eq!(self.gpu_planet_masses_tmp.len(), self.objects.planet_count + 1);
        self.gpu_leapfrog_positions.enque_write(&self.objects.positions, "gpu_leapfrog_positions");
        self.gpu_leapfrog_velocities.enque_write(&self.objects.velocities, "gpu_leapfrog_velocities");
        self.gpu_planet_masses_tmp[..self.objects.planet_count]
            .copy_from_slice(&self.objects.masses[self.objects.planet_range()]);
        self.gpu_leapfrog_planet_masses.enque_write(&self.gpu_planet_masses_tmp, "gpu_leapfrog_planet_masses");
    }

    fn gravity_acceleration(
        object_index: usize,
        position: Vector2<f64>,
        positions: &[Vector2<f64>],
        global_gravity: Vector2<f64>,
        gravitational_constant: f64,
        planet_masses: &[f64],
    ) -> Vector2<f64> {
        let mut gravity = global_gravity;
        for planet_index in 0..planet_masses.len() {
            if planet_index != object_index {
                let to_planet = positions[planet_index] - position;
                let direction = to_planet.normalize();
                gravity +=
                    direction * (gravitational_constant * planet_masses[planet_index] / to_planet.magnitude_squared());
            }
        }
        gravity
    }

    fn process_collisions(&mut self) {
        let start = Instant::now();
        if self.gpu_compute_options.bvh {
            self.find_collision_candidates_gpu();
        } else {
            Self::find_collision_candidates_cpu(
                &self.bvh,
                &self.thread_pool,
                &mut self.candidates,
                &mut self.candidates_flat,
                &self.objects.positions,
                &self.objects.radii,
            );
        };
        println!("found {} candidates in {:?}", self.candidates_flat.len(), start.elapsed());

        let start = Instant::now();
        self.thread_pool.install(|| self.candidates_flat.par_sort_unstable());
        println!("candidates sort {:?} ", start.elapsed());

        let start = Instant::now();
        let max_candidates_per_object =
            self.candidates_flat.chunk_by(|a, b| a.object1_index == b.object1_index).map(<[_]>::len).max().unwrap_or(0);
        self.max_candidates_per_object = self.max_candidates_per_object.max(max_candidates_per_object);
        println!("max candidates per object {} {:?}", self.max_candidates_per_object, start.elapsed());

        let start = Instant::now();
        let previous_length = self.candidates_flat.len();
        self.candidates_flat.dedup();
        println!("candidates dedup {} -> {} {:?}", previous_length, self.candidates_flat.len(), start.elapsed());

        let start = Instant::now();
        self.candidates_flat.shuffle(&mut rng());
        println!("candidates shuffle {:?} ", start.elapsed());

        let start = Instant::now();
        for &NormalizedCollisionPair {
            object1_index,
            object2_index,
        } in &self.candidates_flat
        {
            Self::process_collision_candidate(
                usize::try_from(object1_index).unwrap(),
                usize::try_from(object2_index).unwrap(),
                self.restitution_coefficient,
                &mut self.objects.positions,
                &mut self.objects.velocities,
                &self.objects.radii,
                &self.objects.masses,
                &self.objects.is_planet,
            );
        }
        println!("candidates processed {:?} ", start.elapsed());
    }

    fn find_collision_candidates_cpu<'a>(
        bvh: &Bvh,
        thread_pool: &ThreadPool,
        candidates: &'a mut Vec<Vec<NormalizedCollisionPair>>,
        candidates_flat: &mut Vec<NormalizedCollisionPair>,
        positions: &[Vector2<f64>],
        radii: &[f64],
    ) {
        let chunk_size = positions.len().div_ceil(thread_pool.current_num_threads());
        thread_pool.install(|| {
            candidates.par_iter_mut().enumerate().for_each(|(chunk_index, candidates)| {
                candidates.clear();
                for i in 0..chunk_size {
                    let object_index = chunk_index * chunk_size + i;
                    if object_index < positions.len() {
                        bvh.find_intersections(object_index, positions, radii, candidates);
                    }
                }
                candidates.sort_unstable();
                candidates.dedup();
            });
            candidates_flat.clear();
            candidates_flat.append(
                candidates
                    .par_iter_mut()
                    .reduce_with(|result, v| {
                        result.append(v);
                        result
                    })
                    .unwrap(),
            );
        })
    }

    fn find_collision_candidates_gpu(&mut self) {
        const MAX_CANDIDATES: u32 = 10;

        self.gpu_bvh_node_aabbs.init(self.bvh.node_aabbs().len(), ReadOnly, "gpu_bvh_node_aabbs");
        self.gpu_bvh_node_tags.init(self.bvh.node_tags().len(), ReadOnly, "gpu_bvh_node_tags");
        self.gpu_bvh_node_leaf_indices.init(self.bvh.node_leaf_indices().len(), ReadOnly, "gpu_bvh_node_leaf_indices");
        self.gpu_bvh_node_tree_left.init(self.bvh.node_tree_left().len(), ReadOnly, "gpu_bvh_node_tree_left");
        self.gpu_bvh_node_tree_right.init(self.bvh.node_tree_right().len(), ReadOnly, "gpu_bvh_node_tree_right");
        self.gpu_bvh_object_aabbs.init(self.bvh.object_aabbs().len(), ReadOnly, "gpu_bvh_object_aabbs");
        self.gpu_bvh_object_positions.init(self.objects.len(), ReadOnly, "gpu_bvh_object_positions");
        self.gpu_bvh_object_radii.init(self.objects.len(), ReadOnly, "gpu_bvh_object_radii");
        let candidates_length = self.objects.len() * usize::try_from(MAX_CANDIDATES).unwrap();
        self.gpu_bvh_object_candidates.init(candidates_length, WriteOnly, "gpu_bvh_object_candidates");

        self.gpu_bvh_node_aabbs.enque_write(self.bvh.node_aabbs(), "gpu_bvh_node_aabbs");
        self.gpu_bvh_node_tags.enque_write(self.bvh.node_tags(), "gpu_bvh_node_tags");
        self.gpu_bvh_node_leaf_indices.enque_write(self.bvh.node_leaf_indices(), "gpu_bvh_node_leaf_indices");
        self.gpu_bvh_node_tree_left.enque_write(self.bvh.node_tree_left(), "gpu_bvh_node_tree_left");
        self.gpu_bvh_node_tree_right.enque_write(self.bvh.node_tree_right(), "gpu_bvh_node_tree_right");
        self.gpu_bvh_object_aabbs.enque_write(self.bvh.object_aabbs(), "gpu_bvh_object_aabbs");
        self.gpu_bvh_object_positions.enque_write(&self.objects.positions, "gpu_bvh_object_positions");
        self.gpu_bvh_object_radii.enque_write(&self.objects.radii, "gpu_bvh_object_radii");
        // TODO zero the remainder on the GPU side instead
        self.candidates_flat.clear();
        self.candidates_flat.resize(candidates_length, NormalizedCollisionPair::new(0, 0));
        self.gpu_bvh_object_candidates.enque_write(&self.candidates_flat, "gpu_bvh_object_candidates");

        let mut kernel = ExecuteKernel::new(&self.gpu_bvh_kernel);
        kernel.set_global_work_size(self.objects.len());
        unsafe {
            kernel.set_arg(&self.bvh.root());
            self.gpu_bvh_node_aabbs.set_arg(&mut kernel);
            self.gpu_bvh_node_tags.set_arg(&mut kernel);
            self.gpu_bvh_node_leaf_indices.set_arg(&mut kernel);
            self.gpu_bvh_node_tree_left.set_arg(&mut kernel);
            self.gpu_bvh_node_tree_right.set_arg(&mut kernel);
            self.gpu_bvh_object_aabbs.set_arg(&mut kernel);
            self.gpu_bvh_object_positions.set_arg(&mut kernel);
            self.gpu_bvh_object_radii.set_arg(&mut kernel);
            self.gpu_bvh_object_candidates.set_arg(&mut kernel);
            kernel.set_arg(&MAX_CANDIDATES);
        }
        GPU.enqueue_execute_kernel(&mut kernel).context("Failed to execute kernel").unwrap();
        self.gpu_bvh_object_candidates.enque_read(&mut self.candidates_flat, "gpu_bvh_object_candidates");
        GPU.wait_for_queue_completion().unwrap();
        self.candidates_flat.retain(|pair| pair.object1_index > 0 || pair.object2_index > 0);
    }

    fn process_collision_candidate(
        object1_index: usize,
        object2_index: usize,
        restitution_coefficient: f64,
        positions: &mut [Vector2<f64>],
        velocities: &mut [Vector2<f64>],
        radii: &[f64],
        masses: &[f64],
        is_planet: &[bool],
    ) {
        let object1_position = positions[object1_index];
        let object2_position = positions[object2_index];
        let object1_radius = radii[object1_index];
        let object2_radius = radii[object2_index];
        let distance_squared = (object1_position - object2_position).magnitude_squared();
        let collision_distance = object1_radius + object2_radius;
        if distance_squared < collision_distance * collision_distance {
            Self::process_object_collision(
                object1_index,
                object2_index,
                distance_squared,
                collision_distance,
                restitution_coefficient,
                positions,
                velocities,
                masses,
                is_planet,
            );
        }
    }

    fn process_object_collision(
        object1_index: usize,
        object2_index: usize,
        distance_squared: f64,
        collision_distance: f64,
        restitution_coefficient: f64,
        positions: &mut [Vector2<f64>],
        velocities: &mut [Vector2<f64>],
        masses: &[f64],
        is_planet: &[bool],
    ) {
        let from_1_to_2 = positions[object1_index] - positions[object2_index];
        let distance = distance_squared.sqrt();

        // Compute collision normal
        let normal = from_1_to_2 / distance;

        let mass1 = masses[object1_index];
        let mass2 = masses[object2_index];
        let total_mass = mass1 + mass2;

        // Save the initial velocities for simultaneous impulse calculation.
        let v1_initial = velocities[object1_index];
        let v2_initial = velocities[object2_index];

        // Compute the impulse scalar using the original velocities.
        let impulse_scalar = 2.0 * (v1_initial - v2_initial).dot(normal) / total_mass;

        // Update velocities using the impulse
        let new_v1 = v1_initial - normal * mass2 * impulse_scalar;
        let new_v2 = v2_initial + normal * mass1 * impulse_scalar;

        // Apply restitution coefficient if the objects aren't planets.
        let corrected_v1 = if !is_planet[object1_index] {
            new_v1 * restitution_coefficient
        } else {
            new_v1
        };
        let corrected_v2 = if !is_planet[object2_index] {
            new_v2 * restitution_coefficient
        } else {
            new_v2
        };

        velocities[object1_index] = corrected_v1;
        velocities[object2_index] = corrected_v2;

        // Correct positions based on penetration depth using inverse masses.
        let intersection_depth = collision_distance - distance;
        let inv_mass1 = 1.0 / mass1;
        let inv_mass2 = 1.0 / mass2;
        let total_inv_mass = inv_mass1 + inv_mass2;
        let correction = normal * intersection_depth;
        positions[object1_index] += correction * (inv_mass1 / total_inv_mass);
        positions[object2_index] -= correction * (inv_mass2 / total_inv_mass);
    }

    fn apply_constraints(&mut self) {
        let cb = self.constraints;
        for ((position, velocity), radius) in
            zip(zip(&mut self.objects.positions, &mut self.objects.velocities), &self.objects.radii)
        {
            let initial_velocity = *velocity;
            if position.x - radius < cb.topleft.x {
                position.x = cb.topleft.x + radius;
                if self.enable_constraint_bouncing {
                    velocity.x *= -1.0;
                }
            } else if position.x + radius > cb.bottomright.x {
                position.x = cb.bottomright.x - radius;
                if self.enable_constraint_bouncing {
                    velocity.x *= -1.0;
                }
            }

            if position.y - radius < cb.topleft.y {
                position.y = cb.topleft.y + radius;
                if self.enable_constraint_bouncing {
                    velocity.y *= -1.0;
                }
            } else if position.y + radius > cb.bottomright.y {
                position.y = cb.bottomright.y - radius;
                if self.enable_constraint_bouncing {
                    velocity.y *= -1.0;
                }
            }

            if *velocity != initial_velocity {
                *velocity *= self.restitution_coefficient;
            }
        }
    }
}

trait GpuDeviceBufferUtils<T> {
    fn init(&mut self, length: usize, access_mode: GpuBufferAccessMode, name: &'static str) -> &mut GpuDeviceBuffer<T>;
    fn len(&self) -> usize;
    fn enque_write(&mut self, data: &[T], name: &'static str);
    fn enque_read(&mut self, data: &mut [T], name: &'static str);
    unsafe fn set_arg(&self, kernel: &mut ExecuteKernel);
}

impl<T> GpuDeviceBufferUtils<T> for Option<GpuDeviceBuffer<T>> {
    fn init(&mut self, length: usize, access_mode: GpuBufferAccessMode, name: &'static str) -> &mut GpuDeviceBuffer<T> {
        if self.as_ref().is_none_or(|b| b.len() != length) {
            self.replace(
                GPU.create_device_buffer(length, access_mode)
                    .with_context(|| format!("failed to create GPU device buffer {name}"))
                    .unwrap(),
            );
        }
        self.as_mut().unwrap()
    }

    fn len(&self) -> usize {
        self.as_ref().unwrap().len()
    }

    fn enque_write(&mut self, data: &[T], name: &'static str) {
        GPU.enqueue_write_device_buffer(self.as_mut().unwrap(), data)
            .with_context(|| format!("failed to enqueue write GPU device buffer {name}"))
            .unwrap();
    }

    fn enque_read(&mut self, data: &mut [T], name: &'static str) {
        GPU.enqueue_read_device_buffer(self.as_mut().unwrap(), data)
            .with_context(|| format!("failed to enqueue read GPU device buffer {name}"))
            .unwrap();
    }

    unsafe fn set_arg(&self, kernel: &mut ExecuteKernel) {
        unsafe { kernel.set_arg(self.as_ref().unwrap().buffer()) };
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GpuComputeOptions {
    pub integration: bool,
    pub bvh: bool,
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct NormalizedCollisionPair {
    object1_index: u32,
    object2_index: u32,
}

impl NormalizedCollisionPair {
    pub fn new(object1_index: usize, object2_index: usize) -> Self {
        let object1_index = u32::try_from(object1_index).unwrap();
        let object2_index = u32::try_from(object2_index).unwrap();
        Self {
            object1_index: object1_index.min(object2_index),
            object2_index: object1_index.max(object2_index),
        }
    }
}

#[derive(Clone, Debug)]
pub struct DurationStat {
    pub current: Duration,
    pub lowest: Duration,
    pub highest: Duration,
    pub average: RingBuffer<32, Duration>,
}

impl DurationStat {
    pub fn update(&mut self, duration: Duration) {
        self.current = duration;
        self.lowest = self.lowest.min(duration);
        self.highest = self.highest.max(duration);
        self.average.push(duration);
    }
}

impl Default for DurationStat {
    fn default() -> Self {
        Self {
            current: Duration::ZERO,
            lowest: Duration::MAX,
            highest: Duration::ZERO,
            average: RingBuffer::default(),
        }
    }
}

#[derive(Default, Clone, Debug)]
pub struct Stats {
    pub sim_time: f64,
    pub object_count: usize,
    pub integration_duration: DurationStat,
    pub bvh_duration: DurationStat,
    pub collisions_duration: DurationStat,
    pub constraints_duration: DurationStat,
    pub total_duration: DurationStat,
}
