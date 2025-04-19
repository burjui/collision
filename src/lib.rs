#![allow(clippy::too_many_arguments)]
#![allow(clippy::too_many_lines)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::missing_panics_doc)]
#![feature(random)]
#![feature(impl_trait_in_bindings)]
#![feature(array_repeat)]
#![feature(array_windows)]
#![feature(more_float_constants)]
#![feature(lazy_get)]
#![feature(maybe_uninit_array_assume_init)]
#![feature(coroutines, coroutine_trait)]
#![feature(array_try_map)]
#![feature(iter_chain)]
#![feature(array_chunks)]

pub mod app_config;
pub mod array2;
pub mod bvh;
pub mod demo;
pub mod fixed_vec;
pub mod fps;
pub mod gpu;
pub mod object;
pub mod physics;
pub mod ring_buffer;
pub mod simple_text;
pub mod vector2;
