use std::{path::Path, ptr::null};

use anyhow::{anyhow, Context as _, Ok};
use opencl3::{
    command_queue::CommandQueue,
    context::Context,
    device::{Device, CL_DEVICE_TYPE_GPU},
    kernel::ExecuteKernel,
    memory::{Buffer, CL_MEM_READ_WRITE},
    platform::get_platforms,
    program::Program,
    types::CL_TRUE,
};

pub struct Gpu {
    context: Context,
    queue: CommandQueue,
}

impl Gpu {
    pub fn default() -> anyhow::Result<Self> {
        for platform in get_platforms().context("No platforms found")? {
            let devices = platform
                .get_devices(CL_DEVICE_TYPE_GPU)
                .context("No GPU device found")?;
            if let Some(device_id) = devices.first().copied() {
                let device = Device::from(device_id);
                let context = Context::from_device(&device).context("Failed to create context")?;
                let queue = CommandQueue::create_default_with_properties(&context, 0, 1)
                    .context("Failed to create command queue")?;
                return Ok(Gpu { context, queue });
            }
        }
        Err(anyhow!("No GPU device found"))
    }

    pub fn build_program(&self, path: impl AsRef<Path>) -> anyhow::Result<Program> {
        let binary_path = path.as_ref().with_extension("bin");
        if let anyhow::Result::Ok(binary_metadata) = std::fs::metadata(&binary_path) {
            if let anyhow::Result::Ok(source_metadata) = std::fs::metadata(&path) {
                let source_modified = source_metadata
                    .modified()
                    .context("Failed to get source modified time")?;
                let binary_modified = binary_metadata
                    .modified()
                    .context("Failed to get binary modified time")?;
                if binary_modified >= source_modified {
                    return self.load_program_binary(binary_path);
                }
            }
        }
        let source = &std::fs::read_to_string(path).context("Failed to read source: {path:?}")?;
        let program = Program::create_and_build_from_source(&self.context, source, "").map_err(|e| anyhow!("{e}"))?;
        std::fs::write(binary_path, &program.get_binaries()?[0]).context("Failed to write binary: {binary_path:?}")?;
        Ok(program)
    }

    pub fn load_program_binary(&self, path: impl AsRef<Path>) -> anyhow::Result<Program> {
        let binary = &std::fs::read(path).unwrap();
        Program::create_and_build_from_binary(&self.context, &[binary], "").map_err(|e| anyhow!("{e}"))
    }

    pub fn create_buffer<T>(&self, length: usize) -> anyhow::Result<Buffer<T>> {
        unsafe { Buffer::create(&self.context, CL_MEM_READ_WRITE, length, null::<T>() as *mut _) }
            .context("Failed to create buffer")
    }

    pub fn write_buffer<T>(&self, buffer: &mut Buffer<T>, data: &[T]) -> anyhow::Result<()> {
        unsafe { self.queue.enqueue_write_buffer(buffer, CL_TRUE, 0, &data, &[]) }
            .context("Failed to write buffer")?
            .wait()
            .context("Failed to wait for write buffer event")
    }

    pub fn read_buffer<T>(&self, buffer: &Buffer<T>, data: &mut [T]) -> anyhow::Result<()> {
        unsafe { self.queue.enqueue_read_buffer(buffer, CL_TRUE, 0, data, &[]) }
            .context("Failed to read buffer")?
            .wait()
            .context("Failed to wait for read buffer event")
    }

    pub fn execute_kernel(&self, kernel: &mut ExecuteKernel) -> anyhow::Result<()> {
        unsafe { kernel.enqueue_nd_range(&self.queue) }
            .context("Failed to enqueue kernel")?
            .wait()
            .context("Failed to wait for kernel")
    }
}
