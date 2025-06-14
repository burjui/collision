use std::{path::Path, ptr::null_mut, sync::LazyLock};

use anyhow::{Context as _, Ok, anyhow};
use opencl3::{
    command_queue::CommandQueue,
    context::Context,
    device::{CL_DEVICE_TYPE_GPU, Device},
    event::Event,
    kernel::ExecuteKernel,
    memory::{Buffer, CL_MEM_READ_ONLY, CL_MEM_READ_WRITE, CL_MEM_USE_HOST_PTR, CL_MEM_WRITE_ONLY},
    platform::get_platforms,
    program::Program,
    types::{CL_FALSE, cl_mem_flags},
};

pub static GPU: LazyLock<Gpu> = LazyLock::new(|| Gpu::first_available(20).unwrap());

pub struct Gpu {
    context: Context,
    queue: CommandQueue,
}

// TODO don't return results (there's no point)
impl Gpu {
    pub fn first_available(queue_length: u32) -> anyhow::Result<Self> {
        let platforms = get_platforms().context("No platforms found")?;
        println!("Available OpenCL platforms:");
        for (i, platform) in platforms.iter().enumerate() {
            println!(
                "Platform {i}: {} {}",
                platform.name().context("Failed to get platform name")?,
                platform.version().context("Failed to get platform name")?
            );
            let devices = platform.get_devices(CL_DEVICE_TYPE_GPU).context("No GPU device found")?;
            for (i, &device_id) in devices.iter().enumerate() {
                let device = Device::from(device_id);
                println!("  Device {i}: {}", device.name().context("Failed to get device name")?);
            }
        }
        for platform in platforms {
            let devices = platform.get_devices(CL_DEVICE_TYPE_GPU).context("No GPU device found")?;
            if let Some(device_id) = devices.first().copied() {
                let device = Device::from(device_id);
                let context = Context::from_device(&device).context("Failed to create context")?;
                let queue = CommandQueue::create_default_with_properties(&context, 0, queue_length)
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
                let source_modified = source_metadata.modified().context("Failed to get source modified time")?;
                let binary_modified = binary_metadata.modified().context("Failed to get binary modified time")?;
                if binary_modified >= source_modified {
                    return self.load_program_binary(binary_path);
                }
            }
        }
        println!("Building OpenCL kernel: {}", path.as_ref().display());
        let source = &std::fs::read_to_string(path).context("Failed to read source: {path:?}")?;
        let program = Program::create_and_build_from_source(&self.context, source, "").map_err(|e| anyhow!("{e}"))?;
        std::fs::write(binary_path, &program.get_binaries()?[0]).context("Failed to write binary: {binary_path:?}")?;
        Ok(program)
    }

    pub fn load_program_binary(&self, path: impl AsRef<Path>) -> anyhow::Result<Program> {
        let binary = &std::fs::read(path).unwrap();
        Program::create_and_build_from_binary(&self.context, &[binary], "").map_err(|e| anyhow!("{e}"))
    }

    pub unsafe fn create_host_ptr_buffer<T>(
        &self,
        data: &mut [T],
        access_mode: GpuBufferAccessMode,
    ) -> anyhow::Result<GpuHostPtrBuffer<T>> {
        let buffer = unsafe {
            Buffer::create(
                &self.context,
                access_mode.cl_mem_flags() | CL_MEM_USE_HOST_PTR,
                data.len(),
                data.as_ptr() as *mut _,
            )
        }
        .context("Failed to create host ptr buffer")?;
        Ok(GpuHostPtrBuffer {
            buffer,
            length: data.len(),
        })
    }

    pub fn create_host_buffer<T>(
        &self,
        data: Vec<T>,
        access_mode: GpuBufferAccessMode,
    ) -> anyhow::Result<GpuHostBuffer<T>> {
        let buffer = unsafe {
            Buffer::create(
                &self.context,
                access_mode.cl_mem_flags() | CL_MEM_USE_HOST_PTR,
                data.len(),
                data.as_ptr() as *mut _,
            )
        }
        .context("Failed to create host buffer")?;
        Ok(GpuHostBuffer { data, buffer })
    }

    pub fn create_device_buffer<T>(
        &self,
        length: usize,
        access_mode: GpuBufferAccessMode,
    ) -> anyhow::Result<GpuDeviceBuffer<T>> {
        let buffer = unsafe { Buffer::create(&self.context, access_mode.cl_mem_flags(), length, null_mut()) }
            .context("Failed to create device buffer")?;
        Ok(GpuDeviceBuffer { buffer, length })
    }

    pub fn enqueue_write_device_buffer<T>(&self, buffer: &mut GpuDeviceBuffer<T>, data: &[T]) -> anyhow::Result<Event> {
        unsafe {
            self.queue
                .enqueue_write_buffer(&mut buffer.buffer, CL_FALSE, 0, data, &[])
                .context("Failed to write device buffer")
        }
    }

    pub fn enqueue_read_device_buffer<T>(&self, buffer: &GpuDeviceBuffer<T>, dst: &mut [T]) -> anyhow::Result<Event> {
        unsafe { self.queue.enqueue_read_buffer(&buffer.buffer, CL_FALSE, 0, dst, &[]) }
            .context("Failed to read device buffer")
    }

    pub fn enqueue_execute_kernel(&self, kernel: &mut ExecuteKernel) -> anyhow::Result<Event> {
        unsafe { kernel.enqueue_nd_range(&self.queue) }.context("Failed to enqueue kernel")
    }

    pub fn wait_for_queue_completion(&self) -> anyhow::Result<()> {
        self.queue.finish().context("Failed to submit queue")
    }
}

#[derive(Copy, Clone)]
pub enum GpuBufferAccessMode {
    ReadOnly,
    WriteOnly,
    ReadWrite,
}

impl GpuBufferAccessMode {
    #[must_use]
    pub fn cl_mem_flags(&self) -> cl_mem_flags {
        match self {
            GpuBufferAccessMode::ReadOnly => CL_MEM_READ_ONLY,
            GpuBufferAccessMode::WriteOnly => CL_MEM_WRITE_ONLY,
            GpuBufferAccessMode::ReadWrite => CL_MEM_READ_WRITE,
        }
    }
}

pub struct GpuHostPtrBuffer<T> {
    buffer: Buffer<T>,
    length: usize,
}

impl<T> GpuHostPtrBuffer<T> {
    #[must_use]
    pub fn buffer(&self) -> &Buffer<T> {
        &self.buffer
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.length
    }
}

pub trait GpuHostPtrBufferUtils<T> {
    unsafe fn init(
        &mut self,
        data: &mut [T],
        access_mode: GpuBufferAccessMode,
        name: &'static str,
    ) -> &mut GpuHostPtrBuffer<T>;

    fn len(&self) -> usize;
    unsafe fn set_arg(&self, kernel: &mut ExecuteKernel);
}

impl<T> GpuHostPtrBufferUtils<T> for Option<GpuHostPtrBuffer<T>> {
    unsafe fn init(
        &mut self,
        data: &mut [T],
        access_mode: GpuBufferAccessMode,
        name: &'static str,
    ) -> &mut GpuHostPtrBuffer<T> {
        self.take();
        self.insert(unsafe {
            GPU.create_host_ptr_buffer(data, access_mode)
                .with_context(|| format!("failed to create GPU host ptr buffer {name}"))
                .unwrap()
        })
    }

    fn len(&self) -> usize {
        self.as_ref().unwrap().length
    }

    unsafe fn set_arg(&self, kernel: &mut ExecuteKernel) {
        unsafe { kernel.set_arg(&self.as_ref().unwrap().buffer) };
    }
}

pub struct GpuHostBuffer<T> {
    data: Vec<T>,
    buffer: Buffer<T>,
}

impl<T> GpuHostBuffer<T> {
    #[must_use]
    pub fn data(&self) -> &Vec<T> {
        &self.data
    }

    pub fn data_mut(&mut self) -> &mut Vec<T> {
        &mut self.data
    }

    #[must_use]
    pub fn buffer(&self) -> &Buffer<T> {
        &self.buffer
    }
}

pub trait GpuHostBufferUtils<T> {
    fn init(&mut self, data: Vec<T>, access_mode: GpuBufferAccessMode, name: &'static str) -> &mut GpuHostBuffer<T>;

    fn len(&self) -> usize;
    unsafe fn set_arg(&self, kernel: &mut ExecuteKernel);
}

impl<T> GpuHostBufferUtils<T> for Option<GpuHostBuffer<T>> {
    fn init(&mut self, data: Vec<T>, access_mode: GpuBufferAccessMode, name: &'static str) -> &mut GpuHostBuffer<T> {
        self.take();
        self.insert(
            GPU.create_host_buffer(data, access_mode)
                .with_context(|| format!("failed to create GPU host buffer {name}"))
                .unwrap(),
        )
    }

    fn len(&self) -> usize {
        self.as_ref().unwrap().data.len()
    }

    unsafe fn set_arg(&self, kernel: &mut ExecuteKernel) {
        unsafe { kernel.set_arg(&self.as_ref().unwrap().buffer) };
    }
}

pub struct GpuDeviceBuffer<T> {
    buffer: Buffer<T>,
    length: usize,
}

impl<T> GpuDeviceBuffer<T> {
    #[must_use]
    pub fn buffer(&self) -> &Buffer<T> {
        &self.buffer
    }

    #[must_use]
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.length
    }
}

pub trait GpuDeviceBufferUtils<T> {
    fn init(&mut self, length: usize, access_mode: GpuBufferAccessMode, name: &'static str) -> &mut GpuDeviceBuffer<T>;
    fn len(&self) -> usize;
    unsafe fn set_arg(&self, kernel: &mut ExecuteKernel);
    fn enqueue_write(&mut self, data: &[T], name: &'static str) -> anyhow::Result<Event>;
    fn enqueue_read(&self, data: &mut [T], name: &'static str) -> anyhow::Result<Event>;
}

impl<T> GpuDeviceBufferUtils<T> for Option<GpuDeviceBuffer<T>> {
    fn init(&mut self, length: usize, access_mode: GpuBufferAccessMode, name: &'static str) -> &mut GpuDeviceBuffer<T> {
        self.take_if(|b| length > b.len());
        self.get_or_insert_with(|| {
            GPU.create_device_buffer(length, access_mode)
                .with_context(|| format!("failed to create GPU device buffer {name}"))
                .unwrap()
        })
    }

    fn len(&self) -> usize {
        self.as_ref().unwrap().length
    }

    unsafe fn set_arg(&self, kernel: &mut ExecuteKernel) {
        unsafe { kernel.set_arg(self.as_ref().unwrap().buffer()) };
    }

    fn enqueue_write(&mut self, data: &[T], name: &'static str) -> anyhow::Result<Event> {
        GPU.enqueue_write_device_buffer(self.as_mut().unwrap(), data)
            .with_context(|| format!("failed to enqueue write GPU device buffer {name}"))
    }

    fn enqueue_read(&self, data: &mut [T], name: &'static str) -> anyhow::Result<Event> {
        GPU.enqueue_read_device_buffer(self.as_ref().unwrap(), data)
            .with_context(|| format!("failed to enqueue read GPU device buffer {name}"))
    }
}
