use std::{fs::File, io::Read, path::Path};

use anyhow::{anyhow, Context};
use serde_derive::Deserialize;

#[derive(Deserialize)]
pub struct AppConfig {
    pub width: u32,
    pub height: u32,
    pub restitution_coefficient: f32,
    pub gravity: (f32, f32),
}

impl AppConfig {
    pub fn from_file(config_path: &Path) -> anyhow::Result<AppConfig> {
        let mut config_file =
            File::open(config_path).context(format!("open config \"{}\"", config_path.to_string_lossy()))?;
        let mut config_string = String::new();
        config_file.read_to_string(&mut config_string).context("read config")?;
        let config: AppConfig = toml::from_str(&config_string).context("parse config")?;
        config.validate().context("validate config")?;
        Ok(config)
    }

    fn validate(&self) -> anyhow::Result<()> {
        validate_positive(self.width, "screen_width")?;
        validate_positive(self.height, "screen_height")?;
        validate_restitution_coefficient(self.restitution_coefficient, "restitution_coefficient")?;
        Ok(())
    }
}

fn validate_positive(value: u32, name: &'static str) -> anyhow::Result<()> {
    if value > 0 {
        Ok(())
    } else {
        Err(anyhow!("{} must be positive", name))
    }
}

fn validate_restitution_coefficient(value: f32, name: &'static str) -> anyhow::Result<()> {
    if (0.0..=1.0).contains(&value) {
        Ok(())
    } else {
        Err(anyhow!("{} must be in range [0.0, 1.0]", name))
    }
}
