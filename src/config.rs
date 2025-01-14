use std::{fmt::Display, fs::File, io::Read, path::Path};

use anyhow::{anyhow, Context};
use num_traits::Zero;
use serde_derive::Deserialize;

#[derive(Deserialize)]
pub struct Config {
    pub screen_width: u32,
    pub screen_height: u32,
}

impl Config {
    pub fn from_file(config_path: &Path) -> anyhow::Result<Config> {
        let mut config_file =
            File::open(config_path).context(format!("open config \"{}\"", config_path.to_string_lossy()))?;
        let mut config_string = String::new();
        config_file.read_to_string(&mut config_string).context("read config")?;
        let config: Config = toml::from_str(&config_string).context("parse config")?;
        config.validate().context("validate config")?;
        Ok(config)
    }

    fn validate(&self) -> anyhow::Result<()> {
        check(&self.screen_width, "screen_width", Positive)?;
        check(&self.screen_height, "screen_height", Positive)?;
        Ok(())
    }
}

fn check<T, R: Rule<T>>(value: &T, name: &'static str, rule: R) -> anyhow::Result<()> {
    if rule.condition(value) {
        Ok(())
    } else {
        Err(anyhow!("{}", rule.error_message(value, name)))
    }
}
struct Positive;

trait Rule<T> {
    fn condition(&self, value: &T) -> bool;
    fn error_message(&self, value: &T, name: &'static str) -> String;
}

impl<T: Zero + PartialOrd + Display> Rule<T> for Positive {
    fn condition(&self, value: &T) -> bool {
        value > &Zero::zero()
    }

    fn error_message(&self, value: &T, name: &'static str) -> String {
        format!("{} ({}) must be positive", name, value)
    }
}

#[allow(unused)]
mod unused {
    use std::{fmt::Display, ops::RangeInclusive};

    use super::Rule;

    struct GreatOrEqual<T>(T);
    struct IsInRange<T>(RangeInclusive<T>);

    impl<T: PartialOrd + Display> Rule<T> for GreatOrEqual<T> {
        fn condition(&self, value: &T) -> bool {
            value >= &self.0
        }

        fn error_message(&self, value: &T, name: &'static str) -> String {
            format!("{} ({}) must be greater than {}", name, value, &self.0)
        }
    }

    impl<T: PartialOrd + Display> Rule<T> for IsInRange<T> {
        fn condition(&self, value: &T) -> bool {
            self.0.contains(value)
        }

        fn error_message(&self, value: &T, name: &'static str) -> String {
            format!(
                "{} ({}) is out of range [{}; {}]",
                name,
                value,
                self.0.start(),
                self.0.end()
            )
        }
    }
}
