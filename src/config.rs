use core::result::Result;
use core::result::Result::Ok;
use num_traits::Zero;
use serde_derive::Deserialize;
use std::error::Error;
use std::fmt::Display;
use std::fs::File;
use std::io::Read;
use std::ops::RangeInclusive;
use std::path::Path;

#[derive(Deserialize)]
pub struct Config {
    pub screen_width: u32,
    pub screen_height: u32,
}

impl Config {
    pub fn from_file(config_path: &Path) -> Result<Config, String> {
        fn read(config_path: &Path) -> Result<Config, Box<dyn Error>> {
            let mut config_file = File::open(config_path)?;
            let mut config_string = String::new();
            config_file.read_to_string(&mut config_string)?;
            let config: Config = toml::from_str(&config_string)?;
            config.verify()?;
            Ok(config)
        }

        read(config_path).map_err(|error| format!("{}: {}", config_path.to_string_lossy(), error))
    }

    fn verify(&self) -> Result<(), String> {
        check(&self.screen_width, "screen_width", Positive)?;
        Ok(())
    }
}

fn check<T, R: Rule<T>>(value: &T, name: &'static str, rule: R) -> Result<(), String> {
    if rule.condition(value) {
        Ok(())
    } else {
        Err(rule.error_message(value, name))
    }
}

struct GreatOrEqual<T>(T);
struct Positive;
struct IsInRange<T>(RangeInclusive<T>);

trait Rule<T> {
    fn condition(&self, value: &T) -> bool;
    fn error_message(&self, value: &T, name: &'static str) -> String;
}

impl<T: PartialOrd + Display> Rule<T> for GreatOrEqual<T> {
    fn condition(&self, value: &T) -> bool {
        value >= &self.0
    }

    fn error_message(&self, value: &T, name: &'static str) -> String {
        format!("{} ({}) must be greater than {}", name, value, &self.0)
    }
}

impl<T: Zero + PartialOrd + Display> Rule<T> for Positive {
    fn condition(&self, value: &T) -> bool {
        value > &Zero::zero()
    }

    fn error_message(&self, value: &T, name: &'static str) -> String {
        format!("{} ({}) must be positive", name, value)
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
