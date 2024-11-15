use anyhow::{Context, Result};
use core::option::Option;
use core::option::Option::{None, Some};
use std::collections::VecDeque;
use std::convert::TryFrom;
use std::time::Instant;

pub struct FpsCalculator {
    last_measure_time: Instant,
    frame_count_queue: VecDeque<(Instant, u128)>,
}

impl FpsCalculator {
    pub fn new() -> Self {
        Self {
            last_measure_time: Instant::now(),
            frame_count_queue: VecDeque::new(),
        }
    }

    pub fn update(&mut self, frame_count: usize) -> Result<Option<u128>> {
        let frame_count = u128::try_from(frame_count).context("FpsCalculator::update")?;

        const FRAMES_MEASURE_PERIOD_MILLIS: u128 = 100;
        const {
            assert!(FRAMES_MEASURE_PERIOD_MILLIS > 0);
        }

        let now = Instant::now();
        if (now - self.last_measure_time).as_millis() >= FRAMES_MEASURE_PERIOD_MILLIS {
            self.frame_count_queue.push_back((now, frame_count));
            self.last_measure_time = now;
        }

        if let Some((measure_start, start_frame_count)) = self.frame_count_queue.front().cloned() {
            const STATS_AVERAGING_PERIOD_MILLIS: u128 = 300;
            const {
                assert!(STATS_AVERAGING_PERIOD_MILLIS > 0);
            }

            let period_millis = (now - measure_start).as_millis();
            if period_millis > STATS_AVERAGING_PERIOD_MILLIS {
                while let Some((measure_start, _)) = self.frame_count_queue.front().cloned() {
                    if (now - measure_start).as_millis() > STATS_AVERAGING_PERIOD_MILLIS {
                        self.frame_count_queue.pop_front();
                    } else {
                        break;
                    }
                }

                let frames_in_period = frame_count - start_frame_count;
                let frames_per_second = frames_in_period * 1000 / period_millis;
                return Ok(Some(frames_per_second));
            }
        }

        Ok(None)
    }
}
