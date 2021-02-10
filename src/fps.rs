use core::option::Option;
use core::option::Option::{None, Some};
use static_assertions::const_assert;
use std::collections::VecDeque;
use std::convert::TryFrom;
use std::time::Instant;

pub struct FpsCalculator {
    last_measure_time: Instant,
    frame_count_queue: VecDeque<(Instant, usize)>,
}

impl FpsCalculator {
    pub fn new() -> Self {
        Self {
            last_measure_time: Instant::now(),
            frame_count_queue: VecDeque::new(),
        }
    }

    pub fn update(&mut self, frame_count: usize) -> Option<u128> {
        const FRAMES_MEASURE_PERIOD_MILLIS: u128 = 100;
        const_assert!(FRAMES_MEASURE_PERIOD_MILLIS > 0);

        let real_time = Instant::now();
        if (real_time - self.last_measure_time).as_millis() >= FRAMES_MEASURE_PERIOD_MILLIS {
            self.frame_count_queue.push_back((real_time, frame_count));
            self.last_measure_time = real_time;
        }

        if let Some((measure_start, start_frame_count)) = self.frame_count_queue.front().cloned() {
            const STATS_AVERAGING_PERIOD_MILLIS: u128 = 200;
            const_assert!(STATS_AVERAGING_PERIOD_MILLIS > 0);

            let period_millis = (real_time - measure_start).as_millis();
            if period_millis > FRAMES_MEASURE_PERIOD_MILLIS {
                while let Some((measure_start, _)) = self.frame_count_queue.front().cloned() {
                    if (real_time - measure_start).as_millis() > STATS_AVERAGING_PERIOD_MILLIS {
                        self.frame_count_queue.pop_front();
                    } else {
                        break;
                    }
                }

                let frames_in_period = frame_count - start_frame_count;
                let frames_per_second =
                    u128::try_from(frames_in_period).unwrap() * 1000 / period_millis;
                return Some(frames_per_second);
            }
        }

        None
    }
}
