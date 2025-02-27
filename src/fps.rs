use core::option::{
    Option,
    Option::{None, Some},
};
use std::{collections::VecDeque, time::Instant};

pub struct FpsCalculator {
    last_measure_time: Instant,
    frame_count_queue: VecDeque<(Instant, usize)>,
}

impl FpsCalculator {
    pub fn update(&mut self, frame_count: usize) -> Option<usize> {
        const FRAMES_MEASURE_PERIOD_MILLIS: usize = 100;
        const {
            assert!(FRAMES_MEASURE_PERIOD_MILLIS > 0);
        }

        let now = Instant::now();
        if (now - self.last_measure_time).as_millis() >= FRAMES_MEASURE_PERIOD_MILLIS as u128 {
            self.frame_count_queue.push_back((now, frame_count));
            self.last_measure_time = now;
        }

        if let Some((measure_start, start_frame_count)) = self.frame_count_queue.front().copied() {
            const STATS_AVERAGING_PERIOD_MILLIS: usize = 300;
            const {
                assert!(STATS_AVERAGING_PERIOD_MILLIS > 0);
            }

            let period_millis = (now - measure_start).as_millis();
            if period_millis > STATS_AVERAGING_PERIOD_MILLIS as u128 {
                while let Some((measure_start, _)) = self.frame_count_queue.front().copied() {
                    if (now - measure_start).as_millis() > STATS_AVERAGING_PERIOD_MILLIS as u128 {
                        self.frame_count_queue.pop_front();
                    } else {
                        break;
                    }
                }

                let frames_in_period = frame_count - start_frame_count;
                let frames_per_second = frames_in_period * 1000 / period_millis as usize;
                return Some(frames_per_second);
            }
        }

        None
    }
}

impl Default for FpsCalculator {
    fn default() -> Self {
        Self {
            last_measure_time: Instant::now(),
            frame_count_queue: VecDeque::new(),
        }
    }
}
