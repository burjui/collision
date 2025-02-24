pub const C1: f32 = 0.6756036;
pub const C2: f32 = -0.17560363;
pub const C3: f32 = -0.17560363;
pub const C4: f32 = 0.6756036;
pub const D1: f32 = 1.3512073;
pub const D2: f32 = -1.7024145;
pub const D3: f32 = 1.3512073;

#[allow(unused)]
pub struct YoshidaCoefficients {
    pub c1: f32,
    pub c2: f32,
    pub c3: f32,
    pub c4: f32,
    pub d1: f32,
    pub d2: f32,
    pub d3: f32,
}

impl Default for YoshidaCoefficients {
    fn default() -> Self {
        let cbrt2 = 2.0f32.cbrt();
        let w0 = -cbrt2 / (2.0 - cbrt2);
        let w1 = 1.0 / (2.0 - cbrt2);
        let c1 @ c4 = 0.5 * w1;
        let c2 @ c3 = 0.5 * (w0 + w1);
        let d1 @ d3 = w1;
        let d2 = w0;
        Self {
            c1,
            c2,
            c3,
            c4,
            d1,
            d2,
            d3,
        }
    }
}
