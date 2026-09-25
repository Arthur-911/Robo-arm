use serde::{Deserialize, Serialize};

/// Type of trajectory interpolation polynomial.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PolynomialType {
    /// 3rd order polynomial (specifies position and velocity boundary conditions).
    Cubic,
    /// 5th order polynomial (specifies position, velocity, and acceleration; minimum jerk).
    Quintic,
}

/// 3rd-order (Cubic) polynomial segment: s(t) = a0 + a1*t + a2*t^2 + a3*t^3.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CubicPolynomial {
    pub a0: f64,
    pub a1: f64,
    pub a2: f64,
    pub a3: f64,
    pub duration: f64,
}

impl CubicPolynomial {
    /// Creates a cubic polynomial satisfying s(0)=s0, s(T)=s1, v(0)=v0, v(T)=v1.
    pub fn new(s0: f64, s1: f64, v0: f64, v1: f64, duration: f64) -> Self {
        let t = duration.max(1e-4);
        let t2 = t * t;
        let t3 = t2 * t;

        let a0 = s0;
        let a1 = v0;
        let a2 = (3.0 * (s1 - s0) - (2.0 * v0 + v1) * t) / t2;
        let a3 = (-2.0 * (s1 - s0) + (v0 + v1) * t) / t3;

        Self {
            a0,
            a1,
            a2,
            a3,
            duration: t,
        }
    }

    pub fn position(&self, t: f64) -> f64 {
        let tau = t.clamp(0.0, self.duration);
        self.a0 + self.a1 * tau + self.a2 * tau * tau + self.a3 * tau * tau * tau
    }

    pub fn velocity(&self, t: f64) -> f64 {
        let tau = t.clamp(0.0, self.duration);
        self.a1 + 2.0 * self.a2 * tau + 3.0 * self.a3 * tau * tau
    }

    pub fn acceleration(&self, t: f64) -> f64 {
        let tau = t.clamp(0.0, self.duration);
        2.0 * self.a2 + 6.0 * self.a3 * tau
    }
}

/// 5th-order (Quintic) polynomial segment: s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5.
/// Guarantees zero or continuous acceleration boundaries, yielding minimum-jerk motion.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct QuinticPolynomial {
    pub a0: f64,
    pub a1: f64,
    pub a2: f64,
    pub a3: f64,
    pub a4: f64,
    pub a5: f64,
    pub duration: f64,
}

impl QuinticPolynomial {
    /// Creates a quintic polynomial satisfying s(0)=s0, s(T)=s1, v(0)=v0, v(T)=v1, a(0)=a0, a(T)=a1.
    pub fn new(
        s0: f64,
        s1: f64,
        v0: f64,
        v1: f64,
        a0_val: f64,
        a1_val: f64,
        duration: f64,
    ) -> Self {
        let t = duration.max(1e-4);
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;

        let a0 = s0;
        let a1 = v0;
        let a2 = 0.5 * a0_val;
        let a3 = (20.0 * (s1 - s0) - (8.0 * v1 + 12.0 * v0) * t - (3.0 * a0_val - a1_val) * t2)
            / (2.0 * t3);
        let a4 =
            (30.0 * (s0 - s1) + (14.0 * v1 + 16.0 * v0) * t + (3.0 * a0_val - 2.0 * a1_val) * t2)
                / (2.0 * t4);
        let a5 = (12.0 * (s1 - s0) - 6.0 * (v1 + v0) * t - (a0_val - a1_val) * t2) / (2.0 * t5);

        Self {
            a0,
            a1,
            a2,
            a3,
            a4,
            a5,
            duration: t,
        }
    }

    pub fn position(&self, t: f64) -> f64 {
        let tau = t.clamp(0.0, self.duration);
        let tau2 = tau * tau;
        let tau3 = tau2 * tau;
        let tau4 = tau3 * tau;
        let tau5 = tau4 * tau;
        self.a0 + self.a1 * tau + self.a2 * tau2 + self.a3 * tau3 + self.a4 * tau4 + self.a5 * tau5
    }

    pub fn velocity(&self, t: f64) -> f64 {
        let tau = t.clamp(0.0, self.duration);
        let tau2 = tau * tau;
        let tau3 = tau2 * tau;
        let tau4 = tau3 * tau;
        self.a1
            + 2.0 * self.a2 * tau
            + 3.0 * self.a3 * tau2
            + 4.0 * self.a4 * tau3
            + 5.0 * self.a5 * tau4
    }

    pub fn acceleration(&self, t: f64) -> f64 {
        let tau = t.clamp(0.0, self.duration);
        let tau2 = tau * tau;
        let tau3 = tau2 * tau;
        2.0 * self.a2 + 6.0 * self.a3 * tau + 12.0 * self.a4 * tau2 + 20.0 * self.a5 * tau3
    }
}
