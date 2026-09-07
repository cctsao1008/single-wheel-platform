#![no_std]

/// Canonical real-time floating-point dot product.
///
/// On the STM32F103 (`thumbv7m-none-eabi`) this calls the CMSIS-DSP Cortex-M3
/// implementation. There is no alternate scalar production path on ARM.
///
/// Non-ARM builds provide only a host semantic emulator so the model, estimator,
/// and controller can retain deterministic unit tests on CI hosts that cannot
/// link an ARM CMSIS-DSP archive.
#[inline(always)]
pub fn dot_f32(lhs: &[f32], rhs: &[f32]) -> f32 {
    assert_eq!(lhs.len(), rhs.len());
    backend::dot_f32(lhs, rhs)
}

#[cfg(target_arch = "arm")]
mod backend {
    #[inline(always)]
    pub fn dot_f32(lhs: &[f32], rhs: &[f32]) -> f32 {
        cmsis_dsp::basic::dot_product_f32(lhs, rhs)
    }
}

#[cfg(not(target_arch = "arm"))]
mod backend {
    /// Host-only semantic emulator for unit tests and offline verification.
    /// Production Cortex-M builds never compile this implementation.
    #[inline(always)]
    pub fn dot_f32(lhs: &[f32], rhs: &[f32]) -> f32 {
        lhs.iter()
            .zip(rhs.iter())
            .map(|(left, right)| left * right)
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dot_product_contract_matches_linear_algebra_definition() {
        let lhs = [1.0, -2.0, 3.0, 0.5];
        let rhs = [4.0, 5.0, -1.0, 2.0];
        assert!((dot_f32(&lhs, &rhs) + 8.0).abs() < 1.0e-6);
    }
}
