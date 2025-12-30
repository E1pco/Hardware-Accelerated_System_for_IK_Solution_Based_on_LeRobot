/*
 * Copyright 2025 Advanced Micro Devices, Inc.
 */

#ifndef _LM_SOLVER_SO101_HPP_
#define _LM_SOLVER_SO101_HPP_

#include "hls_stream.h"
#include "hls_math.h"
#include "ap_fixed.h"
#include "xf_solver_L1.hpp"

namespace xf {
namespace solver {

// Fixed-point type definitions
// Q1.30: 32-bit signed fixed-point (1 sign bit + 30 fractional bits), range [-1, 1-2^-30]
typedef ap_fixed<32, 2> fixed_q229_t;  // 2 sign bits + 29 fractional bits, range [-2, 2-2^-29]
typedef ap_fixed<32, 1> fixed_q130_t;  // 1 sign bit + 30 fractional bits, range [-1, 1-2^-30]

// Conversion and utility functions
namespace FixedPoint {
    const float Q130_SCALE = 1073741824.0f;  // 2^30
    const float Q229_SCALE = 536870912.0f;   // 2^29
    
    // Float ↔ Q1.30 fixed-point
    inline fixed_q130_t float_to_q130(float val) {
        return val;  // ap_fixed automatic conversion
    }
    
    inline float q130_to_float(fixed_q130_t val) {
        return (float)val;
    }
    
    // Float ↔ Q2.29 fixed-point
    inline fixed_q229_t float_to_q229(float val) {
        return val;  // ap_fixed 自动转换
    }
    
    inline float q229_to_float(fixed_q229_t val) {
        return (float)val;
    }
    
    // Q1.30 * Q1.30 → Q2.29
    inline fixed_q229_t mul_q130_q130(fixed_q130_t a, fixed_q130_t b) {
        return a * b;  // ap_fixed automatically handles precision promotion
    }
    
    // Q2.29 * Q1.30 → Q2.29
    inline fixed_q229_t mul_q229_q130(fixed_q229_t a, fixed_q130_t b) {
        return a * b;
    }
};



/**
 * @brief SO-101 5-DOF robotic arm geometric parameters
 * 
 * Based on ET sequence:
 * E1: tx(0.0612), tz(0.0598), Rz(q0)
 * E2: tx(0.02943), tz(0.05504), Ry(q1)
 * E3: tz(0.1127), tx(0.02798), Ry(q2)
 * E4: tx(0.15504), tz(0.00519), Ry(q3)
 * E5: tx(0.0593), tz(0.00996), Rx(q4)
 */
struct SO101_Params {
    // Joint 1 (Rz)
    static constexpr float j1_tx = 0.0612f;
    static constexpr float j1_tz = 0.0598f;
    
    // Joint 2 (Ry)
    static constexpr float j2_tx = 0.02943f;
    static constexpr float j2_tz = 0.05504f;
    
    // Joint 3 (Ry) - Note: In Python, it's Tz → Tx order
    static constexpr float j3_tz = 0.1127f;
    static constexpr float j3_tx = 0.02798f;
    
    // Joint 4 (Ry)
    static constexpr float j4_tx = 0.15504f;
    static constexpr float j4_tz = 0.00519f;
    
    // Joint 5 (Rx)
    static constexpr float j5_tx = 0.0593f;
    static constexpr float j5_tz = 0.00996f;
};

/**
 * @brief Compute SO-101 forward kinematics (position and orientation only)
 * 
 * @tparam T Data type (float/double)
 * @param sin_q sin value array [5]
 * @param cos_q cos value array [5]
 * @param p_end Output: end-effector position [3] (x, y, z)
 * @param R_end Output: end-effector rotation matrix [3][3]
 */
template <typename T>
void compute_fk_so101_only(
    const T sin_q[5],
    const T cos_q[5],
    T p_end[3],
    T R_end[3][3]
) {
    #pragma HLS INLINE
    
    const T s1 = sin_q[0], c1 = cos_q[0];
    const T s2 = sin_q[1], c2 = cos_q[1];
    const T s3 = sin_q[2], c3 = cos_q[2];
    const T s4 = sin_q[3], c4 = cos_q[3];
    const T s5 = sin_q[4], c5 = cos_q[4];
    
    // Mechanism parameters
    const T tx1 = SO101_Params::j1_tx;
    const T tz1 = SO101_Params::j1_tz;
    const T tx2 = SO101_Params::j2_tx;
    const T tz2 = SO101_Params::j2_tz;
    const T tx3 = SO101_Params::j3_tx;
    const T tz3 = SO101_Params::j3_tz;
    const T tx4 = SO101_Params::j4_tx;
    const T tz4 = SO101_Params::j4_tz;
    const T tx5 = SO101_Params::j5_tx;
    const T tz5 = SO101_Params::j5_tz;
    
    // Initial position (base)
    T px = tx1, py = 0.0f, pz = tz1;
    
    T R[3][3];
    R[0][0] = 1.0f; R[0][1] = 0.0f; R[0][2] = 0.0f;
    R[1][0] = 0.0f; R[1][1] = 1.0f; R[1][2] = 0.0f;
    R[2][0] = 0.0f; R[2][1] = 0.0f; R[2][2] = 1.0f;
    
    // ============================================
    // Joint 1: Rz(q1)
    // ============================================
    T R_temp[3][3];
    R_temp[0][0] = R[0][0]*c1 + R[0][1]*s1;
    R_temp[0][1] = -R[0][0]*s1 + R[0][1]*c1;
    R_temp[0][2] = R[0][2];
    
    R_temp[1][0] = R[1][0]*c1 + R[1][1]*s1;
    R_temp[1][1] = -R[1][0]*s1 + R[1][1]*c1;
    R_temp[1][2] = R[1][2];
    
    R_temp[2][0] = R[2][0]*c1 + R[2][1]*s1;
    R_temp[2][1] = -R[2][0]*s1 + R[2][1]*c1;
    R_temp[2][2] = R[2][2];
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R[i][j] = R_temp[i][j];
        }
    }
    
    px += R[0][0]*tx2 + R[0][2]*tz2;
    py += R[1][0]*tx2 + R[1][2]*tz2;
    pz += R[2][0]*tx2 + R[2][2]*tz2;
    
    // ============================================
    // Joint 2: Ry(q2)
    // ============================================
    R_temp[0][0] = R[0][0]*c2 - R[0][2]*s2;
    R_temp[0][1] = R[0][1];
    R_temp[0][2] = R[0][0]*s2 + R[0][2]*c2;
    
    R_temp[1][0] = R[1][0]*c2 - R[1][2]*s2;
    R_temp[1][1] = R[1][1];
    R_temp[1][2] = R[1][0]*s2 + R[1][2]*c2;
    
    R_temp[2][0] = R[2][0]*c2 - R[2][2]*s2;
    R_temp[2][1] = R[2][1];
    R_temp[2][2] = R[2][0]*s2 + R[2][2]*c2;
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R[i][j] = R_temp[i][j];
        }
    }
    
    px += R[0][2]*tz3;
    py += R[1][2]*tz3;
    pz += R[2][2]*tz3;
    
    px += R[0][0]*tx3;
    py += R[1][0]*tx3;
    pz += R[2][0]*tx3;
    
    // ============================================
    // Joint 3: Ry(q3)
    // ============================================
    R_temp[0][0] = R[0][0]*c3 - R[0][2]*s3;
    R_temp[0][1] = R[0][1];
    R_temp[0][2] = R[0][0]*s3 + R[0][2]*c3;
    
    R_temp[1][0] = R[1][0]*c3 - R[1][2]*s3;
    R_temp[1][1] = R[1][1];
    R_temp[1][2] = R[1][0]*s3 + R[1][2]*c3;
    
    R_temp[2][0] = R[2][0]*c3 - R[2][2]*s3;
    R_temp[2][1] = R[2][1];
    R_temp[2][2] = R[2][0]*s3 + R[2][2]*c3;
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R[i][j] = R_temp[i][j];
        }
    }
    
    px += R[0][0]*tx4 + R[0][2]*tz4;
    py += R[1][0]*tx4 + R[1][2]*tz4;
    pz += R[2][0]*tx4 + R[2][2]*tz4;
    
    // ============================================
    // Joint 4: Ry(q4)
    // ============================================
    R_temp[0][0] = R[0][0]*c4 - R[0][2]*s4;
    R_temp[0][1] = R[0][1];
    R_temp[0][2] = R[0][0]*s4 + R[0][2]*c4;
    
    R_temp[1][0] = R[1][0]*c4 - R[1][2]*s4;
    R_temp[1][1] = R[1][1];
    R_temp[1][2] = R[1][0]*s4 + R[1][2]*c4;
    
    R_temp[2][0] = R[2][0]*c4 - R[2][2]*s4;
    R_temp[2][1] = R[2][1];
    R_temp[2][2] = R[2][0]*s4 + R[2][2]*c4;
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R[i][j] = R_temp[i][j];
        }
    }
    
    px += R[0][0]*tx5 + R[0][2]*tz5;
    py += R[1][0]*tx5 + R[1][2]*tz5;
    pz += R[2][0]*tx5 + R[2][2]*tz5;
    
    // ============================================
    // Joint 5: Rx(q5) - 仅影响姿态，不影响位置
    // ============================================
    R_temp[0][0] = R[0][0];
    R_temp[0][1] = R[0][1]*c5 + R[0][2]*s5;
    R_temp[0][2] = -R[0][1]*s5 + R[0][2]*c5;
    
    R_temp[1][0] = R[1][0];
    R_temp[1][1] = R[1][1]*c5 + R[1][2]*s5;
    R_temp[1][2] = -R[1][1]*s5 + R[1][2]*c5;
    
    R_temp[2][0] = R[2][0];
    R_temp[2][1] = R[2][1]*c5 + R[2][2]*s5;
    R_temp[2][2] = -R[2][1]*s5 + R[2][2]*c5;
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R[i][j] = R_temp[i][j];
        }
    }
    
    p_end[0] = px;
    p_end[1] = py;
    p_end[2] = pz;
    
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R_end[i][j] = R[i][j];
        }
    }
}

/**
 * @brief Compute SO-101 forward kinematics and geometric Jacobian (Analytical)
 * 
 * @tparam T Data type (float/double)
 * @param sin_q sin value array [5]
 * @param cos_q cos value array [5]
 * @param p_end Output: end-effector position [3] (x, y, z)
 * @param J Output: Jacobian matrix [6][5] (row-major)
 */
template <typename T>
void compute_fk_jacobian_so101(
    const T sin_q[5],
    const T cos_q[5],
    T p_end[3],
    T J[6][5]
) {
    #pragma HLS INLINE off
    
    const T s1 = sin_q[0], c1 = cos_q[0];
    const T s2 = sin_q[1], c2 = cos_q[1];
    const T s3 = sin_q[2], c3 = cos_q[2];
    const T s4 = sin_q[3], c4 = cos_q[3];
    const T s5 = sin_q[4], c5 = cos_q[4];
    
    // 机构参数
    const T tx1 = SO101_Params::j1_tx;
    const T tz1 = SO101_Params::j1_tz;
    const T tx2 = SO101_Params::j2_tx;
    const T tz2 = SO101_Params::j2_tz;
    const T tx3 = SO101_Params::j3_tx;
    const T tz3 = SO101_Params::j3_tz;
    const T tx4 = SO101_Params::j4_tx;
    const T tz4 = SO101_Params::j4_tz;
    const T tx5 = SO101_Params::j5_tx;
    const T tz5 = SO101_Params::j5_tz;
    
    // Store joint axes (z) and positions (p)
    // z_axes[i] is the rotation axis of joint i+1
    // p_pos[i] is the origin position of joint i+1
    T z_axes[5][3];
    T p_pos[5][3];

    // Initial state (Base)
    // Joint 1 (Rz) located at (tx1, 0, tz1), rotation axis is Z
    z_axes[0][0] = 0.0f; z_axes[0][1] = 0.0f; z_axes[0][2] = 1.0f;
    p_pos[0][0] = tx1;   p_pos[0][1] = 0.0f;   p_pos[0][2] = tz1;

    // 当前 R 和 p
    T R[3][3];
    R[0][0] = 1.0f; R[0][1] = 0.0f; R[0][2] = 0.0f;
    R[1][0] = 0.0f; R[1][1] = 1.0f; R[1][2] = 0.0f;
    R[2][0] = 0.0f; R[2][1] = 0.0f; R[2][2] = 1.0f;
    
    T p[3];
    p[0] = tx1; p[1] = 0.0f; p[2] = tz1;

    // --- Joint 1 (Rz) ---
    // Update R: R = R * Rz(q0)
    T R_next[3][3];
    R_next[0][0] = R[0][0]*c1 + R[0][1]*s1;
    R_next[0][1] = -R[0][0]*s1 + R[0][1]*c1;
    R_next[0][2] = R[0][2];
    R_next[1][0] = R[1][0]*c1 + R[1][1]*s1;
    R_next[1][1] = -R[1][0]*s1 + R[1][1]*c1;
    R_next[1][2] = R[1][2];
    R_next[2][0] = R[2][0]*c1 + R[2][1]*s1;
    R_next[2][1] = -R[2][0]*s1 + R[2][1]*c1;
    R_next[2][2] = R[2][2];
    
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) R[i][j] = R_next[i][j];

    // Update p: p += R * [tx2, 0, tz2]
    p[0] += R[0][0]*tx2 + R[0][2]*tz2;
    p[1] += R[1][0]*tx2 + R[1][2]*tz2;
    p[2] += R[2][0]*tx2 + R[2][2]*tz2;

    // Save Joint 2 information (Ry)
    // Rotation axis is the Y-axis of the current frame: R * [0, 1, 0]
    z_axes[1][0] = R[0][1];
    z_axes[1][1] = R[1][1];
    z_axes[1][2] = R[2][1];
    p_pos[1][0] = p[0];
    p_pos[1][1] = p[1];
    p_pos[1][2] = p[2];

    // --- Joint 2 (Ry) ---
    // Update R: R = R * Ry(q1)
    R_next[0][0] = R[0][0]*c2 - R[0][2]*s2;
    R_next[0][1] = R[0][1];
    R_next[0][2] = R[0][0]*s2 + R[0][2]*c2;
    R_next[1][0] = R[1][0]*c2 - R[1][2]*s2;
    R_next[1][1] = R[1][1];
    R_next[1][2] = R[1][0]*s2 + R[1][2]*c2;
    R_next[2][0] = R[2][0]*c2 - R[2][2]*s2;
    R_next[2][1] = R[2][1];
    R_next[2][2] = R[2][0]*s2 + R[2][2]*c2;
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) R[i][j] = R_next[i][j];

    // Update p: p += R * [tx3, 0, tz3]
    p[0] += R[0][0]*tx3 + R[0][2]*tz3;
    p[1] += R[1][0]*tx3 + R[1][2]*tz3;
    p[2] += R[2][0]*tx3 + R[2][2]*tz3;

    // Save Joint 3 information (Ry)
    z_axes[2][0] = R[0][1];
    z_axes[2][1] = R[1][1];
    z_axes[2][2] = R[2][1];
    p_pos[2][0] = p[0];
    p_pos[2][1] = p[1];
    p_pos[2][2] = p[2];

    // --- Joint 3 (Ry) ---
    // Update R: R = R * Ry(q2)
    R_next[0][0] = R[0][0]*c3 - R[0][2]*s3;
    R_next[0][1] = R[0][1];
    R_next[0][2] = R[0][0]*s3 + R[0][2]*c3;
    R_next[1][0] = R[1][0]*c3 - R[1][2]*s3;
    R_next[1][1] = R[1][1];
    R_next[1][2] = R[1][0]*s3 + R[1][2]*c3;
    R_next[2][0] = R[2][0]*c3 - R[2][2]*s3;
    R_next[2][1] = R[2][1];
    R_next[2][2] = R[2][0]*s3 + R[2][2]*c3;
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) R[i][j] = R_next[i][j];

    // Update p: p += R * [tx4, 0, tz4]
    p[0] += R[0][0]*tx4 + R[0][2]*tz4;
    p[1] += R[1][0]*tx4 + R[1][2]*tz4;
    p[2] += R[2][0]*tx4 + R[2][2]*tz4;

    // Save Joint 4 information (Ry)
    z_axes[3][0] = R[0][1];
    z_axes[3][1] = R[1][1];
    z_axes[3][2] = R[2][1];
    p_pos[3][0] = p[0];
    p_pos[3][1] = p[1];
    p_pos[3][2] = p[2];

    // --- Joint 4 (Ry) ---
    // Update R: R = R * Ry(q3)
    R_next[0][0] = R[0][0]*c4 - R[0][2]*s4;
    R_next[0][1] = R[0][1];
    R_next[0][2] = R[0][0]*s4 + R[0][2]*c4;
    R_next[1][0] = R[1][0]*c4 - R[1][2]*s4;
    R_next[1][1] = R[1][1];
    R_next[1][2] = R[1][0]*s4 + R[1][2]*c4;
    R_next[2][0] = R[2][0]*c4 - R[2][2]*s4;
    R_next[2][1] = R[2][1];
    R_next[2][2] = R[2][0]*s4 + R[2][2]*c4;
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) R[i][j] = R_next[i][j];

    // Update p: p += R * [tx5, 0, tz5]
    p[0] += R[0][0]*tx5 + R[0][2]*tz5;
    p[1] += R[1][0]*tx5 + R[1][2]*tz5;
    p[2] += R[2][0]*tx5 + R[2][2]*tz5;

    // Save Joint 5 information (Rx)
    // Rotation axis is the X-axis of the current frame: R * [1, 0, 0]
    z_axes[4][0] = R[0][0];
    z_axes[4][1] = R[1][0];
    z_axes[4][2] = R[2][0];
    p_pos[4][0] = p[0];
    p_pos[4][1] = p[1];
    p_pos[4][2] = p[2];

    // --- Joint 5 (Rx) ---
    // Update R: R = R * Rx(q4)
    R_next[0][0] = R[0][0];
    R_next[0][1] = R[0][1]*c5 + R[0][2]*s5;
    R_next[0][2] = -R[0][1]*s5 + R[0][2]*c5;
    R_next[1][0] = R[1][0];
    R_next[1][1] = R[1][1]*c5 + R[1][2]*s5;
    R_next[1][2] = -R[1][1]*s5 + R[1][2]*c5;
    R_next[2][0] = R[2][0];
    R_next[2][1] = R[2][1]*c5 + R[2][2]*s5;
    R_next[2][2] = -R[2][1]*s5 + R[2][2]*c5;
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) R[i][j] = R_next[i][j];

    // No translation after Joint 5
    
    // Output end-effector position
    p_end[0] = p[0];
    p_end[1] = p[1];
    p_end[2] = p[2];

    // Compute Jacobian
    // J_v = z x (p_end - p_joint)
    // J_w = z
    // Partial unroll: UNROLL factor=3, balance DSP and latency
    
    compute_jacobian_cols:
    for(int i=0; i<5; i++) {
        #pragma HLS UNROLL factor=3
        T dx = p_end[0] - p_pos[i][0];
        T dy = p_end[1] - p_pos[i][1];
        T dz = p_end[2] - p_pos[i][2];
        
        // Cross product: z_axes[i] x [dx, dy, dz]
        J[0][i] = z_axes[i][1]*dz - z_axes[i][2]*dy;
        J[1][i] = z_axes[i][2]*dx - z_axes[i][0]*dz;
        J[2][i] = z_axes[i][0]*dy - z_axes[i][1]*dx;
        
        J[3][i] = z_axes[i][0];
        J[4][i] = z_axes[i][1];
        J[5][i] = z_axes[i][2];
    }
}

/**
 * @brief SO-101 LM solver single-step solution
 * 
 * Starting from sin/cos and error vector, complete:
 * 1. FK + Jacobian computation
 * 2. A = J^T * J
 * 3. b = J^T * e
 * 4. Cholesky solve (A + λI) * delta = b
 * 
 * @param sin_q sin value array [5]
 * @param cos_q cos value array [5]
 * @param error error vector [M] (up to 6 dimensions)
 * @param lambda LM damping factor
 * @param delta output: joint increments [5]
 * @return int status code (0=success, 1=failure)
 */
template <typename T>
int lm_solve_step_so101(
    const T sin_q[5],
    const T cos_q[5],
    const T error[6],
    T lambda,
    T delta[5]
) {
    #pragma HLS INLINE off
    
    const int N = 5;       // 5 DOF
    const int MAX_M = 6;   // Up to 6 dimensions error
    
    // 1. Compute FK and Jacobian
    T p_end[3];
    T J[MAX_M][N];
    #pragma HLS ARRAY_PARTITION variable=J dim=2 complete
    #pragma HLS ARRAY_PARTITION variable=J dim=1 complete
    
    compute_fk_jacobian_so101<T>(sin_q, cos_q, p_end, J);
    
    // 2. Compute A = J^T * J (N x N) - Use Vitis L1 library matrix multiplication
    T A[N][N];
    #pragma HLS ARRAY_PARTITION variable=A dim=0 complete
    
    // Define traits type: J^T (5x6) * J (6x5) = A (5x5)
    typedef matrixMultiplyTraits<Transpose, NoTranspose, MAX_M, N, MAX_M, N, T, T> JtJ_Traits;
    
    // Call L1 library matrix multiplication: A = J^T * J
    matrixMultiplyTop<Transpose, NoTranspose, MAX_M, N, MAX_M, N, N, N, JtJ_Traits, T, T>(J, J, A);
    
    T e_mat[MAX_M][1];
    #pragma HLS ARRAY_PARTITION variable=e_mat complete
    for(int i = 0; i < MAX_M; i++) {
        #pragma HLS UNROLL
        e_mat[i][0] = error[i];
    }
    
    T b_mat[N][1];
    #pragma HLS ARRAY_PARTITION variable=b_mat complete
    
    typedef matrixMultiplyTraits<Transpose, NoTranspose, MAX_M, N, MAX_M, 1, T, T> Jte_Traits;
    
    matrixMultiplyTop<Transpose, NoTranspose, MAX_M, N, MAX_M, 1, N, 1, Jte_Traits, T, T>(J, e_mat, b_mat);
    
    T b[N];
    #pragma HLS ARRAY_PARTITION variable=b complete
    for(int i = 0; i < N; i++) {
        #pragma HLS UNROLL
        b[i] = b_mat[i][0];
    }
    
    add_damping:
    for(int i = 0; i < N; i++) {
        #pragma HLS UNROLL
        A[i][i] += lambda;
    }
    
    T L[N][N];
    #pragma HLS ARRAY_PARTITION variable=L dim=0 complete
    
    typedef choleskyTraits<true, N, T, T> CholeskyConfig;
    int chol_ret = choleskyTop<true, N, CholeskyConfig, T, T>(A, L);
    
    if (chol_ret != 0) {
        for(int i = 0; i < N; i++) {
            delta[i] = 0.0f;
        }
        return 1;
    }
    
    T y[N];
    #pragma HLS ARRAY_PARTITION variable=y complete
    
    T L_diag_inv[N];
    #pragma HLS ARRAY_PARTITION variable=L_diag_inv complete
    compute_diag_inv:
    for(int i = 0; i < N; i++) {
        #pragma HLS UNROLL
        L_diag_inv[i] = T(1.0) / L[i][i];
    }
    y[0] = b[0] * L_diag_inv[0];
    y[1] = (b[1] - L[1][0]*y[0]) * L_diag_inv[1];
    y[2] = (b[2] - L[2][0]*y[0] - L[2][1]*y[1]) * L_diag_inv[2];
    y[3] = (b[3] - L[3][0]*y[0] - L[3][1]*y[1] - L[3][2]*y[2]) * L_diag_inv[3];
    y[4] = (b[4] - L[4][0]*y[0] - L[4][1]*y[1] - L[4][2]*y[2] - L[4][3]*y[3]) * L_diag_inv[4];
    
    delta[4] = y[4] * L_diag_inv[4];
    delta[3] = (y[3] - L[4][3]*delta[4]) * L_diag_inv[3];
    delta[2] = (y[2] - L[3][2]*delta[3] - L[4][2]*delta[4]) * L_diag_inv[2];
    delta[1] = (y[1] - L[2][1]*delta[2] - L[3][1]*delta[3] - L[4][1]*delta[4]) * L_diag_inv[1];
    delta[0] = (y[0] - L[1][0]*delta[1] - L[2][0]*delta[2] - L[3][0]*delta[3] - L[4][0]*delta[4]) * L_diag_inv[0];
    
    return 0;
}

} // namespace solver
} // namespace xf

#endif // _LM_SOLVER_SO101_HPP_
