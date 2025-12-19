/*
 * Copyright 2025 Advanced Micro Devices, Inc.
 */

/**
 * @file xf_solver_lm_so101.cpp
 *
 * @brief SO-101 dedicated LM solver HLS wrapper - pure AXI-Lite interface
 * 
 * PS side responsibilities:
 * - Provide sin(q), cos(q) (5 joints)
 * - Provide error vector e (up to 6 dimensions)
 * - Provide damping factor λ
 * - Control LM iteration loop
 * - Update q and check convergence
 * 
 * PL side responsibilities (this module):
 * - Compute FK and Jacobian from sin/cos
 * - Compute A = J^T * J
 * - Compute b = J^T * e
 * - Cholesky求解 (A + λI) * Δq = b
 */

#include "lm_solver_so101.hpp"

extern "C" {

/**
 * @brief SO-101 LM solver core - pure AXI-Lite interface
 * 
 * All parameters passed through independent AXI-Lite registers
 * No ap_ctrl handshake signals, suitable for register-mapped access
 * 
 * Input registers:
 *   sin0-sin4@0x10-0x30, cos0-cos4@0x38-0x58
 *   e0-e5@0x60-0x88, lambda@0x90
 * 
 * Output registers:
 *   d0-d4@0x98-0xD8
 *   status@0xE8
 *   state@0xF0 (new: running state)
 */
void xf_solver_lm_so101(
    // Input: sin(q)
    float sin0, float sin1, float sin2, float sin3, float sin4,
    
    // Input: cos(q)
    float cos0, float cos1, float cos2, float cos3, float cos4,
    
    // Input: error vector 6 dimensions
    float e0, float e1, float e2, float e3, float e4, float e5,
    
    // LM damping factor
    float lambda,
    
    // Output: delta_q (pointer method, must use volatile)
    volatile float *d0, volatile float *d1, volatile float *d2, 
    volatile float *d3, volatile float *d4,
    
    // Output: status code (0=success, 1=failure)
    volatile int *status,
    
    // Output: running state (new, for iteration control)
    // bit[0]=1: core ready
    // bit[1]=1: computation complete
    // bit[2]=1: computing
    volatile int *state
) {
    // ============================================
    // AXI-Lite interface configuration (all scalars independently mapped)
    // ============================================
    #pragma HLS INTERFACE s_axilite port=sin0     bundle=control
    #pragma HLS INTERFACE s_axilite port=sin1     bundle=control
    #pragma HLS INTERFACE s_axilite port=sin2     bundle=control
    #pragma HLS INTERFACE s_axilite port=sin3     bundle=control
    #pragma HLS INTERFACE s_axilite port=sin4     bundle=control
    
    #pragma HLS INTERFACE s_axilite port=cos0     bundle=control
    #pragma HLS INTERFACE s_axilite port=cos1     bundle=control
    #pragma HLS INTERFACE s_axilite port=cos2     bundle=control
    #pragma HLS INTERFACE s_axilite port=cos3     bundle=control
    #pragma HLS INTERFACE s_axilite port=cos4     bundle=control
    
    #pragma HLS INTERFACE s_axilite port=e0       bundle=control
    #pragma HLS INTERFACE s_axilite port=e1       bundle=control
    #pragma HLS INTERFACE s_axilite port=e2       bundle=control
    #pragma HLS INTERFACE s_axilite port=e3       bundle=control
    #pragma HLS INTERFACE s_axilite port=e4       bundle=control
    #pragma HLS INTERFACE s_axilite port=e5       bundle=control
    
    #pragma HLS INTERFACE s_axilite port=lambda   bundle=control
    
    #pragma HLS INTERFACE s_axilite port=d0       bundle=control
    #pragma HLS INTERFACE s_axilite port=d1       bundle=control
    #pragma HLS INTERFACE s_axilite port=d2       bundle=control
    #pragma HLS INTERFACE s_axilite port=d3       bundle=control
    #pragma HLS INTERFACE s_axilite port=d4       bundle=control
    
    #pragma HLS INTERFACE s_axilite port=status   bundle=control
    #pragma HLS INTERFACE s_axilite port=state    bundle=control
    
    #pragma HLS INTERFACE s_axilite port=return   bundle=control
    
    // ============================================
    // State management
    // ============================================
    // bit[0] = Ready (always 1)
    // bit[1] = Done (computation complete)
    // bit[2] = Busy (computing)
    
    // Set Ready flag
    *state = 0x01;
    
    // Set Busy flag
    *state |= 0x04;
    
    // ============================================
    // Assemble into arrays for core LM solver
    // ============================================
    float sin_q[5] = {sin0, sin1, sin2, sin3, sin4};
    float cos_q[5] = {cos0, cos1, cos2, cos3, cos4};
    float error[6] = {e0, e1, e2, e3, e4, e5};
    float delta[5];
    
    // Call core LM solving step
    int solve_status = xf::solver::lm_solve_step_so101<float>(
        sin_q, 
        cos_q, 
        error, 
        lambda, 
        delta
    );
    
    // Write output back to AXI-Lite registers
    *d0 = delta[0];
    *d1 = delta[1];
    *d2 = delta[2];
    *d3 = delta[3];
    *d4 = delta[4];
    *status = solve_status;
    
    // Clear Busy flag, set Done flag
    *state = 0x03;  // Ready (bit0=1) + Done (bit1=1)
}

}
