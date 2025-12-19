/*
 * Copyright 2025 Advanced Micro Devices, Inc.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

// ========================================
// SO-101 Forward Kinematics & Error Calculation
// ========================================

// 4x4 matrix multiply: C = A * B
void mat4_multiply(const float A[16], const float B[16], float C[16]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i*4 + j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                C[i*4 + j] += A[i*4 + k] * B[k*4 + j];
            }
        }
    }
}

// Set identity matrix
void mat4_identity(float T[16]) {
    memset(T, 0, 16 * sizeof(float));
    T[0] = T[5] = T[10] = T[15] = 1.0f;
}

// Elementary transform: Tx
void mat4_tx(float d, float T[16]) {
    mat4_identity(T);
    T[3] = d;  // T[0,3]
}

// Elementary transform: Tz
void mat4_tz(float d, float T[16]) {
    mat4_identity(T);
    T[11] = d;  // T[2,3]
}

// Elementary transform: Rx
void mat4_rx(float angle, float T[16]) {
    mat4_identity(T);
    float c = cosf(angle);
    float s = sinf(angle);
    T[5] = c;   // T[1,1]
    T[6] = -s;  // T[1,2]
    T[9] = s;   // T[2,1]
    T[10] = c;  // T[2,2]
}

// Elementary transform: Ry
void mat4_ry(float angle, float T[16]) {
    mat4_identity(T);
    float c = cosf(angle);
    float s = sinf(angle);
    T[0] = c;    // T[0,0]
    T[2] = s;    // T[0,2]
    T[8] = -s;   // T[2,0]
    T[10] = c;   // T[2,2]
}

// Elementary transform: Rz
void mat4_rz(float angle, float T[16]) {
    mat4_identity(T);
    float c = cosf(angle);
    float s = sinf(angle);
    T[0] = c;   // T[0,0]
    T[1] = -s;  // T[0,1]
    T[4] = s;   // T[1,0]
    T[5] = c;   // T[1,1]
}

// Helper function: Compute angle-axis error from relative rotation matrix
void angle_axis_from_rotation_delta(const float R_delta[9], float rot_error[3]) {
    float li[3];
    li[0] = R_delta[7] - R_delta[5];  // R[2,1] - R[1,2]
    li[1] = R_delta[2] - R_delta[6];  // R[0,2] - R[2,0]
    li[2] = R_delta[3] - R_delta[1];  // R[1,0] - R[0,1]
    
    float li_norm = sqrtf(li[0]*li[0] + li[1]*li[1] + li[2]*li[2]);
    float trace_R = R_delta[0] + R_delta[4] + R_delta[8];
    
    if (li_norm < 1e-6f) {
        rot_error[0] = 0.0f;
        rot_error[1] = 0.0f;
        rot_error[2] = 0.0f;
    } else {
        float angle = atan2f(li_norm, trace_R - 1.0f);
        float scale = angle / li_norm;
        rot_error[0] = scale * li[0];
        rot_error[1] = scale * li[1];
        rot_error[2] = scale * li[2];
    }
}

// SO-101 Forward Kinematics
// ETS: tx/tz/Rz, tx/tz/Ry, tz/tx/Ry, tx/tz/Ry, tx/tz/Rx
void fk_so101(const float q[5], float T_result[16]) {
    float T_temp[16], T_accum[16], T_next[16];
    
    // Initialize as identity
    mat4_identity(T_accum);
    
    // E1: tx(0.0612), tz(0.0598), Rz(q0)
    mat4_tx(0.0612f, T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    mat4_tz(0.0598f, T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    mat4_rz(q[0], T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    // E2: tx(0.02943), tz(0.05504), Ry(q1)
    mat4_tx(0.02943f, T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    mat4_tz(0.05504f, T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    mat4_ry(q[1], T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    // E3: tz(0.1127), tx(0.02798), Ry(q2)
    mat4_tz(0.1127f, T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    mat4_tx(0.02798f, T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    mat4_ry(q[2], T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    // E4: tx(0.15504), tz(0.00519), Ry(q3)
    mat4_tx(0.15504f, T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    mat4_tz(0.00519f, T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    mat4_ry(q[3], T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    // E5: tx(0.0593), tz(0.00996), Rx(q4)
    mat4_tx(0.0593f, T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    mat4_tz(0.00996f, T_temp);
    mat4_multiply(T_accum, T_temp, T_next);
    memcpy(T_accum, T_next, 16 * sizeof(float));
    
    mat4_rx(q[4], T_temp);
    mat4_multiply(T_accum, T_temp, T_result);
}

// Compute 6D error vector (Position + Angle-Axis Rotation Error)
// Matches verify_tb.py and HLS Jacobian definition
void compute_error(const float T_current[16], const float T_target[16], float error[6]) {
    // Position error: e[:3] = T_target[:3,3] - T_current[:3,3]
    error[0] = T_target[3]  - T_current[3];   // x
    error[1] = T_target[7]  - T_current[7];   // y
    error[2] = T_target[11] - T_current[11];  // z
    
    // Orientation error: Angle-Axis
    // 1. Extract rotation matrices
    float R_current[9], R_target[9];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            R_current[i*3+j] = T_current[i*4+j];
            R_target[i*3+j]  = T_target[i*4+j];
        }
    }
    
    // 2. Compute relative rotation: R_delta = R_target * R_current^T
    float R_delta[9];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            float sum = 0.0f;
            for(int k=0; k<3; k++) {
                // R_current^T[k,j] = R_current[j,k]
                sum += R_target[i*3+k] * R_current[j*3+k];
            }
            R_delta[i*3+j] = sum;
        }
    }
    
    // 3. Convert to angle-axis vector
    float rot_error[3];
    angle_axis_from_rotation_delta(R_delta, rot_error);
    
    error[3] = rot_error[0];
    error[4] = rot_error[1];
    error[5] = rot_error[2];
}

// ========================================
// HLS IP Core Interface
// ========================================

// 函数声明
extern "C" {
void xf_solver_lm_so101(
    // 输入：sin(q)
    float sin0, float sin1, float sin2, float sin3, float sin4,
    // 输入：cos(q)
    float cos0, float cos1, float cos2, float cos3, float cos4,
    // 输入：误差向量 6 维
    float e0, float e1, float e2, float e3, float e4, float e5,
    // LM 阻尼因子
    float lambda,
    // 输出：delta_q
    volatile float *d0, volatile float *d1, volatile float *d2, 
    volatile float *d3, volatile float *d4,
    // 输出：状态码
    volatile int *status,
    // 输出：状态寄存器
    volatile int *state
);
}

void test_so101_3d() {
    printf("\n=== Test 1: SO-101 with 3D Position Error ===\n");
    const int N = 5;
    
    // 测试关节角度 q = [0.1, -0.2, 0.3, -0.1, 0.15] rad
    float q[N] = {0.1f, -0.2f, 0.3f, -0.1f, 0.15f};
    
    // 计算 sin/cos
    float sin_q[N], cos_q[N];
    for (int i = 0; i < N; i++) {
        sin_q[i] = sinf(q[i]);
        cos_q[i] = cosf(q[i]);
    }
    
    // 误差向量
    float error[6] = {0.02f, 0.015f, 0.01f, 0.0f, 0.0f, 0.0f};
    float lambda = 0.01f;
    
    // 输出变量
    float delta[N];
    int status;
    int state_reg;
    
    // 调用求解器
    xf_solver_lm_so101(
        sin_q[0], sin_q[1], sin_q[2], sin_q[3], sin_q[4],
        cos_q[0], cos_q[1], cos_q[2], cos_q[3], cos_q[4],
        error[0], error[1], error[2], error[3], error[4], error[5],
        lambda,
        &delta[0], &delta[1], &delta[2], &delta[3], &delta[4],
        &status,
        &state_reg
    );
    
    // 输出结果
    printf("Joint angles q: [");
    for (int i = 0; i < N; i++)
        printf("%7.4f ", q[i]);
    printf("]\n");
    
    printf("Delta: [");
    for (int i = 0; i < N; i++)
        printf("%8.6f ", delta[i]);
    printf("]\nStatus: %d\nState: 0x%02x (Ready:%d Done:%d Busy:%d)\n", 
           status, state_reg, 
           (state_reg & 0x01) != 0,
           (state_reg & 0x02) != 0,
           (state_reg & 0x04) != 0);
}

void test_so101_6d() {
    printf("\n=== Test 2: SO-101 with 6D Pose Error ===\n");
    const int N = 5;
    
    // 测试关节角度 q = [0.0, -0.3, 0.4, -0.2, 0.0] rad
    float q[N] = {0.0f, -0.3f, 0.4f, -0.2f, 0.0f};
    
    // 计算 sin/cos
    float sin_q[N], cos_q[N];
    for (int i = 0; i < N; i++) {
        sin_q[i] = sinf(q[i]);
        cos_q[i] = cosf(q[i]);
    }
    
    // 6D位姿误差
    float error[6] = {0.01f, 0.015f, 0.008f, 0.005f, 0.007f, 0.006f};
    float lambda = 0.01f;
    
    // 输出变量
    float delta[N];
    int status;
    int state_reg;
    
    // 调用求解器
    xf_solver_lm_so101(
        sin_q[0], sin_q[1], sin_q[2], sin_q[3], sin_q[4],
        cos_q[0], cos_q[1], cos_q[2], cos_q[3], cos_q[4],
        error[0], error[1], error[2], error[3], error[4], error[5],
        lambda,
        &delta[0], &delta[1], &delta[2], &delta[3], &delta[4],
        &status,
        &state_reg
    );
    
    // 输出结果
    printf("Joint angles q: [");
    for (int i = 0; i < N; i++)
        printf("%7.4f ", q[i]);
    printf("]\n");
    
    printf("Delta: [");
    for (int i = 0; i < N; i++)
        printf("%8.6f ", delta[i]);
    printf("]\nStatus: %d\nState: 0x%02x (Ready:%d Done:%d Busy:%d)\n", 
           status, state_reg, 
           (state_reg & 0x01) != 0,
           (state_reg & 0x02) != 0,
           (state_reg & 0x04) != 0);
}

void test_so101_zero() {
    printf("\n=== Test 3: SO-101 at Zero Position ===\n");
    const int N = 5;
    
    // 零位姿态 q = [0, 0, 0, 0, 0]
    float q[N] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    // 计算 sin/cos
    float sin_q[N], cos_q[N];
    for (int i = 0; i < N; i++) {
        sin_q[i] = sinf(q[i]);
        cos_q[i] = cosf(q[i]);
    }
    
    // 小的位置误差
    float error[6] = {0.02f, 0.03f, 0.04f, 0.01f, 0.005f, 0.01f};
    float lambda = 0.01f;
    
    // 输出变量
    float delta[N];
    int status;
    int state_reg;
    
    // 调用求解器
    xf_solver_lm_so101(
        sin_q[0], sin_q[1], sin_q[2], sin_q[3], sin_q[4],
        cos_q[0], cos_q[1], cos_q[2], cos_q[3], cos_q[4],
        error[0], error[1], error[2], error[3], error[4], error[5],
        lambda,
        &delta[0], &delta[1], &delta[2], &delta[3], &delta[4],
        &status,
        &state_reg
    );
    
    // 输出结果
    printf("Joint angles q: [");
    for (int i = 0; i < N; i++)
        printf("%7.4f ", q[i]);
    printf("]\n");
    
    printf("Delta: [");
    for (int i = 0; i < N; i++)
        printf("%8.6f ", delta[i]);
    printf("]\nStatus: %d\nState: 0x%02x (Ready:%d Done:%d Busy:%d)\n", 
           status, state_reg, 
           (state_reg & 0x01) != 0,
           (state_reg & 0x02) != 0,
           (state_reg & 0x04) != 0);
}

int main() {
    printf("========================================\n");
    printf("  SO-101 LM Solver Test (sin/cos input)\n");
    printf("========================================\n");
    
    // ========================================
    // Part 1: Single-step tests (对应 tb.cpp 原有测试)
    // ========================================
    test_so101_3d();
    test_so101_6d();
    test_so101_zero();
    
    // ========================================
    // Part 2: Iterative solving tests
    // ========================================
    printf("\n\n========================================\n");
    printf("  Iterative LM Solving Tests\n");
    printf("========================================\n");
    
    // 辅助函数：计算向量范数
    auto norm = [](float *v, int n) {
        float sum = 0;
        for (int i = 0; i < n; i++) sum += v[i] * v[i];
        return sqrtf(sum);
    };
    
    // Test Case 1: 从零位求解到指定位置 [0.3, 0, 0.2]
    {
        printf("\n[Test Case 1: From home position to target [0.3, 0.0, 0.2]]\n");
        
        const int N = 5;
        float q[N] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        
        // 构建目标位姿矩阵（仅设置位置，姿态保持单位矩阵）
        float T_target[16];
        mat4_identity(T_target);
        T_target[3]  = 0.3f;   // x
        T_target[7]  = 0.0f;   // y
        T_target[11] = 0.2f;   // z
        
        float lambda = 0.01f;
        int ilimit = 100;
        float tol = 1e-5f;
        
        float delta[N];
        int status;
        int state_reg;
        
        printf("Initial q: [0.0, 0.0, 0.0, 0.0, 0.0]\n");
        printf("Target:    [%.3f, %.3f, %.3f]\n", T_target[3], T_target[7], T_target[11]);
        printf("\nIteration | Error norm | Delta norm | Lambda   | Status | State\n");
        printf("-----------------------------------------------------------------------\n");
        
        for (int it = 0; it < ilimit; it++) {
            // 计算当前位姿的FK
            float T_current[16];
            fk_so101(q, T_current);
            
            // 计算6D误差向量
            float error[6];
            compute_error(T_current, T_target, error);
            
            // 准备sin/cos输入
            float sin_q[N], cos_q[N];
            for (int i = 0; i < N; i++) {
                sin_q[i] = sinf(q[i]);
                cos_q[i] = cosf(q[i]);
            }
            
            // 调用求解器
            xf_solver_lm_so101(
                sin_q[0], sin_q[1], sin_q[2], sin_q[3], sin_q[4],
                cos_q[0], cos_q[1], cos_q[2], cos_q[3], cos_q[4],
                error[0], error[1], error[2], error[3], error[4], error[5],
                lambda,
                &delta[0], &delta[1], &delta[2], &delta[3], &delta[4],
                &status,
                &state_reg
            );
            
            float delta_norm = norm(delta, N);
            
            // 计算误差范数（位置误差 + 姿态误差）
            float error_norm = sqrtf(error[0]*error[0] + error[1]*error[1] + error[2]*error[2] +
                                     error[3]*error[3] + error[4]*error[4] + error[5]*error[5]);
            
            printf("%8d | %10.6e | %10.6e | %8.1e | %6d | 0x%02x\n", 
                   it, error_norm, delta_norm, lambda, status, state_reg);
            
            // 检查状态寄存器
            if (!(state_reg & 0x01)) {
                printf("Error: Ready flag not set!\n");
                break;
            }
            if (!(state_reg & 0x02)) {
                printf("Error: Done flag not set!\n");
                break;
            }
            
            // 收敛判断（基于误差范数）
            if (error_norm < tol) {
                printf("\n✓ Converged at iteration %d (error=%.6e)\n", it, error_norm);
                break;
            }
            
            // 更新关节角度
            for (int i = 0; i < N; i++) {
                q[i] += delta[i];
            }
            
            // 动态调整阻尼
            if (delta_norm < 1e-4f) {
                lambda *= 0.8f;
            } else if (delta_norm > 0.05f) {
                lambda *= 1.5f;
            }
        }
        
        printf("\nFinal q: [");
        for (int i = 0; i < N; i++)
            printf("%8.6f ", q[i]);
        printf("]\n");
    }
    
    // Test Case 2: 从非零初位置求解
    {
        printf("\n[Test Case 2: From arbitrary position to nearby target]\n");
        
        const int N = 5;
        float q[N] = {0.1f, -0.2f, 0.3f, -0.1f, 0.15f};
        
        // 先计算初始位置
        float T_init[16];
        fk_so101(q, T_init);
        
        // 目标位置：初始位置 + 小偏移 [0.05, 0.01, 0.02]
        float T_target[16];
        mat4_identity(T_target);
        T_target[3]  = T_init[3]  + 0.05f;  // x + 0.05
        T_target[7]  = T_init[7]  + 0.01f;  // y + 0.01
        T_target[11] = T_init[11] + 0.02f;  // z + 0.02
        
        float lambda = 0.01f;
        int ilimit = 50;
        float tol = 1e-3f;
        
        float delta[N];
        int status;
        int state_reg;
        
        printf("Initial q: [%.1f, %.1f, %.1f, %.1f, %.2f]\n", q[0], q[1], q[2], q[3], q[4]);
        printf("Initial pos: [%.4f, %.4f, %.4f]\n", T_init[3], T_init[7], T_init[11]);
        printf("Target pos:  [%.4f, %.4f, %.4f]\n", T_target[3], T_target[7], T_target[11]);
        printf("\nIteration | Error norm | Delta norm | Lambda   | Status | State\n");
        printf("-----------------------------------------------------------------------\n");
        
        for (int it = 0; it < ilimit; it++) {
            // 计算当前位姿FK
            float T_current[16];
            fk_so101(q, T_current);
            
            // 计算6D误差
            float error[6];
            compute_error(T_current, T_target, error);
            
            // 准备sin/cos输入
            float sin_q[N], cos_q[N];
            for (int i = 0; i < N; i++) {
                sin_q[i] = sinf(q[i]);
                cos_q[i] = cosf(q[i]);
            }
            
            // 调用求解器
            xf_solver_lm_so101(
                sin_q[0], sin_q[1], sin_q[2], sin_q[3], sin_q[4],
                cos_q[0], cos_q[1], cos_q[2], cos_q[3], cos_q[4],
                error[0], error[1], error[2], error[3], error[4], error[5],
                lambda,
                &delta[0], &delta[1], &delta[2], &delta[3], &delta[4],
                &status,
                &state_reg
            );
            
            float delta_norm = norm(delta, N);
            
            // 计算误差范数
            float error_norm = sqrtf(error[0]*error[0] + error[1]*error[1] + error[2]*error[2] +
                                     error[3]*error[3] + error[4]*error[4] + error[5]*error[5]);
            
            printf("%8d | %10.6e | %10.6e | %8.1e | %6d | 0x%02x\n", 
                   it, error_norm, delta_norm, lambda, status, state_reg);
            
            // 检查状态寄存器
            if (!(state_reg & 0x02)) {
                printf("Warning: Done flag not set at iteration %d\n", it);
            }
            
            // 收敛判断（基于误差范数）
            if (error_norm < tol) {
                printf("\n✓ Converged at iteration %d (error=%.6e)\n", it, error_norm);
                break;
            }
            
            // 更新关节角度
            for (int i = 0; i < N; i++) {
                q[i] += delta[i];
            }
            
            // 动态调整阻尼
            if (delta_norm < 1e-4f) {
                lambda *= 0.8f;
            } else if (delta_norm > 0.05f) {
                lambda *= 1.5f;
            }
        }
        
        printf("\nFinal q: [");
        for (int i = 0; i < N; i++)
            printf("%8.6f ", q[i]);
        printf("]\n");
    }
    
    printf("\n========================================\n");
    printf("  Tests completed\n");
    printf("========================================\n");
    return 0;
}
