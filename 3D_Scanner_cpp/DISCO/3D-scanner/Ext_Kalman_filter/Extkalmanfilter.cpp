/*
 * Extkalmanfilter.cpp
 *
 *  Created on: Jun 20, 2025
 *      Author: Barbra Gitonga (barbragitonga@gmail.com)
 *
 */

#include <Ext_Kalman_filter/Extkalmanfilter.h>

ExtKalmanFilter::ExtKalmanFilter(float Pinit, float *Q_var, float *R_var, float phi_bias, float theta_bias) {
    // state estimates
	state.phi_rad= 0.0f;
	state.theta_rad = 0.0f; // value of phi and theta
	state.bias_phi = phi_bias;
	state.bias_theta = theta_bias;

	// a priori covariance matrix
	for (int i = 0; i < 16; ++i) {
	    state.P[i] = 0.0f;
	}

	state.P[0]  = Pinit; // P[0][0]
	state.P[5]  = Pinit; // P[1][1]
	state.P[10] = Pinit; // P[2][2]
	state.P[15] = Pinit; // P[3][3]

	// process noise covariance matrix (prediction error/ uncertainty of the matrix)
	state.Q[0] = Q_var[0];
	state.Q[1] = (phi_bias);
	state.Q[2] = Q_var[1];
	state.Q[3] = (theta_bias);

	//covariance measurement ( uncertainty of the accelerometer / measurement error)
	state.R[0] = R_var[0];
	state.R[1] = R_var[1];
	state.R[2] = R_var[2];
}

void ExtKalmanFilter::predict(const MPU6050Data& gyro, float dt) {
	float p = gyro.gyroX;
	float q = gyro.gyroY;
	float r = gyro.gyroZ;

	float sp = sinf(state.phi_rad);
	float cp = cosf(state.phi_rad);
	if (fabs(cp) < 1e-6f) cp = 1e-6f;
	float tt = tanf(state.theta_rad);

	// state transition x = x + T * f(x,u)
	float phi_dot = p + tt * (q * sp + r * cp);
	float theta_dot = q * cp - r * sp;

	state.phi_rad += dt * (phi_dot  - state.bias_phi);
	state.theta_rad += dt * (theta_dot - state.bias_theta );

	// Jacobian of f(x,u)
	sp = sinf(state.phi_rad);
	cp = cosf(state.phi_rad);
	if (fabs(cp) < 1e-6f) cp = 1e-6f;
	float st = sinf(state.theta_rad);
	float ct = cosf(state.theta_rad);
	if (fabs(ct) < 1e-6f) ct = 1e-6f;
	tt = st / ct;

//	float A[4] = {
//			tt * (q * cp - r * sp),
//			(r * cp + q * sp) * ( tt * tt + 1.0f),
//			-(r * cp + q * sp),
//			0.0f
//	};

	float A[16] = {
				tt * (q * cp - r * sp), 0.0f, 0.0f, 0.0f ,
				- (r * cp - q * sp), 0.0f, 0.0f, 0.0f,
				 0.0f, 0.0f, 0.0f, 0.0f,
				 0.0f, -1.0f, 0.0f, 0.0f
	};


	// Predict covariance ( T * (A*P + P*A' + Q)
//	float Ptemp[4] = {
//	    dt * (state.Q[0] + 2.0f * A[0] * state.P[0] + A[1] * state.P[1] + A[1] * state.P[2]),
//	    dt * (A[0] * state.P[1] + A[2] * state.P[0] + A[1] * state.P[3] + state.P[1] * A[3]),
//	    dt * (A[0] * state.P[2] + A[2] * state.P[0] + A[1] * state.P[3] + A[3] * state.P[2]),
//	    dt * (state.Q[1] + A[2] * state.P[1] + A[2] * state.P[2] + 2.0f * A[3] * state.P[3])
//	};

	float Ptemp[16];
//
//	// Update symmetric 4x4 Ptemp matrix from: Ptemp = dt * (A*P + P*A' + Q)
//
//	// Row 0
//	Ptemp[0]  = dt * (state.Q[0] + 2 * state.P[0] * A[0]);                                  // (1,1)
//	Ptemp[1]  = dt * (state.Q[0] + state.P[1] * A[0] + state.P[4] * A[0]);                  // (1,2) — P[1,0] * A[0] + P[0,1] * A^T[1,0]
//	Ptemp[2]  = dt * (state.Q[0] + state.P[2] * A[0]);                                      // (1,3)
//	Ptemp[3]  = dt * (state.Q[0] + state.P[3] * A[0]);                                      // (1,4)
//
//	// Row 1
//	Ptemp[4]  = dt * (state.Q[1] + state.P[0] * A[4] + state.P[4] * A[0]);                  // (2,1)
//	Ptemp[5]  = dt * (state.Q[1] + 2 * state.P[5] * A[4]);                                  // (2,2)
//	Ptemp[6]  = dt * (state.Q[1] + state.P[6] * A[4]);                                      // (2,3)
//	Ptemp[7]  = dt * (state.Q[1] + state.P[7] * A[4]);                                      // (2,4)
//
//	// Row 2 (no dependence on A)
//	Ptemp[8]  = dt * (state.Q[2]);                                                         // (3,1)
//	Ptemp[9]  = dt * (state.Q[2]);                                                         // (3,2)
//	Ptemp[10] = dt * (state.Q[2] + 2 * state.P[10]);                                       // (3,3) — or just Q[2] if P is not being used here
//	Ptemp[11] = dt * (state.Q[2]);                                                         // (3,4)
//
//	// Row 3 (no dependence on A)
//	Ptemp[12] = dt * (state.Q[3]);                                                         // (4,1)
//	Ptemp[13] = dt * (state.Q[3]);                                                         // (4,2)
//	Ptemp[14] = dt * (state.Q[3]);                                                         // (4,3)
//	Ptemp[15] = dt * (state.Q[3] + 2 * state.P[15]);                                       // (4,4) — same note as above


	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 4; k++) {
				for (int l = 0; l < 4; l++) {
					sum += A[i*4 + k] * state.P[k*4 + l] * A[j*4 + l];
				}
			}
			Ptemp[i*4 + j] = dt * sum;
			if (i == j) {
				Ptemp[i*4 + j] += dt * state.Q[i];  // Add process noise
			}
		}
	}

	for (int i = 0; i < 16; i++) {
	    state.P[i] = Ptemp[i];
	}


}

void ExtKalmanFilter::update(const MPU6050Data& accel) {
	float ax = accel.accX;
	float ay = accel.accY;
	float az = accel.accZ;

	float acc_norm = sqrt(ax*ax + ay*ay + az*az);
	if (acc_norm < 1e-6f) return; // Skip update if no acceleration data

	ax /= acc_norm;
	ay /= acc_norm;
	az /= acc_norm;

	float sp = sinf(state.phi_rad);
	float cp = cosf(state.phi_rad);
	if (fabs(cp) < 1e-6f) cp = 1e-6f;
	float st = sinf(state.theta_rad);
	float ct = cosf(state.theta_rad);
	if (fabs(ct) < 1e-6f) ct = 1e-6f;

	// sensor model
	float h[3] = {
			g * st,
			-g * ct * sp,
			-g * ct * cp
	};

	// jacobian of h(x,u)
	float C[12] = {
	        0.0f, g * ct, 0.0f, 0.0f,
	        -g * ct * cp, g * st * sp, 0.0f, 0.0f,
	        g * ct * sp, g * st * cp, 0.0f, 0.0f
	    };


// G = (C*P*C' + R)
	// calcuating C * P

//	float G[9] = {
//				state.P[3]*C[1]*C[1] + state.R[0],
//				C[1]*C[2]*state.P[2] + C[1]*C[3]*state.P[3],
//				C[1]*C[4]*state.P[2] + C[1]*C[5]*state.P[3],
//				C[1]*(C[2]*state.P[1] + C[3]*state.P[3]),
//				state.R[1] + C[2]*(C[2]*state.P[0] + C[3]*state.P[2]) + C[3]*(C[2]*state.P[1] + C[3]*state.P[3]),
//				C[4]*(C[2]*state.P[0] + C[3]*state.P[2]) + C[5]*(C[2]*state.P[1] + C[3]*state.P[3]),
//		        C[1]*(C[4]*state.P[1] + C[5]*state.P[3]),
//				C[2]*(C[4]*state.P[0] + C[5]*state.P[2]) + C[3]*(C[4]*state.P[1] + C[5]*state.P[3]),
//				state.R[2] + C[4]*(C[4]*state.P[0] + C[5]*state.P[2]) + C[5]*(C[4]*state.P[1] + C[5]*state.P[3])
//	};

//	float G[9];
//	float c1 = -g * cp * ct;  // C[4]
//	float c2 =  g * sp * ct;  // C[8]
//	G[0] = state.R[0];                          // G(0,0)
//	G[1] = 0.0f;                          // G(0,1)
//	G[2] = 0.0f;                          // G(0,2)
//
//	G[3] = 0.0f;                          // G(1,0)
//	G[4] = state.R[1] + c1 * c1 * state.P[0];         // G(1,1)
//	G[5] = c1 * c2 * state.P[0];                // G(1,2)
//
//	G[6] = 0.0f;                          // G(2,0)
//	G[7] = c1 * c2 * state.P[0];                // G(2,1)
//	G[8] = state.R[2] + c2 * c2 * state.P[0];         // G(2,2)

	float S[9];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 4; k++) {
				for (int l = 0; l < 4; l++) {
					sum += C[i*4 + k] * state.P[k*4 + l] * C[j*4 + l];
				}
			}
			S[i*3 + j] = sum;
			if (i == j) {
				S[i*3 + j] += state.R[i];
			}
		}
	}

	float detS = S[0]*(S[4]*S[8] - S[5]*S[7]) - S[1]*(S[3]*S[8] - S[5]*S[6]) + S[2]*(S[3]*S[7] - S[4]*S[6]);

   if (fabs(detS) < 1e-6f) return;

   float invDet = 1.0f / detS;
   float Sinv[9];
   Sinv[0] = invDet * (S[4]*S[8] - S[5]*S[7]);
   Sinv[1] = -invDet * (S[1]*S[8] - S[2]*S[7]);
   Sinv[2] = invDet * (S[1]*S[5] - S[2]*S[4]);
   Sinv[3] = -invDet * (S[3]*S[8] - S[5]*S[6]);
   Sinv[4] = invDet * (S[0]*S[8] - S[2]*S[6]);
   Sinv[5] = -invDet * (S[0]*S[5] - S[2]*S[3]);
   Sinv[6] = invDet * (S[3]*S[7] - S[4]*S[6]);
   Sinv[7] = -invDet * (S[0]*S[7] - S[1]*S[6]);
   Sinv[8] = invDet * (S[0]*S[4] - S[1]*S[3]);

//	float detG =
//	    G[0]*(G[4]*G[8] - G[5]*G[7]) -
//	    G[1]*(G[3]*G[8] + G[5]*G[6]) +
//	    G[2]*(G[3]*G[7] - G[4]*G[6]);
//
//	if (fabs(detG) < 1e-6f) return;  // skip update if unstable

//	float invDet = 1.0f / detG;
//
//	if (fabs(invDet) < 1e-6f) return;  // skip update if unstable

//	float Ginv[9];  // Inverse of G G^-1 = invDet * adjoint(G)
//	Ginv[0] =  invDet * (G[4]*G[8] - G[5]*G[7]);
//	Ginv[1] = -invDet * (G[1]*G[8] - G[2]*G[7]);
//	Ginv[2] =  invDet * (G[1]*G[5] - G[2]*G[4]);
//
//	Ginv[3] = -invDet * (G[3]*G[8] - G[5]*G[6]);
//	Ginv[4] =  invDet * (G[0]*G[8] - G[2]*G[6]);
//	Ginv[5] = -invDet * (G[0]*G[5] - G[2]*G[3]);
//
//	Ginv[6] =  invDet * (G[3]*G[7] - G[4]*G[6]);
//	Ginv[7] = -invDet * (G[0]*G[7] - G[1]*G[6]);
//	Ginv[8] =  invDet * (G[0]*G[4] - G[1]*G[3]);

	// Computing Kalman gain
	float K[12]; // Kalman Gain K = P*C'*Ginv

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			float sum = 0.0f;
			for (int k = 0; k < 4; k++) {
				for (int l = 0; l < 3; l++) {
					sum += state.P[i*4 + k] * C[l*4 + k] * Sinv[l*3 + j];
				}
			}
			K[i*3 + j] = sum;
		}
	}
	// Aliases for readability
//	float C0 = -g * cp * ct;
//	float C1 =  g * sp * ct;

	// K is 4x3, stored row-major: K[4 * col + row]
//	K[0] = Ginv[0] * (state.P[0]  * C0) + Ginv[3] * (state.P[0]  * C1); // (1,1)
//	K[1] = Ginv[0] * (state.P[4]  * C0) + Ginv[3] * (state.P[4]  * C1); // (2,1)
//	K[2] = Ginv[0] * (state.P[8]  * C0) + Ginv[3] * (state.P[8]  * C1); // (3,1)
//	K[3] = Ginv[0] * (state.P[12] * C0) + Ginv[3] * (state.P[12] * C1); // (4,1)
//
//	K[4] = Ginv[1] * (state.P[0]  * C0) + Ginv[4] * (state.P[0]  * C1); // (1,2)
//	K[5] = Ginv[1] * (state.P[4]  * C0) + Ginv[4] * (state.P[4]  * C1); // (2,2)
//	K[6] = Ginv[1] * (state.P[8]  * C0) + Ginv[4] * (state.P[8]  * C1); // (3,2)
//	K[7] = Ginv[1] * (state.P[12] * C0) + Ginv[4] * (state.P[12] * C1); // (4,2)
//
//	K[8]  = Ginv[2] * (state.P[0]  * C0) + Ginv[5] * (state.P[0]  * C1);  // (1,3)
//	K[9]  = Ginv[2] * (state.P[4]  * C0) + Ginv[5] * (state.P[4]  * C1);  // (2,3)
//	K[10] = Ginv[2] * (state.P[8]  * C0) + Ginv[5] * (state.P[8]  * C1);  // (3,3)
//	K[11] = Ginv[2] * (state.P[12] * C0) + Ginv[5] * (state.P[12] * C1);  // (4,3)


	float I_KC[16];
	    for (int i = 0; i < 4; i++) {
	        for (int j = 0; j < 4; j++) {
	            float sum = 0.0f;
	            for (int k = 0; k < 3; k++) {
	                sum += K[i*3 + k] * C[k*4 + j];
	            }
	            I_KC[i*4 + j] = (i == j ? 1.0f : 0.0f) - sum;
	        }
	    }

	    float Pnew[16];
	    for (int i = 0; i < 4; i++) {
	        for (int j = 0; j < 4; j++) {
	            float sum = 0.0f;
	            for (int k = 0; k < 4; k++) {
	                sum += I_KC[i*4 + k] * state.P[k*4 + j];
	            }
	            Pnew[i*4 + j] = sum;
	        }
	    }

	    for (int i = 0; i < 16; i++) {
	        state.P[i] = Pnew[i];
	    }

//	float Ptemp[4];
//	// Updating covariance matrix P = ( I - K*C) * P
//	// Row 0
//	Ptemp[0] =
//	    - state.P[0]  * (C[0]*K[0] + C[4]*K[1] + C[8]*K[2] - 1.0f)
//	    - state.P[4]  * (C[1]*K[0] + C[5]*K[1] + C[9]*K[2])
//	    - state.P[8]  * (C[2]*K[0] + C[6]*K[1] + C[10]*K[2])
//	    - state.P[12] * (C[3]*K[0] + C[7]*K[1] + C[11]*K[2]);
//
//	Ptemp[1] =
//	    - state.P[1]  * (C[0]*K[0] + C[4]*K[1] + C[8]*K[2] - 1.0f)
//	    - state.P[5]  * (C[1]*K[0] + C[5]*K[1] + C[9]*K[2])
//	    - state.P[9]  * (C[2]*K[0] + C[6]*K[1] + C[10]*K[2])
//	    - state.P[13] * (C[3]*K[0] + C[7]*K[1] + C[11]*K[2]);
//
//	Ptemp[2] =
//	    - state.P[2]  * (C[0]*K[0] + C[4]*K[1] + C[8]*K[2] - 1.0f)
//	    - state.P[6]  * (C[1]*K[0] + C[5]*K[1] + C[9]*K[2])
//	    - state.P[10] * (C[2]*K[0] + C[6]*K[1] + C[10]*K[2])
//	    - state.P[14] * (C[3]*K[0] + C[7]*K[1] + C[11]*K[2]);
//
//	Ptemp[3] =
//	    - state.P[3]  * (C[0]*K[0] + C[4]*K[1] + C[8]*K[2] - 1.0f)
//	    - state.P[7]  * (C[1]*K[0] + C[5]*K[1] + C[9]*K[2])
//	    - state.P[11] * (C[2]*K[0] + C[6]*K[1] + C[10]*K[2])
//	    - state.P[15] * (C[3]*K[0] + C[7]*K[1] + C[11]*K[2]);
//
//	// Row 1
//	Ptemp[4] =
//	    - state.P[0]  * (C[0]*K[3] + C[4]*K[4] + C[8]*K[5])
//	    - state.P[4]  * (C[1]*K[3] + C[5]*K[4] + C[9]*K[5] - 1.0f)
//	    - state.P[8]  * (C[2]*K[3] + C[6]*K[4] + C[10]*K[5])
//	    - state.P[12] * (C[3]*K[3] + C[7]*K[4] + C[11]*K[5]);
//
//	Ptemp[5] =
//	    - state.P[1]  * (C[0]*K[3] + C[4]*K[4] + C[8]*K[5])
//	    - state.P[5]  * (C[1]*K[3] + C[5]*K[4] + C[9]*K[5] - 1.0f)
//	    - state.P[9]  * (C[2]*K[3] + C[6]*K[4] + C[10]*K[5])
//	    - state.P[13] * (C[3]*K[3] + C[7]*K[4] + C[11]*K[5]);
//
//	Ptemp[6] =
//	    - state.P[2]  * (C[0]*K[3] + C[4]*K[4] + C[8]*K[5])
//	    - state.P[6]  * (C[1]*K[3] + C[5]*K[4] + C[9]*K[5] - 1.0f)
//	    - state.P[10] * (C[2]*K[3] + C[6]*K[4] + C[10]*K[5])
//	    - state.P[14] * (C[3]*K[3] + C[7]*K[4] + C[11]*K[5]);
//
//	Ptemp[7] =
//	    - state.P[3]  * (C[0]*K[3] + C[4]*K[4] + C[8]*K[5])
//	    - state.P[7]  * (C[1]*K[3] + C[5]*K[4] + C[9]*K[5] - 1.0f)
//	    - state.P[11] * (C[2]*K[3] + C[6]*K[4] + C[10]*K[5])
//	    - state.P[15] * (C[3]*K[3] + C[7]*K[4] + C[11]*K[5]);
//
//	// Row 2
//	Ptemp[8] =
//	    - state.P[0]  * (C[0]*K[6] + C[4]*K[7] + C[8]*K[8])
//	    - state.P[4]  * (C[1]*K[6] + C[5]*K[7] + C[9]*K[8])
//	    - state.P[8]  * (C[2]*K[6] + C[6]*K[7] + C[10]*K[8] - 1.0f)
//	    - state.P[12] * (C[3]*K[6] + C[7]*K[7] + C[11]*K[8]);
//
//	Ptemp[9] =
//	    - state.P[1]  * (C[0]*K[6] + C[4]*K[7] + C[8]*K[8])
//	    - state.P[5]  * (C[1]*K[6] + C[5]*K[7] + C[9]*K[8])
//	    - state.P[9]  * (C[2]*K[6] + C[6]*K[7] + C[10]*K[8] - 1.0f)
//	    - state.P[13] * (C[3]*K[6] + C[7]*K[7] + C[11]*K[8]);
//
//	Ptemp[10] =
//	    - state.P[2]  * (C[0]*K[6] + C[4]*K[7] + C[8]*K[8])
//	    - state.P[6]  * (C[1]*K[6] + C[5]*K[7] + C[9]*K[8])
//	    - state.P[10] * (C[2]*K[6] + C[6]*K[7] + C[10]*K[8] - 1.0f)
//	    - state.P[14] * (C[3]*K[6] + C[7]*K[7] + C[11]*K[8]);
//
//	Ptemp[11] =
//	    - state.P[3]  * (C[0]*K[6] + C[4]*K[7] + C[8]*K[8])
//	    - state.P[7]  * (C[1]*K[6] + C[5]*K[7] + C[9]*K[8])
//	    - state.P[11] * (C[2]*K[6] + C[6]*K[7] + C[10]*K[8] - 1.0f)
//	    - state.P[15] * (C[3]*K[6] + C[7]*K[7] + C[11]*K[8]);
//
//	// Row 3
//	Ptemp[12] =
//	    - state.P[0]  * (C[0]*K[9]  + C[4]*K[10] + C[8]*K[11])
//	    - state.P[4]  * (C[1]*K[9]  + C[5]*K[10] + C[9]*K[11])
//	    - state.P[8]  * (C[2]*K[9]  + C[6]*K[10] + C[10]*K[11])
//	    - state.P[12] * (C[3]*K[9]  + C[7]*K[10] + C[11]*K[11] - 1.0f);
//
//	Ptemp[13] =
//	    - state.P[1]  * (C[0]*K[9]  + C[4]*K[10] + C[8]*K[11])
//	    - state.P[5]  * (C[1]*K[9]  + C[5]*K[10] + C[9]*K[11])
//	    - state.P[9]  * (C[2]*K[9]  + C[6]*K[10] + C[10]*K[11])
//	    - state.P[13] * (C[3]*K[9]  + C[7]*K[10] + C[11]*K[11] - 1.0f);
//
//	Ptemp[14] =
//	    - state.P[2]  * (C[0]*K[9]  + C[4]*K[10] + C[8]*K[11])
//	    - state.P[6]  * (C[1]*K[9]  + C[5]*K[10] + C[9]*K[11])
//	    - state.P[10] * (C[2]*K[9]  + C[6]*K[10] + C[10]*K[11])
//	    - state.P[14] * (C[3]*K[9]  + C[7]*K[10] + C[11]*K[11] - 1.0f);
//
//	Ptemp[15] =
//	    - state.P[3]  * (C[0]*K[9]  + C[4]*K[10] + C[8]*K[11])
//	    - state.P[7]  * (C[1]*K[9]  + C[5]*K[10] + C[9]*K[11])
//	    - state.P[11] * (C[2]*K[9]  + C[6]*K[10] + C[10]*K[11])
//	    - state.P[15] * (C[3]*K[9]  + C[7]*K[10] + C[11]*K[11] - 1.0f);



//	for (int i = 0; i < 16; ++i) {
//	    state.P[i] = Ptemp[i];
//	}


	// Applying correction to state matrix x = x + K*(y - h(x,u))
	float y0 = ax - h[0] ;
	float y1 = ay - h[1];
	float y2 = az - h[2];

	state.phi_rad += K[0]*y0 + K[1]*y1 + K[2]*y2;
	state.theta_rad += K[3]*y0 + K[4]*y1 + K[5]*y2;
	state.bias_phi += K[6]*y0 + K[7]*y1 + K[8]*y2;  // Row 2 of K
	state.bias_theta += K[9]*y0 + K[10]*y1 + K[11]*y2; // Row 3 of K

}

AngleEstimate ExtKalmanFilter::getAngle() const {
    AngleEstimate angle;
    angle.roll = state.phi_rad;   // or x[0] * RAD_TO_DEG
    angle.pitch = state.theta_rad; // or x[1] * RAD_TO_DEG
    return angle;
}
