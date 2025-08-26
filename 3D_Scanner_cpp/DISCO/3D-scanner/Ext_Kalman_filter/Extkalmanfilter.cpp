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
	if (fabs(cp) < 1e-6f) cp = 1e-6f; // clamps
	float tt = tanf(state.theta_rad);

	// state transition x = x + T * f(x,u)
	// calculating phi_dot and theta_dot
	float phi_dot = p + tt * (q * sp + r * cp);
	float theta_dot = q * cp - r * sp;

	// updating phi_rad and theta_rad, the state matrix
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

	float A[16] = {
	    tt*(q*cp - r*sp),   (tt*tt + 1) * (r*cp* + q*sp),   0.0f,  0.0f,
		-r*cp - q*sp,         0.0f,                         0.0f,  0.0f,
	    0.0f,                0.0f,                          0.0f,  0.0f,
	    0.0f,                0.0f,                          0.0f,  0.0f
	};

//  Update symmetric 4x4 Ptemp matrix from: Ptemp = dt * (A*P + P*A' + Q)
	float Ptemp[16];

	Ptemp[0]  = dt * (state.Q[0] + state.P[0]*A[0] + state.P[1]*A[1] + state.P[2]*A[2] + state.P[3]*A[3]
	                           + A[0]*state.P[0] + A[1]*state.P[4] + A[2]*state.P[8] + A[3]*state.P[12]);

	Ptemp[1]  = dt * (state.P[0]*A[4] + state.P[1]*A[5] + state.P[2]*A[6] + state.P[3]*A[7]
	                           + A[0]*state.P[1] + A[1]*state.P[5] + A[2]*state.P[9] + A[3]*state.P[13]);

	Ptemp[2]  = dt * (state.P[0]*A[8] + state.P[1]*A[9] + state.P[2]*A[10] + state.P[3]*A[11]
	                           + A[0]*state.P[2] + A[1]*state.P[6] + A[2]*state.P[10] + A[3]*state.P[14]);

	Ptemp[3]  = dt * (state.P[0]*A[12] + state.P[1]*A[13] + state.P[2]*A[14] + state.P[3]*A[15]
	                           + A[0]*state.P[3] + A[1]*state.P[7] + A[2]*state.P[11] + A[3]*state.P[15]);

	Ptemp[4]  = dt * (state.P[4]*A[0] + state.P[5]*A[1] + state.P[6]*A[2] + state.P[7]*A[3]
	                           + A[4]*state.P[0] + A[5]*state.P[4] + A[6]*state.P[8] + A[7]*state.P[12]);

	Ptemp[5]  = dt * (state.Q[1] + state.P[4]*A[4] + state.P[5]*A[5] + state.P[6]*A[6] + state.P[7]*A[7]
	                           + A[4]*state.P[1] + A[5]*state.P[5] + A[6]*state.P[9] + A[7]*state.P[13]);

	Ptemp[6]  = dt * (state.P[4]*A[8] + state.P[5]*A[9] + state.P[6]*A[10] + state.P[7]*A[11]
	                           + A[4]*state.P[2] + A[5]*state.P[6] + A[6]*state.P[10] + A[7]*state.P[14]);

	Ptemp[7]  = dt * (state.P[4]*A[12] + state.P[5]*A[13] + state.P[6]*A[14] + state.P[7]*A[15]
	                           + A[4]*state.P[3] + A[5]*state.P[7] + A[6]*state.P[11] + A[7]*state.P[15]);

	Ptemp[8]  = dt * (state.P[8]*A[0] + state.P[9]*A[1] + state.P[10]*A[2] + state.P[11]*A[3]
	                           + A[8]*state.P[0] + A[9]*state.P[4] + A[10]*state.P[8] + A[11]*state.P[12]);

	Ptemp[9]  = dt * (state.P[8]*A[4] + state.P[9]*A[5] + state.P[10]*A[6] + state.P[11]*A[7]
	                           + A[8]*state.P[1] + A[9]*state.P[5] + A[10]*state.P[9] + A[11]*state.P[13]);

	Ptemp[10] = dt * (state.Q[2] + state.P[8]*A[8] + state.P[9]*A[9] + state.P[10]*A[10] + state.P[11]*A[11]
	                           + A[8]*state.P[2] + A[9]*state.P[6] + A[10]*state.P[10] + A[11]*state.P[14]);

	Ptemp[11] = dt * (state.P[8]*A[12] + state.P[9]*A[13] + state.P[10]*A[14] + state.P[11]*A[15]
	                           + A[8]*state.P[3] + A[9]*state.P[7] + A[10]*state.P[11] + A[11]*state.P[15]);

	Ptemp[12] = dt * (state.P[12]*A[0] + state.P[13]*A[1] + state.P[14]*A[2] + state.P[15]*A[3]
	                           + A[12]*state.P[0] + A[13]*state.P[4] + A[14]*state.P[8] + A[15]*state.P[12]);

	Ptemp[13] = dt * (state.P[12]*A[4] + state.P[13]*A[5] + state.P[14]*A[6] + state.P[15]*A[7]
	                           + A[12]*state.P[1] + A[13]*state.P[5] + A[14]*state.P[9] + A[15]*state.P[13]);

	Ptemp[14] = dt * (state.P[12]*A[8] + state.P[13]*A[9] + state.P[14]*A[10] + state.P[15]*A[11]
	                           + A[12]*state.P[2] + A[13]*state.P[6] + A[14]*state.P[10] + A[15]*state.P[14]);

	Ptemp[15] = dt * (state.Q[3] + state.P[12]*A[12] + state.P[13]*A[13] + state.P[14]*A[14] + state.P[15]*A[15]
	                           + A[12]*state.P[3] + A[13]*state.P[7] + A[14]*state.P[11] + A[15]*state.P[15]);


	for (int i = 0; i < 16; i++) {
	    state.P[i] += Ptemp[i];
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


// S = (C*P*C' + R)
	float S[9];

	S[0] = state.R[0] +
	       C[0]*(C[0]*state.P[0]  + C[1]*state.P[4]  + C[2]*state.P[8]  + C[3]*state.P[12]) +
	       C[1]*(C[0]*state.P[1]  + C[1]*state.P[5]  + C[2]*state.P[9]  + C[3]*state.P[13]) +
	       C[2]*(C[0]*state.P[2]  + C[1]*state.P[6]  + C[2]*state.P[10] + C[3]*state.P[14]) +
	       C[3]*(C[0]*state.P[3]  + C[1]*state.P[7]  + C[2]*state.P[11] + C[3]*state.P[15]);

	S[1] = C[4]*(C[0]*state.P[0]  + C[1]*state.P[4]  + C[2]*state.P[8]  + C[3]*state.P[12]) +
	       C[5]*(C[0]*state.P[1]  + C[1]*state.P[5]  + C[2]*state.P[9]  + C[3]*state.P[13]) +
	       C[6]*(C[0]*state.P[2]  + C[1]*state.P[6]  + C[2]*state.P[10] + C[3]*state.P[14]) +
	       C[7]*(C[0]*state.P[3]  + C[1]*state.P[7]  + C[2]*state.P[11] + C[3]*state.P[15]);

	S[2] = C[8]*(C[0]*state.P[0]  + C[1]*state.P[4]  + C[2]*state.P[8]  + C[3]*state.P[12]) +
	       C[9]*(C[0]*state.P[1]  + C[1]*state.P[5]  + C[2]*state.P[9]  + C[3]*state.P[13]) +
	       C[10]*(C[0]*state.P[2] + C[1]*state.P[6]  + C[2]*state.P[10] + C[3]*state.P[14]) +
	       C[11]*(C[0]*state.P[3] + C[1]*state.P[7]  + C[2]*state.P[11] + C[3]*state.P[15]);

	S[3] = S[1]; // S is symmetric, so S[3] = S[1]

	S[4] = state.R[1] +
	       C[4]*(C[4]*state.P[0]  + C[5]*state.P[4]  + C[6]*state.P[8]  + C[7]*state.P[12]) +
	       C[5]*(C[4]*state.P[1]  + C[5]*state.P[5]  + C[6]*state.P[9]  + C[7]*state.P[13]) +
	       C[6]*(C[4]*state.P[2]  + C[5]*state.P[6]  + C[6]*state.P[10] + C[7]*state.P[14]) +
	       C[7]*(C[4]*state.P[3]  + C[5]*state.P[7]  + C[6]*state.P[11] + C[7]*state.P[15]);

	S[5] = C[8]*(C[4]*state.P[0]  + C[5]*state.P[4]  + C[6]*state.P[8]  + C[7]*state.P[12]) +
	       C[9]*(C[4]*state.P[1]  + C[5]*state.P[5]  + C[6]*state.P[9]  + C[7]*state.P[13]) +
	       C[10]*(C[4]*state.P[2] + C[5]*state.P[6]  + C[6]*state.P[10] + C[7]*state.P[14]) +
	       C[11]*(C[4]*state.P[3] + C[5]*state.P[7]  + C[6]*state.P[11] + C[7]*state.P[15]);

	S[6] = S[2]; // S is symmetric, so S[6] = S[2]
	S[7] = S[5]; // S is symmetric, so S[7] = S[5]

	S[8] = state.R[2] +
	       C[8]*(C[8]*state.P[0]  + C[9]*state.P[4]  + C[10]*state.P[8]  + C[11]*state.P[12]) +
	       C[9]*(C[8]*state.P[1]  + C[9]*state.P[5]  + C[10]*state.P[9]  + C[11]*state.P[13]) +
	       C[10]*(C[8]*state.P[2] + C[9]*state.P[6]  + C[10]*state.P[10] + C[11]*state.P[14]) +
	       C[11]*(C[8]*state.P[3] + C[9]*state.P[7]  + C[10]*state.P[11] + C[11]*state.P[15]);


	// Calculating inverse of S

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

	// Computing Kalman gain
	float K[12]; // Kalman Gain K = P*C'*Ginv

	K[0] = Sinv[0]*(C[0]*state.P[0]  + C[1]*state.P[1]  + C[2]*state.P[2]  + C[3]*state.P[3]) +
	       Sinv[3]*(C[4]*state.P[0]  + C[5]*state.P[1]  + C[6]*state.P[2]  + C[7]*state.P[3]) +
	       Sinv[6]*(C[8]*state.P[0]  + C[9]*state.P[1]  + C[10]*state.P[2] + C[11]*state.P[3]);

	K[1] = Sinv[1]*(C[0]*state.P[0]  + C[1]*state.P[1]  + C[2]*state.P[2]  + C[3]*state.P[3]) +
	       Sinv[4]*(C[4]*state.P[0]  + C[5]*state.P[1]  + C[6]*state.P[2]  + C[7]*state.P[3]) +
	       Sinv[7]*(C[8]*state.P[0]  + C[9]*state.P[1]  + C[10]*state.P[2] + C[11]*state.P[3]);

	K[2] = Sinv[2]*(C[0]*state.P[0]  + C[1]*state.P[1]  + C[2]*state.P[2]  + C[3]*state.P[3]) +
	       Sinv[5]*(C[4]*state.P[0]  + C[5]*state.P[1]  + C[6]*state.P[2]  + C[7]*state.P[3]) +
	       Sinv[8]*(C[8]*state.P[0]  + C[9]*state.P[1]  + C[10]*state.P[2] + C[11]*state.P[3]);

	K[3] = Sinv[0]*(C[0]*state.P[4]  + C[1]*state.P[5]  + C[2]*state.P[6]  + C[3]*state.P[7]) +
	       Sinv[3]*(C[4]*state.P[4]  + C[5]*state.P[5]  + C[6]*state.P[6]  + C[7]*state.P[7]) +
	       Sinv[6]*(C[8]*state.P[4]  + C[9]*state.P[5]  + C[10]*state.P[6] + C[11]*state.P[7]);

	K[4] = Sinv[1]*(C[0]*state.P[4]  + C[1]*state.P[5]  + C[2]*state.P[6]  + C[3]*state.P[7]) +
	       Sinv[4]*(C[4]*state.P[4]  + C[5]*state.P[5]  + C[6]*state.P[6]  + C[7]*state.P[7]) +
	       Sinv[7]*(C[8]*state.P[4]  + C[9]*state.P[5]  + C[10]*state.P[6] + C[11]*state.P[7]);

	K[5] = Sinv[2]*(C[0]*state.P[4]  + C[1]*state.P[5]  + C[2]*state.P[6]  + C[3]*state.P[7]) +
	       Sinv[5]*(C[4]*state.P[4]  + C[5]*state.P[5]  + C[6]*state.P[6]  + C[7]*state.P[7]) +
	       Sinv[8]*(C[8]*state.P[4]  + C[9]*state.P[5]  + C[10]*state.P[6] + C[11]*state.P[7]);

	K[6] = Sinv[0]*(C[0]*state.P[8]  + C[1]*state.P[9]  + C[2]*state.P[10] + C[3]*state.P[11]) +
	       Sinv[3]*(C[4]*state.P[8]  + C[5]*state.P[9]  + C[6]*state.P[10] + C[7]*state.P[11]) +
	       Sinv[6]*(C[8]*state.P[8]  + C[9]*state.P[9]  + C[10]*state.P[10]+ C[11]*state.P[11]);

	K[7] = Sinv[1]*(C[0]*state.P[8]  + C[1]*state.P[9]  + C[2]*state.P[10] + C[3]*state.P[11]) +
	       Sinv[4]*(C[4]*state.P[8]  + C[5]*state.P[9]  + C[6]*state.P[10] + C[7]*state.P[11]) +
	       Sinv[7]*(C[8]*state.P[8]  + C[9]*state.P[9]  + C[10]*state.P[10]+ C[11]*state.P[11]);

	K[8] = Sinv[2]*(C[0]*state.P[8]  + C[1]*state.P[9]  + C[2]*state.P[10] + C[3]*state.P[11]) +
	       Sinv[5]*(C[4]*state.P[8]  + C[5]*state.P[9]  + C[6]*state.P[10] + C[7]*state.P[11]) +
	       Sinv[8]*(C[8]*state.P[8]  + C[9]*state.P[9]  + C[10]*state.P[10]+ C[11]*state.P[11]);

	K[9] = Sinv[0]*(C[0]*state.P[12] + C[1]*state.P[13] + C[2]*state.P[14] + C[3]*state.P[15]) +
	       Sinv[3]*(C[4]*state.P[12] + C[5]*state.P[13] + C[6]*state.P[14] + C[7]*state.P[15]) +
	       Sinv[6]*(C[8]*state.P[12] + C[9]*state.P[13] + C[10]*state.P[14]+ C[11]*state.P[15]);

	K[10] = Sinv[1]*(C[0]*state.P[12] + C[1]*state.P[13] + C[2]*state.P[14] + C[3]*state.P[15]) +
	        Sinv[4]*(C[4]*state.P[12] + C[5]*state.P[13] + C[6]*state.P[14] + C[7]*state.P[15]) +
	        Sinv[7]*(C[8]*state.P[12] + C[9]*state.P[13] + C[10]*state.P[14]+ C[11]*state.P[15]);

	K[11] = Sinv[2]*(C[0]*state.P[12] + C[1]*state.P[13] + C[2]*state.P[14] + C[3]*state.P[15]) +
	        Sinv[5]*(C[4]*state.P[12] + C[5]*state.P[13] + C[6]*state.P[14] + C[7]*state.P[15]) +
	        Sinv[8]*(C[8]*state.P[12] + C[9]*state.P[13] + C[10]*state.P[14]+ C[11]*state.P[15]);



	// Applying correction to state matrix x = x + K*(y - h(x,u))
	float y0 = ax - h[0] ;
	float y1 = ay - h[1];
	float y2 = az - h[2];

	state.phi_rad += K[0]*y0 + K[1]*y1 + K[2]*y2;
	state.theta_rad += K[3]*y0 + K[4]*y1 + K[5]*y2;
	state.bias_phi += K[6]*y0 + K[7]*y1 + K[8]*y2;  // Row 2 of K
	state.bias_theta += K[9]*y0 + K[10]*y1 + K[11]*y2; // Row 3 of K

	float Pt[16];

	Pt[0]  = - state.P[0]*(C[0]*K[0]  + C[4]*K[1]  + C[8]*K[2]  - 1.0f)
	         - state.P[4]*(C[1]*K[0]  + C[5]*K[1]  + C[9]*K[2])
	         - state.P[8]*(C[2]*K[0]  + C[6]*K[1]  + C[10]*K[2])
	         - state.P[12]*(C[3]*K[0] + C[7]*K[1]  + C[11]*K[2]);

	Pt[1]  = - state.P[1]*(C[0]*K[0]  + C[4]*K[1]  + C[8]*K[2]  - 1.0f)
	         - state.P[5]*(C[1]*K[0]  + C[5]*K[1]  + C[9]*K[2])
	         - state.P[9]*(C[2]*K[0]  + C[6]*K[1]  + C[10]*K[2])
	         - state.P[13]*(C[3]*K[0] + C[7]*K[1]  + C[11]*K[2]);

	Pt[2]  = - state.P[2]*(C[0]*K[0]  + C[4]*K[1]  + C[8]*K[2]  - 1.0f)
	         - state.P[6]*(C[1]*K[0]  + C[5]*K[1]  + C[9]*K[2])
	         - state.P[10]*(C[2]*K[0] + C[6]*K[1]  + C[10]*K[2])
	         - state.P[14]*(C[3]*K[0] + C[7]*K[1]  + C[11]*K[2]);

	Pt[3]  = - state.P[3]*(C[0]*K[0]  + C[4]*K[1]  + C[8]*K[2]  - 1.0f)
	         - state.P[7]*(C[1]*K[0]  + C[5]*K[1]  + C[9]*K[2])
	         - state.P[11]*(C[2]*K[0] + C[6]*K[1]  + C[10]*K[2])
	         - state.P[15]*(C[3]*K[0] + C[7]*K[1]  + C[11]*K[2]);

	Pt[4]  = - state.P[0]*(C[0]*K[3]  + C[4]*K[4]  + C[8]*K[5])
	         - state.P[4]*(C[1]*K[3]  + C[5]*K[4]  + C[9]*K[5]  - 1.0f)
	         - state.P[8]*(C[2]*K[3]  + C[6]*K[4]  + C[10]*K[5])
	         - state.P[12]*(C[3]*K[3] + C[7]*K[4]  + C[11]*K[5]);

	Pt[5]  = - state.P[1]*(C[0]*K[3]  + C[4]*K[4]  + C[8]*K[5])
	         - state.P[5]*(C[1]*K[3]  + C[5]*K[4]  + C[9]*K[5]  - 1.0f)
	         - state.P[9]*(C[2]*K[3]  + C[6]*K[4]  + C[10]*K[5])
	         - state.P[13]*(C[3]*K[3] + C[7]*K[4]  + C[11]*K[5]);

	Pt[6]  = - state.P[2]*(C[0]*K[3]  + C[4]*K[4]  + C[8]*K[5])
	         - state.P[6]*(C[1]*K[3]  + C[5]*K[4]  + C[9]*K[5]  - 1.0f)
	         - state.P[10]*(C[2]*K[3] + C[6]*K[4]  + C[10]*K[5])
	         - state.P[14]*(C[3]*K[3] + C[7]*K[4]  + C[11]*K[5]);

	Pt[7]  = - state.P[3]*(C[0]*K[3]  + C[4]*K[4]  + C[8]*K[5])
	         - state.P[7]*(C[1]*K[3]  + C[5]*K[4]  + C[9]*K[5]  - 1.0f)
	         - state.P[11]*(C[2]*K[3] + C[6]*K[4]  + C[10]*K[5])
	         - state.P[15]*(C[3]*K[3] + C[7]*K[4]  + C[11]*K[5]);

	Pt[8]  = - state.P[0]*(C[0]*K[6]  + C[4]*K[7]  + C[8]*K[8])
	         - state.P[4]*(C[1]*K[6]  + C[5]*K[7]  + C[9]*K[8])
	         - state.P[8]*(C[2]*K[6]  + C[6]*K[7]  + C[10]*K[8] - 1.0f)
	         - state.P[12]*(C[3]*K[6] + C[7]*K[7]  + C[11]*K[8]);

	Pt[9]  = - state.P[1]*(C[0]*K[6]  + C[4]*K[7]  + C[8]*K[8])
	         - state.P[5]*(C[1]*K[6]  + C[5]*K[7]  + C[9]*K[8])
	         - state.P[9]*(C[2]*K[6]  + C[6]*K[7]  + C[10]*K[8] - 1.0f)
	         - state.P[13]*(C[3]*K[6] + C[7]*K[7]  + C[11]*K[8]);

	Pt[10] = - state.P[2]*(C[0]*K[6]  + C[4]*K[7]  + C[8]*K[8])
	         - state.P[6]*(C[1]*K[6]  + C[5]*K[7]  + C[9]*K[8])
	         - state.P[10]*(C[2]*K[6] + C[6]*K[7]  + C[10]*K[8] - 1.0f)
	         - state.P[14]*(C[3]*K[6] + C[7]*K[7]  + C[11]*K[8]);

	Pt[11] = - state.P[3]*(C[0]*K[6]  + C[4]*K[7]  + C[8]*K[8])
	         - state.P[7]*(C[1]*K[6]  + C[5]*K[7]  + C[9]*K[8])
	         - state.P[11]*(C[2]*K[6] + C[6]*K[7]  + C[10]*K[8] - 1.0f)
	         - state.P[15]*(C[3]*K[6] + C[7]*K[7]  + C[11]*K[8]);

	Pt[12] = - state.P[0]*(C[0]*K[9]  + C[4]*K[10]  + C[8]*K[11])
	         - state.P[4]*(C[1]*K[9]  + C[5]*K[10]  + C[9]*K[11])
	         - state.P[8]*(C[2]*K[9]  + C[6]*K[10]  + C[10]*K[11])
	         - state.P[12]*(C[3]*K[9] + C[7]*K[10]  + C[11]*K[11] - 1.0f);

	Pt[13] = - state.P[1]*(C[0]*K[9]  + C[4]*K[10]  + C[8]*K[11])
	         - state.P[5]*(C[1]*K[9]  + C[5]*K[10]  + C[9]*K[11])
	         - state.P[9]*(C[2]*K[9]  + C[6]*K[10]  + C[10]*K[11])
	         - state.P[13]*(C[3]*K[9] + C[7]*K[10]  + C[11]*K[11] - 1.0f);

	Pt[14] = - state.P[2]*(C[0]*K[9]  + C[4]*K[10]  + C[8]*K[11])
	         - state.P[6]*(C[1]*K[9]  + C[5]*K[10]  + C[9]*K[11])
	         - state.P[10]*(C[2]*K[9] + C[6]*K[10]  + C[10]*K[11])
	         - state.P[14]*(C[3]*K[9] + C[7]*K[10]  + C[11]*K[11] - 1.0f);

	Pt[15] = - state.P[3]*(C[0]*K[9]  + C[4]*K[10]  + C[8]*K[11])
	         - state.P[7]*(C[1]*K[9]  + C[5]*K[10]  + C[9]*K[11])
	         - state.P[11]*(C[2]*K[9] + C[6]*K[10]  + C[10]*K[11])
	         - state.P[15]*(C[3]*K[9] + C[7]*K[10]  + C[11]*K[11] - 1.0f);



	for (int i = 0; i < 16; i++) {
		state.P[i] = Pt[i];
	}


}

AngleEstimate ExtKalmanFilter::getAngle() const {
    AngleEstimate angle;
    angle.roll = state.phi_rad * RAD_TO_DEG;   // or x[0] * RAD_TO_DEG
    angle.pitch = state.theta_rad * RAD_TO_DEG; // or x[1] * RAD_TO_DEG
    return angle;
}
