/*
 * Extkalmanfilter.cpp
 *
 *  Created on: Jun 20, 2025
 *      Author: Barbra Gitonga (barbragitonga@gmail.com)
 *
 */

#include <Ext_Kalman_filter/Extkalmanfilter.h>

ExtKalmanFilter::ExtKalmanFilter(float Pinit, float *Q_var, float *R_var) {
    // state estimates
	state.phi_rad= 0.0f;
	state.theta_rad = 0.0f; // value of phi and theta

	// a priori covariance matrix
	state.P[0] = Pinit;
	state.P[1] = 0.0f;
	state.P[2] = 0.0f;
	state.P[3] = Pinit;

	// process noise covariance matrix
	state.Q[0] = Q_var[0];
	state.Q[1] = Q_var[1];
	//covariance measurement
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
	state.phi_rad = state.phi_rad + dt * (p + tt * (q * sp + r * cp));
	state.theta_rad = state.theta_rad + dt * (q * cp - r * sp);

	// Jacobian of f(x,u)
	sp = sinf(state.phi_rad);
	cp = cosf(state.phi_rad);
	if (fabs(cp) < 1e-6f) cp = 1e-6f;
	float st = sinf(state.theta_rad);
	float ct = cosf(state.theta_rad);
	if (fabs(ct) < 1e-6f) ct = 1e-6f;
	tt = st / ct;

	float A[4] = {
			tt * (q * cp - r * sp),
			(r * cp + q * sp) * ( tt * tt + 1.0f),
			-(r * cp + q * sp),
			0.0f
	};

	// Predict covariance ( T * (A*P + P*' + Q)
	float Ptemp[4] = {
	    dt * (state.Q[0] + 2.0f * A[0] * state.P[0] + A[1] * state.P[1] + A[1] * state.P[2]),
	    dt * (A[0] * state.P[1] + A[2] * state.P[0] + A[1] * state.P[3] + state.P[1] * A[3]),
	    dt * (A[0] * state.P[2] + A[2] * state.P[0] + A[1] * state.P[3] + A[3] * state.P[2]),
	    dt * (state.Q[1] + A[2] * state.P[1] + A[2] * state.P[2] + 2.0f * A[3] * state.P[3])
	};

	state.P[0] += Ptemp[0];
	state.P[1] += Ptemp[1];
	state.P[2] += Ptemp[2];
	state.P[3] += Ptemp[3];
}

void ExtKalmanFilter::update(const MPU6050Data& accel) {
	float ax = accel.accX;
	float ay = accel.accY;
	float az = accel.accZ;
//
//	float acc_norm = sqrt(ax*ax + ay*ay + az*az);
//	if (acc_norm < 1e-6f) return; // Skip update if no acceleration data
//
//	ax /= acc_norm;
//	ay /= acc_norm;
//	az /= acc_norm;

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
	float C[6] = {
			0.0f,
			g * ct,
			-g * cp * ct,
			g * sp * st,
			g * sp * ct,
			g  * cp * st
	};

	float G[9] = {
				state.P[3]*C[1]*C[1] + state.R[0],
				C[1]*C[2]*state.P[2] + C[1]*C[3]*state.P[3],
				C[1]*C[4]*state.P[2] + C[1]*C[5]*state.P[3],
				C[1]*(C[2]*state.P[1] + C[3]*state.P[3]),
				state.R[1] + C[2]*(C[2]*state.P[0] + C[3]*state.P[2]) + C[3]*(C[2]*state.P[1] + C[3]*state.P[3]),
				C[4]*(C[2]*state.P[0] + C[3]*state.P[2]) + C[5]*(C[2]*state.P[1] + C[3]*state.P[3]),
		        C[1]*(C[4]*state.P[1] + C[5]*state.P[3]),
				C[2]*(C[4]*state.P[0] + C[5]*state.P[2]) + C[3]*(C[4]*state.P[1] + C[5]*state.P[3]),
				state.R[2] + C[4]*(C[4]*state.P[0] + C[5]*state.P[2]) + C[5]*(C[4]*state.P[1] + C[5]*state.P[3])
	};

//	float G[9]; // G = (C*P*C' + R)

	float detG =
	    G[0]*(G[4]*G[8] - G[5]*G[7]) -
	    G[1]*(G[3]*G[8] + G[5]*G[6]) +
	    G[2]*(G[3]*G[7] - G[4]*G[6]);

	if (fabs(detG) < 1e-6f) return;  // skip update if unstable

	float invDet = 1.0f / detG;

	if (fabs(invDet) < 1e-6f) return;  // skip update if unstable

	float Ginv[9];  // Inverse of G G^-1 = invDet * adjoint(G)
	Ginv[0] =  invDet * (G[4]*G[8] - G[5]*G[7]);
	Ginv[1] = -invDet * (G[1]*G[8] - G[2]*G[7]);
	Ginv[2] =  invDet * (G[1]*G[5] - G[2]*G[4]);

	Ginv[3] = -invDet * (G[3]*G[8] - G[5]*G[6]);
	Ginv[4] =  invDet * (G[0]*G[8] - G[2]*G[6]);
	Ginv[5] = -invDet * (G[0]*G[5] - G[2]*G[3]);

	Ginv[6] =  invDet * (G[3]*G[7] - G[4]*G[6]);
	Ginv[7] = -invDet * (G[0]*G[7] - G[1]*G[6]);
	Ginv[8] =  invDet * (G[0]*G[4] - G[1]*G[3]);

	// Computing Kalman gain
	float K[6]; // Kalman Gain K = P*C'*Ginv

	K[0] = Ginv[3] * (C[2] * state.P[0] + C[3] * state.P[1]) +
	       Ginv[6] * (C[4] * state.P[0] + C[5] * state.P[1]) +
	       Ginv[0] * (C[1] * state.P[1]);

	K[1] = Ginv[4] * (C[2] * state.P[0] + C[3] * state.P[1]) +
	       Ginv[7] * (C[4] * state.P[0] + C[5] * state.P[1]) +
	       Ginv[1] * (C[1] * state.P[1]);

	K[2] = Ginv[5] * (C[2] * state.P[0] + C[3] * state.P[1]) +
	       Ginv[8] * (C[4] * state.P[0] + C[5] * state.P[1]) +
	       Ginv[2] * (C[1] * state.P[1]);

	// K[3:5] = second row of Kalman Gain matrix (for theta)
	K[3] = Ginv[3] * (C[2] * state.P[2] + C[3] * state.P[3]) +
	       Ginv[6] * (C[4] * state.P[2] + C[5] * state.P[3]) +
	       Ginv[0] * (C[1] * state.P[3]);

	K[4] = Ginv[4] * (C[2] * state.P[2] + C[3] * state.P[3]) +
	       Ginv[7] * (C[4] * state.P[2] + C[5] * state.P[3]) +
	       Ginv[1] * (C[1] * state.P[3]);

	K[5] = Ginv[5] * (C[2] * state.P[2] + C[3] * state.P[3]) +
	       Ginv[8] * (C[4] * state.P[2] + C[5] * state.P[3]) +
	       Ginv[2] * (C[1] * state.P[3]);


	float Ptemp[4];
	// Updating covariance matrix P = ( I - K*C) * P
	Ptemp[0] = - state.P[2] * (C[1]*K[0] + C[3]*K[1] + C[5]*K[2])
	           - state.P[0] * (C[2]*K[1] + C[4]*K[2] - 1.0f);

	Ptemp[1] = - state.P[3] * (C[1]*K[0] + C[3]*K[1] + C[5]*K[2])
	           - state.P[1] * (C[2]*K[1] + C[4]*K[2] - 1.0f);

	Ptemp[2] = - state.P[2] * (C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f)
	           - state.P[0] * (C[2]*K[4] + C[4]*K[5]);

	Ptemp[3] = - state.P[3] * (C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f)
	           - state.P[1] * (C[2]*K[4] + C[4]*K[5]);


	state.P[0] = state.P[0] + Ptemp[0];
	state.P[1] = state.P[1] + Ptemp[1];
	state.P[2] = state.P[2] + Ptemp[2];
	state.P[3] = state.P[3] + Ptemp[3];

	// Applying correction to state matrix x = x + K*(y - h(x,u))
	float y0 = ax - h[0];
	float y1 = ay - h[1];
	float y2 = az - h[2];

	state.phi_rad   += K[0]*y0 + K[1]*y1 + K[2]*y2;
	state.theta_rad += K[3]*y0 + K[4]*y1 + K[5]*y2;

}

AngleEstimate ExtKalmanFilter::getAngle() const {
    AngleEstimate angle;
    angle.roll = state.phi_rad;   // or x[0] * RAD_TO_DEG
    angle.pitch = state.theta_rad; // or x[1] * RAD_TO_DEG
    return angle;
}
