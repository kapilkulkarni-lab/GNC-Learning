import numpy as np

from gnc_functions import simulate_accel, simulate_gps, simulate_gyro, complementary_filter_step, Kp_comp_eff, quat_to_euler, quat_to_R

def quad_sensors(dt, state, omega, params, kf, q_est, bias, gyro_bias_est, Kp_comp):
    ''' Parameters:
    t - Time input utilized in complementary filter step
    state: [p, q, r, phi, theta, psi, u, v, w, x, y, z] (NED, body velocities)
    omega: rotor speeds (4,) in rad/s 
    '''

    thrusts = params['cT'] * omega**2
    F_cmd = np.sum(thrusts)
    
    pos_meas = simulate_gps(state[9:], noise_std=0.1)
    accel_meas = simulate_accel(F_cmd, params['m'])
    omega_meas = simulate_gyro(state[:3], 0.01, bias) - gyro_bias_est

    kf.predict()
    pos_vel_kf = kf.update(pos_meas)

    Kp_comp_adj = Kp_comp_eff(F_cmd, params['m'], params['g'], Kp_comp)
    
    q_est = complementary_filter_step(q_est, omega_meas, accel_meas, dt, Kp_comp_adj)
    

    yaw, pitch, roll = quat_to_euler(q_est)
    euler = np.array([roll, pitch, yaw])
    vel_body = quat_to_R(q_est).T @ pos_vel_kf[3:]
    sensor_state = np.concatenate([omega_meas, euler, vel_body, pos_vel_kf[0:3]])

    return sensor_state, q_est