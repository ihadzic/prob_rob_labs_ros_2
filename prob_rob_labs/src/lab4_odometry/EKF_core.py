# EKF calculation
import numpy as np


class EKFCore:
    def __init__(
        self,
        Gv=0.997422,
        Gw=1.000886,
        tau_v=0.455914,
        tau_w=0.034669,
        wheel_radius=0.033,
        wheel_separation=0.1435,
    ):
        self.Gv = float(Gv)
        self.Gw = float(Gw)
        self.tau_v = float(tau_v)
        self.tau_w = float(tau_w)
        self.rw = float(wheel_radius)
        self.R_robot = float(wheel_separation) / 2.0

        self.x = np.zeros(5, dtype=float)       # initial state: x=[theta, x, y, v, omega]

        # guess for initial state covariance matrix P
        self.P = np.diag([0.01, 0.01, 0.01, 0.01, 0.01])

        # Process noise
        # Q_d = Q_rate * dt
        # guess for process noise rate
        self.Q_rate = np.diag([0.001, 0.001, 0.001, 0.01, 0.01])

        # measurement Rm: z = [wr, wl, wg]
        self.C = np.array([
            [0, 0, 0, 1.0 / self.rw, self.R_robot / self.rw],
            [0, 0, 0, 1.0 / self.rw, -self.R_robot / self.rw],
            [0, 0, 0, 0, 1]
        ], dtype=float)

        # measurement covariance
        sigma_wheel = 0.1  # guess for wheel encoder standard deviation
        gyro_variance = 1.6e-7  # gyro variance obtained from IMU topic
        self.Rm = np.diag([sigma_wheel ** 2, sigma_wheel ** 2, gyro_variance])


    def calculate_dynamic_coefficients(self, dt):
        av = 0.1 ** (dt / self.tau_v)
        aw = 0.1 ** (dt / self.tau_w)
        return av, aw


    # normalize yaw to [-pi, pi]
    @staticmethod
    def normalize_angle(theta):
        return np.arctan2(np.sin(theta), np.cos(theta))

    def state_transition(self, x, u, dt):
        theta, px, py, v, w = x # current state
        uv, uw = u #current inputs

        # dynamic model coefficients from assignment3
        av, aw = self.calculate_dynamic_coefficients(dt)

        # update
        theta_new = theta + w * dt
        theta_new = self.normalize_angle(theta_new)
        px_new = px + v * np.cos(theta) * dt
        py_new = py + v * np.sin(theta) * dt
        v_new = (av * v + self.Gv * (1.0 - av) * uv)
        w_new = (aw * w + self.Gw * (1.0 - aw) * uw)

        return np.array([theta_new, px_new, py_new, v_new, w_new], dtype=float)


    def calculate_jacobian(self, x, dt):
        theta, px, py, v, w = x
        av, aw = self.calculate_dynamic_coefficients(dt)
        # F = df/dX
        F = np.array([
            [1, 0, 0, 0, dt],
            [-v * np.sin(theta) * dt, 1, 0, np.cos(theta) * dt, 0],
            [v * np.cos(theta) * dt, 0, 1, np.sin(theta) * dt, 0],
            [0, 0, 0, av, 0],
            [0, 0, 0, 0, aw]
        ], dtype=float)

        return F

    
    # EKF Prediction
    def predict(self, u, dt):
        u = np.asarray(u, dtype=float).reshape(2)
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError(
                'Prediction dt must be positive and finite.'
            )

        if not np.all(np.isfinite(u)):
            raise ValueError(
                'Control input contains invalid values.'
            )

        # save previous state before prediction
        x_old = self.x.copy()
        # Jacobian evaluated at the previous state
        F = self.calculate_jacobian(x_old, dt)

        # 1. Predict state
        self.x = self.state_transition(x_old, u, dt)

        # 2. Discrete process noise for this interval
        Q_d = self.Q_rate * dt

        # 3. Predict covariance
        self.P = F @ self.P @ F.T + Q_d
        self.P = 0.5 * (self.P + self.P.T)

        return self.x.copy(), self.P.copy()


    def update(self, z):

        z = np.asarray(z, dtype=float).reshape(3)
        z_pred = self.C @ self.x

        y = z - z_pred
        S = self.C @ self.P @ self.C.T + self.Rm
        K = np.linalg.solve(S, self.C @ self.P).T   # kalman gain

        self.x = self.x + K @ y
        self.x[0] = self.normalize_angle(self.x[0])

        I = np.eye(5)
        A = I - K @ self.C
        self.P = A @ self.P @ A.T + K @ self.Rm @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        return self.x.copy(), self.P.copy()


    # Read current state
    def get_state(self):
        return self.x.copy()

    # Read current covariance
    def get_covariance(self):
        return self.P.copy()

    # Reset filter
    def reset(self):
        self.x = np.zeros(5, dtype=float)
        self.P = np.diag([0.01, 0.01, 0.01, 0.01, 0.01])
