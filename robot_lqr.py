from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait


class RobotLQR:

    def __init__(self):
        self.hub = PrimeHub()

        self.left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.E)

        self.hub.imu.reset_heading(0)

        self.circ = 19.48  # cm
        self.global_target_theta = 0



    # -------------------------
    # AUXILIAR
    # -------------------------
    def normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    # -------------------------
    # ANDAR
    # -------------------------
    def andar_cm(self, cm):
        graus = (cm / self.circ) * 360

        start_left = self.left_motor.angle()
        start_right = self.right_motor.angle()

        target = (start_left + start_right) / 2 + graus

        target_theta = self.hub.imu.heading()

        last_avg = (start_left + start_right) / 2
        dt = 0.01

        while True:
            left = self.left_motor.angle()
            right = self.right_motor.angle()

            avg = (left + right) / 2
            theta = self.hub.imu.heading()

            x_error = target - avg
            v = (avg - last_avg) / dt

            theta_error = self.normalize_angle(theta - target_theta)

            angle_factor = max(0, 1 - abs(theta_error) / 35)

            forward = 7 * x_error - 0.4 * v
            forward *= angle_factor

            turn = 6.0 * theta_error

            left_power = forward - turn
            right_power = forward + turn

            max_val = max(abs(left_power), abs(right_power), 100)
            left_power = left_power / max_val * 100
            right_power = right_power / max_val * 100

            self.left_motor.dc(left_power)
            self.right_motor.dc(right_power)

            if abs(x_error) < 1.5:
                break

            last_avg = avg
            wait(10)

        self.left_motor.stop()
        self.right_motor.stop()
        wait(10)

    # -------------------------
    # GIRAR
    # -------------------------
    def virar(self, angulo):
        self.global_target_theta += angulo
        target = self.global_target_theta

        last_theta = self.hub.imu.heading()
        dt = 0.05

        k_theta = 7.0
        k_omega = 0.4

        while True:
            theta = self.hub.imu.heading()

            theta_error = self.normalize_angle(theta - target)
            omega = (theta - last_theta) / dt

            turn = (k_theta * theta_error + k_omega * omega)

            left_power = -turn
            right_power = turn

            left_power = max(min(left_power, 100), -100)
            right_power = max(min(right_power, 100), -100)

            self.left_motor.dc(left_power)
            self.right_motor.dc(right_power)

            if abs(theta_error) < 2.5 and abs(omega) < 5:
                break

            last_theta = theta
            wait(10)

        self.left_motor.stop()
        self.right_motor.stop()
        wait(10)