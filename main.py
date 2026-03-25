from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

hub = PrimeHub()

left_motor = Motor(Port.E, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F)

hub.imu.reset_heading(0)

circ = 19.48  # cm

# -------------------------
# ESTADO GLOBAL (ANTI-DRIFT)
# -------------------------
global_target_theta = 0

# -------------------------
# FUNÇÕES AUXILIARES
# -------------------------

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def andar_cm(cm):
    graus = (cm / circ) * 360

    start_left = left_motor.angle()
    start_right = right_motor.angle()

    target = (start_left + start_right) / 2 + graus

    # usa heading atual como referência (sem drift acumulado global)
    target_theta = hub.imu.heading()

    last_avg = (start_left + start_right) / 2
    dt = 0.01

    while True:
        left = left_motor.angle()
        right = right_motor.angle()

        avg = (left + right) / 2
        theta = hub.imu.heading()

        x_error = target - avg

        # velocidade linear (LQR de verdade)
        v = (avg - last_avg) / dt

        # erro angular (mantido no seu padrão que funciona)
        theta_error = normalize_angle(theta - target_theta)

        # -------------------------
        # CONTROLE LQR MELHORADO
        # -------------------------

        # reduz avanço se estiver torto
        angle_factor = max(0, 1 - abs(theta_error) / 35)

        # LQR linear (posição + velocidade)
        forward = 7 * x_error - 0.4 * v
        forward *= angle_factor

        # controle angular
        turn = 6.0 * theta_error

        left_power = forward - turn
        right_power = forward + turn

        # normalização (evita saturação ruim)
        max_val = max(abs(left_power), abs(right_power), 100)
        left_power = left_power / max_val * 100
        right_power = right_power / max_val * 100

        left_motor.dc(left_power)
        right_motor.dc(right_power)

        # parada mais estável (evita drift acumulado)
        if abs(x_error) < 1.5:
            break

        last_avg = avg
        wait(10)

    left_motor.stop()
    right_motor.stop()
    wait(10)

# -------------------------
# GIRAR (LQR estável)
# -------------------------
def girar(angulo):
    global global_target_theta

    global_target_theta += angulo
    target = global_target_theta

    last_theta = hub.imu.heading()
    dt = 0.05

    k_theta = 7.0
    k_omega = 0.4   # ↑ um pouco mais de freio

    while True:
        theta = hub.imu.heading()

        theta_error = normalize_angle(theta - target)
        omega = (theta - last_theta) / dt

        # LQR
        turn = (k_theta * theta_error + k_omega * omega)

        # 🚨 REMOVIDO: força mínima (isso estava travando tudo)
        # if abs(turn) < 8:
        #     turn = 8 if turn > 0 else -8

        left_power = -turn
        right_power = turn

        # ↑ mais força ajuda a “assentar” no final
        left_power = max(min(left_power, 100), -100)
        right_power = max(min(right_power, 100), -100)

        left_motor.dc(left_power)
        right_motor.dc(right_power)

        # 🔥 condição de parada mais realista
        if abs(theta_error) < 2.5 and abs(omega) < 5:
            break

        last_theta = theta
        wait(10)

    # pequeno “freio final” ajuda muito precisão
    left_motor.stop()
    right_motor.stop()
    wait(10)
# -------------------------
# TESTE (QUADRADO)
# -------------------------

# while True:
#     andar_cm(30)
#     girar(90)

andar_cm(-30)
girar(45)
andar_cm(15)
girar(-45)
andar_cm(-90)
