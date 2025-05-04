import numpy as np
from ina219 import INA219, DeviceRangeError
import RPi.GPIO as GPIO
from time import sleep


class PowerController:
    def __init__(self, pwm_pin=12, shunt_ohms=0.1, target_power=None, tolerance=0.1):
        self.pwm_pin = pwm_pin
        self.shunt_ohms = shunt_ohms
        self.target_power = target_power
        self.tolerance = tolerance
        self.neutral_duty = 50.0  # 50% duty = 2.05V = neutral point
        self.current_duty = self.neutral_duty  # Start at neutral

        # Initialize INA219
        self.ina = INA219(shunt_ohms, busnum=1)
        self.ina.configure()

        # PWM Setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pi_pwm = GPIO.PWM(self.pwm_pin, 700)
        self.pi_pwm.start(self.current_duty)

        # Voltage range constraints (0.8V-3.3V)
        self.min_duty = 24.24  # 0.8V / 3.3V * 100
        self.max_duty = 100.0

    def read_power(self):
        """Read current power from INA219 sensor"""
        try:
            return self.ina.power() / 1000.0  # Convert mW to W
        except DeviceRangeError as e:
            print(f"Measurement error: {e}")
            return None

    def get_power_delta(self, current_power):
        """Calculate power difference from target"""
        if current_power is None:
            return 0
        return current_power - self.target_power

    def needs_adjustment(self, current_power):
        """Check if power deviation exceeds tolerance"""
        delta = self.get_power_delta(current_power)
        return abs(delta) > self.tolerance

    def set_duty_cycle(self, duty):
        """Set PWM duty cycle within allowed range"""
        duty = max(self.min_duty, min(duty, self.max_duty))
        self.current_duty = duty
        self.pi_pwm.ChangeDutyCycle(duty)
        return duty

    def get_operation_mode(self):
        """Return current operation mode"""
        if self.current_duty > self.neutral_duty + 2:  # Small buffer
            return "Charging Battery"
        elif self.current_duty < self.neutral_duty - 2:
            return "Discharging Battery"
        return "Neutral Operation"

    def cleanup(self):
        self.pi_pwm.stop()
        GPIO.cleanup()


class PSOOptimizer:
    def __init__(self, power_controller):
        self.pc = power_controller
        self.n_particles = 10
        self.max_iter = 20
        self.w = 0.5
        self.c1 = 1.5
        self.c2 = 1.5

        # Initialize particles biased toward expected direction
        power_delta = self.pc.get_power_delta(self.pc.read_power())
        if power_delta > 0:  # Excess power - bias toward higher duty cycles
            self.particles = np.random.uniform(self.pc.neutral_duty, self.pc.max_duty, self.n_particles)
        else:  # Deficit power - bias toward lower duty cycles
            self.particles = np.random.uniform(self.pc.min_duty, self.pc.neutral_duty, self.n_particles)

        self.velocities = np.zeros(self.n_particles)
        self.best_positions = np.copy(self.particles)
        self.best_scores = np.full(self.n_particles, np.inf)
        self.global_best_position = self.pc.neutral_duty
        self.global_best_score = np.inf

    def evaluate(self, duty):
        self.pc.set_duty_cycle(duty)
        sleep(0.05)
        current_power = self.pc.read_power()
        if current_power is None:
            return np.inf
        return abs(current_power - self.pc.target_power)

    def optimize(self):
        for _ in range(self.max_iter):
            for i in range(self.n_particles):
                score = self.evaluate(self.particles[i])

                if score < self.best_scores[i]:
                    self.best_scores[i] = score
                    self.best_positions[i] = self.particles[i]

                if score < self.global_best_score:
                    self.global_best_score = score
                    self.global_best_position = self.particles[i]

            # Update particles
            for i in range(self.n_particles):
                r1, r2 = np.random.random(2)
                cognitive = self.c1 * r1 * (self.best_positions[i] - self.particles[i])
                social = self.c2 * r2 * (self.global_best_position - self.particles[i])
                self.velocities[i] = self.w * self.velocities[i] + cognitive + social
                self.particles[i] += self.velocities[i]
                self.particles[i] = np.clip(self.particles[i], self.pc.min_duty, self.pc.max_duty)

            if self.global_best_score < self.pc.tolerance:
                break

        return self.global_best_position