from functions import PowerController, PSOOptimizer
import time

# Configuration
TARGET_POWER = 6.0  # Watts
POWER_TOLERANCE = 0.1  # ±0.1W
PWM_PIN = 12
SHUNT_OHMS = 0.1


def main():
    pc = PowerController(pwm_pin=PWM_PIN,
                         shunt_ohms=SHUNT_OHMS,
                         target_power=TARGET_POWER,
                         tolerance=POWER_TOLERANCE)

    try:
        while True:
            current_power = pc.read_power()
            if current_power is None:
                time.sleep(1)
                continue

            power_delta = current_power - TARGET_POWER
            mode = pc.get_operation_mode()

            print(f"Power: {current_power:.2f}W | Target: {TARGET_POWER}W | Delta: {power_delta:+.2f}W")
            print(f"Mode: {mode} | Duty: {pc.current_duty:.2f}%")

            if pc.needs_adjustment(current_power):
                print("Optimizing...")
                pso = PSOOptimizer(pc)
                optimal_duty = pso.optimize()
                pc.set_duty_cycle(optimal_duty)
                print(f"New Duty: {optimal_duty:.2f}%")

            time.sleep(2)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        pc.cleanup()


if __name__ == "__main__":
    main()