import Jetson.GPIO as GPIO
import time

# BOARD pin numbers
BUTTON_PIN = 11  # J41 pin 11 (GPIO17)

def main():
    # Use physical pin numbering
    GPIO.setmode(GPIO.BOARD)

    # Button as input with pull-up
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print("[buttond] started, waiting for button press", flush=True)

    try:
        while True:
            # Wait for falling edge (button pressed)
            GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)

            print("[buttond] button down", flush=True)

            # Simple debounce
            time.sleep(0.2)

    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
