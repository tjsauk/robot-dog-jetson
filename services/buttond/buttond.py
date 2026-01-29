cat > ~/projects/robot-dog-jetson/services/buttond/buttond.py << 'EOF'
import Jetson.GPIO as GPIO
import time

BUTTON_PIN = 11  # BOARD pin 11 (J41)

def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(BUTTON_PIN, GPIO.IN)  # no pull_up_down (ignored anyway)

    print("[buttond] started, waiting for button press", flush=True)

    try:
        while True:
            GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)
            print("[buttond] button down", flush=True)
            time.sleep(0.2)  # debounce
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
EOF
