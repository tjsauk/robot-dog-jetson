from gpiozero import Button
from signal import pause
import time

PIN = 17  # GPIO17 (physical pin 11)
btn = Button(PIN, pull_up=True, bounce_time=0.03)

def on_press():
    print(f"[buttond] button down @ {time.time():.3f}", flush=True)

btn.when_pressed = on_press
print("[buttond] running. press button to print.", flush=True)
pause()
