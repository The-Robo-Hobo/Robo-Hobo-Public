#!/usr/bin/env python3
import RPi.GPIO as GPIO
import sys

SIREN_PIN = 20 # PIN 38
FLASH_PIN = 16 # PIN 36

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(FLASH_PIN, GPIO.OUT)
GPIO.setup(SIREN_PIN, GPIO.OUT)

GPIO.output(FLASH_PIN, GPIO.HIGH)
GPIO.output(SIREN_PIN, GPIO.HIGH)


def run_siren():
    GPIO.output(SIREN_PIN, GPIO.LOW)
    print("Siren pin running.")


def run_flash():
    GPIO.output(FLASH_PIN, GPIO.LOW)
    print("Flash pin running.")


def main():
    terminal_inputs = sys.argv
    while True:
        if len(terminal_inputs) == 1:
            run_flash()
        elif len(terminal_inputs) > 1:
            if terminal_inputs[1] == "siren":
                run_siren()
            elif terminal_inputs[1] == 'flash':
                run_flash()

        user_input = input("Press 'q' to stop")
        print(" ")
        if user_input == 'q':
            GPIO.output(FLASH_PIN, GPIO.HIGH)
            GPIO.output(SIREN_PIN, GPIO.HIGH)
            print("Program stopped")
            break


if __name__ == '__main__':
    main()