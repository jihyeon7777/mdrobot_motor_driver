"""
current_sensor.py  — OROHA v2 Phase 5
Raspberry Pi Pico (MicroPython)

ACS758 전류센서 2개의 raw ADC 값을 20Hz로 시리얼(USB) 전송.
출력 형식 (한 줄에 하나):
    <센서1_raw>,<센서2_raw>
예:
    33208,33016

■ raw 그대로 보낸다 (암페어 변환은 나중에 ROS2/보정 단계에서).
■ Pico에 main.py 로 올리면 전원 인가 시 자동 실행.

배선:
    센서1 → GP26 (ADC0)
    센서2 → GP27 (ADC1)
    센서 전원 3.3V (무전류 시 영점 raw ≈ 33000, 전원 3.3V의 절반)
"""
from machine import ADC, Pin
import time

RATE_HZ = 20                    # 발행 주기 (제어 루프와 맞춤)
PERIOD = 1.0 / RATE_HZ

adc0 = ADC(Pin(26))             # 센서1
adc1 = ADC(Pin(27))             # 센서2


def main():
    next_t = time.ticks_ms()
    while True:
        r0 = adc0.read_u16()
        r1 = adc1.read_u16()
        # CSV 한 줄. print 가 자동으로 개행(\n) 붙여줌 → ROS2 쪽 readline() 과 짝
        print(f"{r0},{r1}")

        # 주기 유지: 다음 발행 시각까지 대기 (드리프트 없는 방식)
        next_t = time.ticks_add(next_t, int(PERIOD * 1000))
        delay = time.ticks_diff(next_t, time.ticks_ms())
        if delay > 0:
            time.sleep_ms(delay)


if __name__ == "__main__":
    main()