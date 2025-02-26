#ifndef KNOB_H
#define KNOB_H

#include <atomic>
#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>

class Knob {
public:
    Knob(uint8_t pinA, uint8_t pinB, int32_t lowerLimit, int32_t upperLimit)
        : pinA(pinA), pinB(pinB), lowerLimit(lowerLimit), upperLimit(upperLimit), rotation(0), prevState(0), lastRotation(0) {}

    void updateRotation() {
        uint8_t currentState = (digitalRead(pinA) << 1) | digitalRead(pinB);
        int8_t rotationChange = 0;

        switch (prevState) {
            case 0b00:
                if (currentState == 0b01) rotationChange = 1;
                else if (currentState == 0b10) rotationChange = -1;
                break;
            case 0b01:
                if (currentState == 0b11) rotationChange = 1;
                else if (currentState == 0b00) rotationChange = -1;
                break;
            case 0b11:
                if (currentState == 0b10) rotationChange = 1;
                else if (currentState == 0b01) rotationChange = -1;
                break;
            case 0b10:
                if (currentState == 0b00) rotationChange = 1;
                else if (currentState == 0b11) rotationChange = -1;
                break;
            default:
                rotationChange = lastRotation;
                break;
        }

        int32_t newRotation = rotation.load(std::memory_order_relaxed) + rotationChange;
        if (newRotation > upperLimit) {
            rotation.store(upperLimit, std::memory_order_relaxed);
        } else if (newRotation < lowerLimit) {
            rotation.store(lowerLimit, std::memory_order_relaxed);
        } else {
            rotation.store(newRotation, std::memory_order_relaxed);
            lastRotation = rotationChange;
            prevState = currentState;
        }
    }

    void setLimits(int32_t lower, int32_t upper) {
        lowerLimit = lower;
        upperLimit = upper;
    }

    int32_t getRotation() const {
        return rotation.load(std::memory_order_relaxed);
    }

private:
    uint8_t pinA;
    uint8_t pinB;
    int32_t lowerLimit;
    int32_t upperLimit;
    std::atomic<int32_t> rotation;
    uint8_t prevState;
    int8_t lastRotation;
};

#endif