#include <Arduino.h>
#include <MotorShieldController.h>
#include <MotorController.h>


// shield pins
// constexpr auto DIR_LEFT = 5;
// constexpr auto ENABLE_LEFT = 3;
// constexpr auto DIR_RIGHT = 4;
// constexpr auto ENABLE_RIGHT = 2;
// MotorShieldController mc(DIR_LEFT, DIR_RIGHT, ENABLE_LEFT, ENABLE_RIGHT);

constexpr auto L_C = 22;
constexpr auto L_D = 26;

constexpr auto R_C = 24;
constexpr auto R_D = 28;

constexpr auto L_ENABLE = 3;
constexpr auto R_ENABLE = 2;

MotorController leftMc(L_C, L_D, L_ENABLE);
MotorController rightMc(R_C, R_D, R_ENABLE);

void setup() {
	constexpr auto speed = 60;
	leftMc.setVelocity(speed);
	rightMc.setVelocity(-speed);
	// pinMode(L_C, OUTPUT);
	// pinMode(L_D, OUTPUT);
	// pinMode(R_C, OUTPUT);
	// pinMode(R_D, OUTPUT);
}

void loop() {
	// digitalWrite(R_C, LOW);
	// digitalWrite(R_D, HIGH);
	// analogWrite(R_ENABLE, 100);
    leftMc.go();
    rightMc.go();
    delay(1000);
    // coast stop (give 2 seconds)
    leftMc.floatStop();
    rightMc.fastStop();
    delay(2000);

	// mc.setLeftVelocity(50);
	// mc.setRightVelocity(-50);
	// mc.go();
}