#include <wiringPi.h>

#include <cstdlib>
#include <iostream>

constexpr int BUTTON_PIN = 27; // GPIO pin (WiringPi pin number, not BCM)

int main() {
	if (wiringPiSetup() == -1) {
		std::cerr << "WiringPi init failed" << std::endl;
		return 1;
	}

	// Configure GPIO pin as input with pull-up
	pinMode(BUTTON_PIN, INPUT);
	pullUpDnControl(BUTTON_PIN, PUD_UP);

	while (true) {
		while (digitalRead(BUTTON_PIN) == HIGH) {
			delay(100);
		}
#if defined(DEBUG) || defined(_DEBUG)
		std::cout << "Button pressed, starting main" << std::endl;
#endif // defined(DEBUG) || defined(_DEBUG)

		// Path to the main program executable

		//TODO
		const char* mainProgramPath = "/path/for/main/program";
		system(mainProgramPath);
	}
	return 0;
}
