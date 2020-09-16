#include "main.h"

using namespace gnss;

int main(int argc, char **argv) {
	Initialize();
	while (1) {
		Loop();
	}
	return 0;
}


