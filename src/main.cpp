#include <iostream>
#include "../include/novatel/gnss.h"

using namespace gnss;

int main(int argc, char **argv) {

	RunGnssThread(); // Start Reading GNSS Data

	double a, t;
//	Position c;
	while (1) {

		Position c = GNSS::getInstance()->GetBestpos();
		a = GNSS::getInstance()->GetTimeBestpos();
		boost::posix_time::ptime present_time(
		boost::posix_time::microsec_clock::universal_time());
		boost::posix_time::time_duration duration(present_time.time_of_day());
		t = (duration.total_milliseconds()) / 1000.0;

//		printf("mt:%8.2f\t ft:%8.2f\t lat:%f\t lon:%f\n", a, t, c.latitude, c.longitude);

		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}

	return 0;
}


