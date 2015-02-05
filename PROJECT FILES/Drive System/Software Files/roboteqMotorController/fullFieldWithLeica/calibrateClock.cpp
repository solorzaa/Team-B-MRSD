#include <iostream>
#include <time.h>

using namespace std;

int main () {

    for(int i = 0; i < 10 ; i++) {

        int first_clock = clock();
        int first_time = time(NULL);

        while(time(NULL) <= first_time) {}

        int second_time = time(NULL);
        int second_clock = clock();

        cout << "Actual clocks per second = " << (second_clock - first_clock)/(second_time - first_time) << "\n";

        cout << "CLOCKS_PER_SEC = " << CLOCKS_PER_SEC << "\n";

    }

    return 0;

}
