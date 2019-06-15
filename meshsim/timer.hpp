#ifndef _TIMER_
#define _TIMER_


#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <iostream>
#include <vector>
#include <stack>

class Timer {
private:
    std::stack<clock_t> begins;
    double runtime;
public:
    void printInfo(std::string name, std::string data = "") {
        std::cout << "[" << name << "] " << runtime << " ms  " <<  data <<  std::endl;
    }

    void start() {
        begins.push(clock());
    }

    void stop() {
        clock_t end = clock();
        clock_t begin = begins.top();
        begins.pop();
        runtime = double (end - begin) / CLOCKS_PER_SEC;
        runtime *= 1000;
    }

};

#endif