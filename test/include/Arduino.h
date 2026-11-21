#pragma once

#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <string>

#define D8 8
#define D9 9
#define A0 0
#define HEX 16

class HardwareSerial {
public:
    std::string input;
    size_t position = 0;

    void begin(unsigned long) {}
    void setInput(const char* value) {
        input = value;
        position = 0;
    }
    int available() {
        return position < input.size() ? static_cast<int>(input.size() - position) : 0;
    }
    int read() {
        return position < input.size() ? static_cast<unsigned char>(input[position++]) : -1;
    }
    template<typename T>
    void print(const T&) {}
    template<typename T, typename U>
    void print(const T&, const U&) {}
    template<typename T>
    void println(const T&) {}
    template<typename T, typename U>
    void println(const T&, const U&) {}
    void println() {}
};

extern HardwareSerial Serial;
