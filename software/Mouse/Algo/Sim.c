#include "FMicro.cpp"
#include "API.c"

void main() {
    initialize();
    while(true)
        doRun();
}