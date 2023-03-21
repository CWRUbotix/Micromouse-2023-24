#include <stdio.h>
#include <stdint.h>

/**
 * Convert a value in range [-128..127] to a motor power value
 * 
 * @param p The input power [-128..127]
 * @return Output power [0..255]
 */
uint8_t convert(int8_t p)
{
    if (p == 0) {
        return 255;
    }
    if (p < 0) {
        if (p == -128) {
            p = -127;
        }
        p = -p;
    }
    return 255 - (((uint8_t)p) * 2);
}

int main()
{
    // @Matthias this is actually valid C syntax
    struct {
        int8_t power_in;
        uint8_t power_out;
    } powers[] = {
        {-128, 1}, // 1 out since 255 - (2*127) = 1
        {-127, 1},
        {127, 1},
        {10, 235},
        {-10, 235},
        {0, 255},
        {0, 0} // END TEST
    };


    size_t i = 0;
    int8_t pi;
    uint8_t po, converted;
    while (!(powers[i].power_in == 0 && powers[i].power_out == 0)) {
        pi = powers[i].power_in;
        po = powers[i].power_out;
        converted = convert(pi);
        
        printf(
            "Power in: %04d, power out: %04d, expected: %04d",
            pi,
            converted,
            po
            );

        if (converted == po) {
            printf(" [PASS]\n");
        }
        else {
            printf(" [FAIL]\n");
        }
        
        ++i;
    }
}


