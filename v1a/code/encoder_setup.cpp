#include <wiringPi.h>
#include <cstdio>
#include <stdio.h>

#include <math.h>

#define M1ENC_SIG1_PIN 7  // BCM 4
#define M1ENC_SIG2_PIN 0  // BCM 17
#define M2ENC_SIG1_PIN 28 // BCM 20
#define M2ENC_SIG2_PIN 24 // BCM 19
#define QUADRATURE 16
#define GEAR_RATIO 50.0
#define DEGS_PER_REV 360

long count1 = 0;
long count2 = 0;
long time_count = 0;

FILE *filePtr = fopen("encoder_data.csv", "w");

void handleM1ENC1_rise()
{
    if (digitalRead(M1ENC_SIG2_PIN) == 1)
    {
        ++count1;
    }
    else
    {
        --count1;
    }

    // if (count1 % 50 == 0)
    // {
        printf("count1: %ld", count1);
    // }
}

void handleM2ENC1_rise()
{
    if (digitalRead(M2ENC_SIG2_PIN) == 1)
    {
        ++count2;
    }
    else
    {
        --count2;
    }
}

int setup_encoders()
{

    pinMode(M1ENC_SIG2_PIN, INPUT);
    pinMode(M2ENC_SIG2_PIN, INPUT);

    wiringPiISR(M1ENC_SIG1_PIN, INT_EDGE_RISING, &handleM1ENC1_rise);
    wiringPiISR(M2ENC_SIG1_PIN, INT_EDGE_RISING, &handleM2ENC1_rise);

    return 0;
}

int main()
{

    wiringPiSetup();
    setup_encoders();

    while (1)
    {
        // if (count1 % 25 == 1)
        // {
            printf("count1: %ld\n\r", count1);
        // }
        // if (count2 % 25 == 1)
        // {
            printf("count2: %ld\n\r", count2);
        // }
        // ++time_count;
        // // if (time_count % 10 == 0)
        // // {
        // // // printf("angle1 turned : %f\t angle2 turned : %f\n\r",  (double) (count1/QUADRATURE/GEAR_RATIO*DEGS_PER_REV), (double) (count2/QUADRATURE/GEAR_RATIO*DEGS_PER_REV));
        // // // printf("count1 : %ld\t count2 : %ld\n\r", count1, count2);
        // // // fprintf(filePtr, "%ld, %ld\n", count1, count2);
        // // // printf("angle2 turned : %f\n\r",  (double) (count2/QUADRATURE/GEAR_RATIO*DEGS_PER_REV));
        // // // fprintf(filePtr, "%f, %f\n", (double) (count1/QUADRATURE/GEAR_RATIO*DEGS_PER_REV), (double) (count2/QUADRATURE/GEAR_RATIO*DEGS_PER_REV) );
        // // }
    }
}

// gcc -o enc encoder_setup.cpp -lwiringPi -lm -lncurses -lrt -lcrypt

// gcc -o enc encoder_setup.cpp -lwiringPi -lm -lncurses -lrt -lcrypt -lpthread
// Harness,
// slow down dynamics with more inertia
// use the eqns of motion
// buiild out a simulator
// look into PID based segway control 

// mathematical treatment
// control diagram
