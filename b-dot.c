#include <stdio.h>
#include <math.h>

typedef struct
{
    float x;
    float y;
    float z;
} Vector3;

typedef struct
{
    Vector3 B_previous;
    float K;
    float dt;
} BDotController;


/*
 * B-Dot Controller initialization
 */
void BDot_Init(BDotController *controller, float K, float dt)
{
    controller->B_previous.x = 0.0f;
    controller->B_previous.y = 0.0f;
    controller->B_previous.z = 0.0f;

    controller->K = K;
    controller->dt = dt;
}


/*
 * B-Dot Algorithm
 *
 * B_dot = (B_current - B_previous) / dt
 *
 * m = -K * B_dot
 */
Vector3 BDot_Update(BDotController *controller, Vector3 B_current)
{
    Vector3 B_dot;
    Vector3 magnetic_moment;

    /*
     * Calculate dB/dt
     */
    B_dot.x = (B_current.x - controller->B_previous.x)
              / controller->dt;

    B_dot.y = (B_current.y - controller->B_previous.y)
              / controller->dt;

    B_dot.z = (B_current.z - controller->B_previous.z)
              / controller->dt;


    /*
     * B-Dot control law
     */
    magnetic_moment.x = -controller->K * B_dot.x;
    magnetic_moment.y = -controller->K * B_dot.y;
    magnetic_moment.z = -controller->K * B_dot.z;


    /*
     * Save current magnetic field
     * for next iteration
     */
    controller->B_previous = B_current;


    return magnetic_moment;
}


int main(void)
{
    BDotController controller;

    /*
     * Example:
     *
     * K  = controller gain
     * dt = sampling period
     *
     * 0.01 s = 100 Hz
     */
    BDot_Init(&controller, 1.0f, 0.01f);


    /*
     * Example magnetometer measurement
     */
    Vector3 B = {
        20.5f,
        -5.2f,
        42.8f
    };


    /*
     * Calculate B-Dot control output
     */
    Vector3 m = BDot_Update(&controller, B);


    printf("Magnetic Moment:\n");

    printf("Mx = %f\n", m.x);
    printf("My = %f\n", m.y);
    printf("Mz = %f\n", m.z);


    return 0;
}
