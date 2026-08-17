#include <stdio.h>
#include <math.h>

/*
 * ============================================================
 * CUBESAT B-DOT DETUMBLING SIMULATION
 * ============================================================
 *
 * Amaç:
 *
 *   CubeSat başlangıçta dönüyor.
 *   Manyetometre Dünya'nın manyetik alanını ölçüyor.
 *   B-Dot algoritması:
 *
 *          m = -K * dB/dt
 *
 *   ile magnetorquer manyetik momentini hesaplıyor.
 *
 *   Magnetorquer'ın uyduya uyguladığı tork:
 *
 *          tau = m x B
 *
 *   Daha sonra:
 *
 *          tau = I * alpha
 *
 *   ile uyduyun açısal hızı güncelleniyor.
 *
 * ============================================================
 */

#define PI 3.14159265358979323846f

/* ------------------------------------------------------------
 * Simülasyon parametreleri
 * ------------------------------------------------------------ */

/* Simülasyon örnekleme süresi */
#define DT 0.01f                 // 10 ms = 100 Hz

/* Toplam simülasyon süresi */
#define SIMULATION_TIME 60.0f    // 60 saniye

/* B-Dot kontrol kazancı */
#define K_BDOT 3000.0f

/* ------------------------------------------------------------
 * CubeSat atalet momentleri
 *
 * kg.m^2
 *
 * Gerçek CubeSat için CAD / kütle dağılımından elde edilir.
 * Burada örnek değer kullanıyoruz.
 * ------------------------------------------------------------ */

#define IX 0.020f
#define IY 0.025f
#define IZ 0.030f


/* ------------------------------------------------------------
 * Dünya manyetik alanı
 *
 * Burada basitlik için NED benzeri sabit bir referans alan
 * kullanıyoruz.
 *
 * Birim: Tesla
 *
 * Gerçek sistemde bu değer IGRF/WMM gibi modellerden veya
 * simülasyon ortamından gelebilir.
 * ------------------------------------------------------------ */

#define B_REF_X 20.0e-6f
#define B_REF_Y -5.0e-6f
#define B_REF_Z 40.0e-6f


/* ------------------------------------------------------------
 * Magnetorquer sınırı
 *
 * Gerçek donanımın maksimum manyetik momentine göre
 * belirlenmelidir.
 *
 * Burada örnek olarak:
 *
 * ±0.2 A.m^2
 *
 * ------------------------------------------------------------ */

#define MAX_MOMENT 0.20f


/* ------------------------------------------------------------
 * Vector3
 * ------------------------------------------------------------ */

typedef struct
{
    float x;
    float y;
    float z;

} Vector3;


/* ------------------------------------------------------------
 * B-Dot Controller
 * ------------------------------------------------------------ */

typedef struct
{
    Vector3 B_previous;

    float K;

    float dt;

    int initialized;

} BDotController;


/* ------------------------------------------------------------
 * Vektör toplama
 * ------------------------------------------------------------ */

Vector3 Vector_Add(Vector3 a, Vector3 b)
{
    Vector3 result;

    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;

    return result;
}


/* ------------------------------------------------------------
 * Vektör çıkarma
 * ------------------------------------------------------------ */

Vector3 Vector_Sub(Vector3 a, Vector3 b)
{
    Vector3 result;

    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;

    return result;
}


/* ------------------------------------------------------------
 * Skaler çarpma
 * ------------------------------------------------------------ */

Vector3 Vector_Scale(Vector3 v, float k)
{
    Vector3 result;

    result.x = v.x * k;
    result.y = v.y * k;
    result.z = v.z * k;

    return result;
}


/* ------------------------------------------------------------
 * Cross product
 *
 * tau = m x B
 * ------------------------------------------------------------ */

Vector3 Vector_Cross(Vector3 a, Vector3 b)
{
    Vector3 result;

    result.x = a.y * b.z - a.z * b.y;

    result.y = a.z * b.x - a.x * b.z;

    result.z = a.x * b.y - a.y * b.x;

    return result;
}


/* ------------------------------------------------------------
 * Vektör büyüklüğü
 * ------------------------------------------------------------ */

float Vector_Norm(Vector3 v)
{
    return sqrtf(
        v.x * v.x +
        v.y * v.y +
        v.z * v.z
    );
}


/* ------------------------------------------------------------
 * B-Dot Initialization
 * ------------------------------------------------------------ */

void BDot_Init(
    BDotController *controller,
    float K,
    float dt)
{
    controller->B_previous.x = 0.0f;
    controller->B_previous.y = 0.0f;
    controller->B_previous.z = 0.0f;

    controller->K = K;

    controller->dt = dt;

    controller->initialized = 0;
}


/* ------------------------------------------------------------
 * B-Dot Update
 *
 * B_dot = (B_current - B_previous) / dt
 *
 * m = -K * B_dot
 *
 * ------------------------------------------------------------ */

Vector3 BDot_Update(
    BDotController *controller,
    Vector3 B_current)
{
    Vector3 B_dot;

    Vector3 m;


    /*
     * İlk ölçümde dB/dt hesaplanamaz.
     *
     * Çünkü elimizde önceki B yok.
     */

    if (controller->initialized == 0)
    {
        controller->B_previous = B_current;

        controller->initialized = 1;

        m.x = 0.0f;
        m.y = 0.0f;
        m.z = 0.0f;

        return m;
    }


    /*
     * dB/dt
     */

    B_dot.x =
        (B_current.x -
         controller->B_previous.x)
        / controller->dt;

    B_dot.y =
        (B_current.y -
         controller->B_previous.y)
        / controller->dt;

    B_dot.z =
        (B_current.z -
         controller->B_previous.z)
        / controller->dt;


    /*
     * B-Dot control law
     *
     * m = -K * dB/dt
     */

    m.x = -controller->K * B_dot.x;

    m.y = -controller->K * B_dot.y;

    m.z = -controller->K * B_dot.z;


    /*
     * Magnetorquer moment saturation
     *
     * Gerçek bobinlerin maksimum kapasitesini aşmamak
     * için çıkışı sınırlandırıyoruz.
     */

    if (m.x > MAX_MOMENT)
        m.x = MAX_MOMENT;

    if (m.x < -MAX_MOMENT)
        m.x = -MAX_MOMENT;


    if (m.y > MAX_MOMENT)
        m.y = MAX_MOMENT;

    if (m.y < -MAX_MOMENT)
        m.y = -MAX_MOMENT;


    if (m.z > MAX_MOMENT)
        m.z = MAX_MOMENT;

    if (m.z < -MAX_MOMENT)
        m.z = -MAX_MOMENT;


    /*
     * Current B becomes previous B
     */

    controller->B_previous = B_current;


    return m;
}


/* ============================================================
 *
 * Basitleştirilmiş attitude -> body magnetic field modeli
 *
 * ============================================================
 *
 * Dünya'nın manyetik alanı inertial/reference frame'de:
 *
 *      B_ref
 *
 * olarak tanımlı.
 *
 * Uydu döndükçe bu vektör body frame'de değişir.
 *
 * Bu fonksiyon gerçek bir quaternion/DCM propagatörü yerine
 * basitleştirilmiş bir 3-eksen dönüş modeli kullanıyor.
 *
 * Daha ileri aşamada bunu quaternion ile değiştireceğiz.
 *
 * ============================================================
 */

Vector3 Simulate_Magnetometer(
    Vector3 angular_rate,
    float time)
{
    Vector3 B;

    /*
     * Burada basit bir yapay hareket modeli oluşturuyoruz.
     *
     * Amaç:
     *
     * Uydu döndükçe B'nin değişmesini sağlamak.
     *
     * Gerçek bir uçuş simülasyonunda:
     *
     *     attitude
     *        ↓
     *     DCM / Quaternion
     *        ↓
     *     B_body = C_bi * B_inertial
     *
     * kullanılmalıdır.
     */

    float wx = angular_rate.x;
    float wy = angular_rate.y;
    float wz = angular_rate.z;


    /*
     * Basitleştirilmiş manyetometre modeli
     */

    B.x =
        B_REF_X * cosf(wz * time)
        - B_REF_Y * sinf(wz * time);


    B.y =
        B_REF_X * sinf(wz * time)
        + B_REF_Y * cosf(wz * time);


    B.z =
        B_REF_Z
        + 5.0e-6f * sinf(wx * time)
        + 3.0e-6f * cosf(wy * time);


    return B;
}


/* ============================================================
 *
 * MAIN
 *
 * ============================================================
 */

int main(void)
{
    BDotController controller;


    /*
     * B-Dot controller oluştur
     */

    BDot_Init(
        &controller,
        K_BDOT,
        DT
    );


    /*
     * Başlangıç açısal hızı
     *
     * rad/s
     *
     * Örneğin:
     *
     * X = 5 deg/s
     * Y = -3 deg/s
     * Z = 8 deg/s
     *
     */

    Vector3 omega;

    omega.x = 5.0f  * PI / 180.0f;

    omega.y = -3.0f * PI / 180.0f;

    omega.z = 8.0f  * PI / 180.0f;


    /*
     * Simülasyon başlangıcı
     */

    float time = 0.0f;


    /*
     * Kaç adım çalışacağız?
     */

    int steps =
        (int)(SIMULATION_TIME / DT);


    printf("\n");
    printf("=============================================\n");
    printf("       CUBESAT B-DOT SIMULATION\n");
    printf("=============================================\n");

    printf("K = %.2f\n", K_BDOT);

    printf("dt = %.3f s\n", DT);

    printf("Simulation = %.1f s\n", SIMULATION_TIME);

    printf("=============================================\n\n");


    /*
     * Simulation loop
     */

    for (int i = 0; i < steps; i++)
    {
        /*
         * ----------------------------------------------------
         * 1. Magnetometer measurement
         * ----------------------------------------------------
         */

        Vector3 B =
            Simulate_Magnetometer(
                omega,
                time
            );


        /*
         * ----------------------------------------------------
         * 2. B-Dot
         * ----------------------------------------------------
         */

        Vector3 m =
            BDot_Update(
                &controller,
                B
            );


        /*
         * ----------------------------------------------------
         * 3. Magnetic torque
         *
         * tau = m x B
         * ----------------------------------------------------
         */

        Vector3 torque =
            Vector_Cross(
                m,
                B
            );


        /*
         * ----------------------------------------------------
         * 4. Angular acceleration
         *
         * tau = I * alpha
         *
         * alpha = tau / I
         * ----------------------------------------------------
         */

        Vector3 alpha;

        alpha.x =
            torque.x / IX;

        alpha.y =
            torque.y / IY;

        alpha.z =
            torque.z / IZ;


        /*
         * ----------------------------------------------------
         * 5. Angular velocity update
         *
         * omega_new =
         * omega_old + alpha * dt
         * ----------------------------------------------------
         */

        omega.x +=
            alpha.x * DT;

        omega.y +=
            alpha.y * DT;

        omega.z +=
            alpha.z * DT;


        /*
         * ----------------------------------------------------
         * 6. Terminal output
         *
         * Her 1 saniyede bir göster.
         * ----------------------------------------------------
         */

        if (i % 100 == 0)
        {
            float omega_deg_x =
                omega.x * 180.0f / PI;

            float omega_deg_y =
                omega.y * 180.0f / PI;

            float omega_deg_z =
                omega.z * 180.0f / PI;


            printf(
                "t = %5.1f s | "
                "B = [%8.3f %8.3f %8.3f] uT | "
                "m = [%7.4f %7.4f %7.4f] A.m2 | "
                "w = [%7.3f %7.3f %7.3f] deg/s\n",

                time,

                B.x * 1e6f,
                B.y * 1e6f,
                B.z * 1e6f,

                m.x,
                m.y,
                m.z,

                omega_deg_x,
                omega_deg_y,
                omega_deg_z
            );
        }


        /*
         * Zamanı ilerlet
         */

        time += DT;
    }


    /*
     * --------------------------------------------------------
     * FINAL RESULT
     * --------------------------------------------------------
     */

    printf("\n");
    printf("=============================================\n");
    printf("              FINAL RESULT\n");
    printf("=============================================\n");

    printf(
        "Final omega X = %.6f deg/s\n",
        omega.x * 180.0f / PI
    );

    printf(
        "Final omega Y = %.6f deg/s\n",
        omega.y * 180.0f / PI
    );

    printf(
        "Final omega Z = %.6f deg/s\n",
        omega.z * 180.0f / PI
    );

    printf("=============================================\n");


    return 0;
}
