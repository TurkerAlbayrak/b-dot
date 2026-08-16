#include <stdio.h>
#include <math.h>
#include <stdint.h>

/*
 * ============================================================
 *        CUBESAT B-DOT DETUMBLING CONTROLLER
 * ============================================================
 *
 * Amaç:
 *   Manyetometreden gelen Bx, By, Bz ölçümlerini kullanarak
 *   uydunun açısal dönmesini azaltmak.
 *
 * Kontrol kanunu:
 *
 *             m = -K * dB/dt
 *
 * Burada:
 *   B  : Manyetik alan [Tesla]
 *   dB : Manyetik alan değişim hızı [Tesla/s]
 *   m  : Magnetorquer manyetik dipol momenti [A.m^2]
 *   K  : Kontrol kazancı
 *
 * Gerçek sistem:
 *
 * Magnetometer
 *      |
 *      v
 *   Bx By Bz
 *      |
 *      v
 *   Filtering
 *      |
 *      v
 *    B-Dot
 *      |
 *      v
 *   Mx My Mz
 *      |
 *      v
 * Magnetorquer Driver
 *      |
 *      v
 *   Magnetorquer
 *
 * NOT:
 * Bu örnek gerçek donanıma doğrudan bağlanmak yerine
 * kontrol algoritmasının güvenli temelini gösterir.
 * ============================================================
 */


/* ============================================================
 * VECTOR3
 * ============================================================ */

typedef struct
{
    float x;
    float y;
    float z;

} Vector3;


/* ============================================================
 * B-DOT CONTROLLER STATE
 * ============================================================ */

typedef struct
{
    /* Önceki manyetik alan ölçümü [Tesla] */
    Vector3 B_previous;

    /* Kontrol kazancı */
    float K;

    /* Örnekleme zamanı [s] */
    float dt;

    /* Maksimum magnetorquer momenti [A.m^2] */
    float max_moment;

    /* İlk ölçüm alındı mı? */
    uint8_t initialized;

} BDotController;


/* ============================================================
 * VECTOR MAGNITUDE
 * ============================================================ */

static float Vector3_Magnitude(Vector3 v)
{
    return sqrtf(
        v.x * v.x +
        v.y * v.y +
        v.z * v.z
    );
}


/* ============================================================
 * VECTOR LIMIT
 *
 * Magnetorquer'ın izin verilen maksimum momentini aşmasını
 * engeller.
 * ============================================================ */

static Vector3 Vector3_Limit(Vector3 v, float max_value)
{
    Vector3 result = v;

    float magnitude = Vector3_Magnitude(v);

    if (magnitude > max_value && magnitude > 0.0f)
    {
        float scale = max_value / magnitude;

        result.x *= scale;
        result.y *= scale;
        result.z *= scale;
    }

    return result;
}


/* ============================================================
 * B-DOT INITIALIZATION
 * ============================================================ */

void BDot_Init(
    BDotController *controller,
    float K,
    float dt,
    float max_moment)
{
    if (controller == NULL)
        return;

    controller->B_previous.x = 0.0f;
    controller->B_previous.y = 0.0f;
    controller->B_previous.z = 0.0f;

    controller->K = K;

    controller->dt = dt;

    controller->max_moment = max_moment;

    /*
     * İlk ölçüm henüz alınmadı.
     */
    controller->initialized = 0;
}


/* ============================================================
 * B-DOT UPDATE
 *
 * Giriş:
 *
 * B_current = mevcut manyetometre ölçümü [Tesla]
 *
 * Çıkış:
 *
 * magnetic_moment = [A.m^2]
 *
 * İlk çağrıda kontrol çıkışı sıfırdır.
 * Bunun nedeni önceki manyetik alanın bilinmemesidir.
 * ============================================================ */

Vector3 BDot_Update(
    BDotController *controller,
    Vector3 B_current)
{
    Vector3 B_dot;

    Vector3 magnetic_moment;

    /* Başlangıç değeri */
    magnetic_moment.x = 0.0f;
    magnetic_moment.y = 0.0f;
    magnetic_moment.z = 0.0f;


    /* --------------------------------------------------------
     * Güvenlik kontrolü
     * -------------------------------------------------------- */

    if (controller == NULL)
        return magnetic_moment;

    if (controller->dt <= 0.0f)
        return magnetic_moment;


    /* --------------------------------------------------------
     * İLK ÖLÇÜM
     *
     * İlk ölçümde dB/dt hesaplamıyoruz.
     *
     * Çünkü:
     *
     * B_previous = 0
     *
     * kabul etmek yanlış ve çok büyük bir kontrol çıktısı
     * oluşturabilir.
     * -------------------------------------------------------- */

    if (controller->initialized == 0)
    {
        controller->B_previous = B_current;

        controller->initialized = 1;

        return magnetic_moment;
    }


    /* --------------------------------------------------------
     * dB/dt hesapla
     *
     * dB/dt = (B_current - B_previous) / dt
     * -------------------------------------------------------- */

    B_dot.x =
        (B_current.x - controller->B_previous.x)
        / controller->dt;

    B_dot.y =
        (B_current.y - controller->B_previous.y)
        / controller->dt;

    B_dot.z =
        (B_current.z - controller->B_previous.z)
        / controller->dt;


    /* --------------------------------------------------------
     * B-DOT CONTROL LAW
     *
     * m = -K * dB/dt
     * -------------------------------------------------------- */

    magnetic_moment.x =
        -controller->K * B_dot.x;

    magnetic_moment.y =
        -controller->K * B_dot.y;

    magnetic_moment.z =
        -controller->K * B_dot.z;


    /* --------------------------------------------------------
     * MOMENT LIMIT
     *
     * Magnetorquer fiziksel sınırını aşmasını engelle.
     * -------------------------------------------------------- */

    magnetic_moment =
        Vector3_Limit(
            magnetic_moment,
            controller->max_moment
        );


    /* --------------------------------------------------------
     * Mevcut ölçümü bir sonraki çevrim için kaydet.
     * -------------------------------------------------------- */

    controller->B_previous = B_current;


    return magnetic_moment;
}


/* ============================================================
 * TESLA UNIT CONVERSION
 *
 * Manyetometreler çoğu zaman:
 *
 *     microTesla [uT]
 *
 * döndürür.
 *
 * Kontrol algoritmasında:
 *
 *     Tesla [T]
 *
 * kullanıyoruz.
 * ============================================================ */

Vector3 MicroTesla_To_Tesla(Vector3 B_uT)
{
    Vector3 B_T;

    B_T.x = B_uT.x * 1.0e-6f;
    B_T.y = B_uT.y * 1.0e-6f;
    B_T.z = B_uT.z * 1.0e-6f;

    return B_T;
}


/* ============================================================
 * EXAMPLE MAIN
 *
 * Gerçek STM32 projesinde main() yerine bu fonksiyonlar
 * sensor driver + timer callback içerisinden çağrılabilir.
 * ============================================================ */

int main(void)
{
    BDotController controller;

    /*
     * --------------------------------------------------------
     * Sistem parametreleri
     * --------------------------------------------------------
     *
     * dt = 0.01 s
     * => 100 Hz kontrol döngüsü
     *
     * max_moment:
     * Magnetorquer'ın izin verilen maksimum momenti.
     *
     * Gerçek CubeSat'ta bu değer donanım tasarımından gelir.
     * --------------------------------------------------------
     */

    BDot_Init(
        &controller,
        100000.0f,      /* K */
        0.01f,          /* dt */
        0.2f            /* max moment [A.m^2] */
    );


    /* --------------------------------------------------------
     * ÖRNEK MANYETOMETRE ÖLÇÜMÜ
     *
     * Gerçek sensör örneğin:
     *
     * Bx = 20.5 uT
     * By = -5.2 uT
     * Bz = 42.8 uT
     * -------------------------------------------------------- */

    Vector3 B_uT =
    {
        20.5f,
        -5.2f,
        42.8f
    };


    /* uT -> Tesla */

    Vector3 B =
        MicroTesla_To_Tesla(B_uT);


    /* --------------------------------------------------------
     * B-DOT
     * -------------------------------------------------------- */

    Vector3 m =
        BDot_Update(
            &controller,
            B
        );


    printf("\nB-DOT OUTPUT\n");

    printf("--------------------------\n");

    printf("Bx = %.9f T\n", B.x);
    printf("By = %.9f T\n", B.y);
    printf("Bz = %.9f T\n", B.z);

    printf("\n");

    printf("Mx = %.9f A.m^2\n", m.x);
    printf("My = %.9f A.m^2\n", m.y);
    printf("Mz = %.9f A.m^2\n", m.z);


    return 0;
}
