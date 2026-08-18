# B-Dot Algoritması (Türkçe Açıklamalı)

Bu README, uydu/CubeSat **detumbling (donme sönümleme)** problemlerinde
en yaygın kullanılan kontrol algoritması olan **B-dot**'u sıfırdan,
formülleri ve çalışan kod örnekleriyle anlatır. Amaç; bir uydu
fırlatıldıktan sonra roketten ayrılırken kazandığı istenmeyen açısal
hızı (dönmeyi), yalnızca magnetometre ve magnetorquer (manyeto-tork
bobinleri) kullanarak, dışarıdan hiçbir aktif tahrik gerektirmeden
söndürmektir.

> **Tek cümlede B-dot:** Manyetik alanın ne kadar hızlı değiştiğini
> ($\dot{\vec{B}}$) ölç, tam tersi yönde bir manyetik dipol
> ($\vec{m}_{cmd} = -k\dot{\vec{B}}$) üret; bu, uydunun dönüşünü
> otomatik olarak frenler — çünkü değişimin kendisi dönüşün belirtisidir.

Bu depo, [`pwm-ornekleri`](../pwm-ornekleri) ve [`triad-ornekleri`](../triad-ornekleri)
ile aynı ADCS (Attitude Determination and Control System) ailesindendir:
B-dot uydunun **hızlı dönüşünü söndürür** (detumbling), TRIAD ise
sönümleme sonrası uydunun **tam yönelimini belirler** (attitude
determination).

---

## İçindekiler

1. [B-Dot Nedir, Neden Kullanılır?](#b-dot-nedir-neden-kullanilir)
2. [Fiziksel Arka Plan](#fiziksel-arka-plan)
3. [Kontrol Yasası](#kontrol-yasasi)
4. [Algoritma Akışı](#algoritma-akisi)
5. [Kod Örnekleri](#kod-ornekleri)
   - [1. Temel dB/dt Hesabı](#1-temel-dbdt-hesabi)
   - [2. Gürültü Filtreli B-Dot Tahmini](#2-gurultu-filtreli-b-dot-tahmini)
   - [3. Klasik (Orantısal) B-Dot Kontrolcü](#3-klasik-orantisal-b-dot-kontrolcu)
   - [4. Bang-Bang B-Dot Varyantı](#4-bang-bang-b-dot-varyanti)
   - [5. Akımdan (mA) Yön + PWM'e Dönüşüm](#5-akimdan-ma-yon--pwme-donusum)
   - [6. Tam Kontrol Döngüsü (Uçtan Uca)](#6-tam-kontrol-dongusu-uctan-uca)
6. [Kazanç (k) Seçimi](#kazanc-k-secimi)
7. [Sayısal Örnek (Uçtan Uca)](#sayisal-ornek-uctan-uca)
8. [Orantısal vs. Bang-Bang Karşılaştırması](#orantisal-vs-bang-bang-karsilastirmasi)
9. [Ölçüm-Aktüasyon Çakışması (Kritik Pratik Sorun)](#olcum-aktuasyon-cakismasi-kritik-pratik-sorun)
10. [Sönümleme Süresinin Kabaca Tahmini](#sonumleme-suresinin-kabaca-tahmini)
11. [Sınırlamalar ve Ne Zaman Yetersiz Kalır](#sinirlamalar-ve-ne-zaman-yetersiz-kalir)
12. [Sık Yapılan Hatalar](#sik-yapilan-hatalar)
13. [Referanslar](#referanslar)

---

## B-Dot Nedir, Neden Kullanılır?

Bir uydu roketten ayrıldığında genellikle her eksende saniyede birkaç
dereceden onlarca dereceye kadar istenmeyen bir açısal hıza (tumbling)
sahiptir. Bu durumda:

- Güneş panelleri Güneş'e sabit bakamaz -> güç üretimi düşer,
- Yıldız izleyici / kamera gibi hassas sensörler kullanılamaz,
- Haberleşme anteni yer istasyonuna sabit bakamaz.

**B-dot**, bu ilk hızlı dönüşü söndürmek için tasarlanmış, son derece
basit, düşük güç tüketen ve **sadece bir magnetometre + magnetorquer
bobinleri** ile çalışan bir kontrol algoritmasıdır. Yıldız izleyici,
jiroskop veya karmaşık bir yönelim tahmini GEREKTİRMEZ — bu da onu
CubeSat'lerin neredeyse tamamında "ilk hat" savunma / kurtarma modu
olarak vazgeçilmez kılar.

## Fiziksel Arka Plan

Bir uydu Dünya'nın manyetik alanı içinde dönerken, gövdesine sabit
magnetometre, zamanla değişen bir manyetik alan vektörü $\vec{B}(t)$
ölçer. Bu değişimin **iki kaynağı** vardır:

1. Uydunun yörünge boyunca hareketi (yavaş değişim),
2. Uydunun kendi ekseni etrafında **dönmesi** (hızlı değişim).

Eğer uydu hızlı dönüyorsa, $\vec{B}(t)$'nin zamana göre türevi
($\dot{\vec{B}}$) büyük ölçüde bu dönme hızından kaynaklanır:

$$\dot{\vec{B}} \approx -\vec{\omega} \times \vec{B} \quad (\text{yaklaşık ilişki, yörünge etkisi ihmal edildiğinde})$$

burada $\vec{\omega}$ uydunun açısal hız vektörüdür. B-dot algoritması
tam olarak bu ilişkiyi kullanarak, **açısal hızı doğrudan ölçmeden**,
sadece $\dot{\vec{B}}$'yi izleyerek dönüşü sönümleyecek bir manyetik
tork üretir.

## Kontrol Yasası

Magnetorquer bobinlerinden bir dipol moment $\vec{m}$ geçirildiğinde,
ortamdaki manyetik alanla etkileşerek bir tork üretilir:

$$\vec{\tau} = \vec{m} \times \vec{B}$$

Klasik B-dot kontrol yasası, bu dipol momenti şöyle komutlar:

$$\boxed{\vec{m}_{cmd} = -k \, \dot{\vec{B}}}$$

- $k > 0$: kontrolör kazancı (tasarım parametresi),
- Negatif işaret, üretilen torkun açısal hızı **her zaman sönümleyecek**
  yönde olmasını sağlar (enerjiyi sistemden çeker, asla eklemez —
  bu yüzden B-dot doğal olarak kararlıdır / Lyapunov anlamında
  "pasif sönümleyici" bir kontrolcüdür).

$\dot{\vec{B}}$ pratikte ardışık iki magnetometre örneğinden geri fark
(backward difference) ile tahmin edilir:

$$\dot{\vec{B}}(t) \approx \frac{\vec{B}(t) - \vec{B}(t-\Delta t)}{\Delta t}$$

### Neden Bu İşe Yarıyor? (Sezgisel/Enerji Bakışı)

B-dot'un "neden kararlı" olduğunu görmek için sistemin dönme kinetik
enerjisini düşünelim: $E = \tfrac{1}{2}\vec{\omega}^T J \vec{\omega}$.
$\dot{\vec{B}} \approx -\vec{\omega}\times\vec{B}$ yaklaşık ilişkisini
kullanarak kontrol yasasını yazarsak:

$$\vec{m}_{cmd} = -k\dot{\vec{B}} \approx k\,(\vec{\omega}\times\vec{B})$$

Üretilen tork $\vec{\tau} = \vec{m}_{cmd}\times\vec{B}$ olduğundan, bu
torkun açısal hızla iç çarpımı (gücü) **her zaman negatif veya sıfırdır**
— yani B-dot, sistemden sürekli enerji **çeker**, asla eklemez. Bu, onu
ek bir kararlılık analizi gerektirmeyen, doğası gereği kararlı
(pasif) bir kontrolcü yapar; bu özelliği CubeSat'lerde tercih
edilmesinin en önemli sebeplerinden biridir.

## Algoritma Akışı

```
   Magnetometre olcumu B(t)
            |
            v
   dB/dt = (B(t) - B(t-dt)) / dt      <- turev tahmini (gurultu icerir!)
            |
            v
   (istege bagli) alcak geciren filtre uygula
            |
            v
   m_cmd = -k * dB/dt                  <- kontrol yasasi
            |
            v
   Isaretli akim: I = m_cmd / (N*A)    <- bobin akimina cevir
            |
            v
   yon = sign(I)   |I|_clamp = min(|I|, Imax)
            |
            v
   duty = (|I|_clamp * R_bobin / V_kaynak) * 100
            |
            v
   H-kopru: YON pini + PWM pini -> Magnetorquer bobinleri
            |
            v
   tork = m x B  -> acisal hiz sonumlenir -> (dongu tekrar eder)
```

Bu döngü genelde **1-10 Hz** arası sabit bir periyotta (timer interrupt
veya RTOS task) sürekli çalıştırılır.

## Kod Örnekleri

### 1. Temel dB/dt Hesabı

```c
typedef struct { float x, y, z; } Vec3;

/**
 * Iki ardisik magnetometre olcumunden turev (dB/dt) tahmini yapar.
 * @param B_simdi  Su anki olcum [Tesla]
 * @param B_onceki Bir onceki olcum [Tesla]
 * @param dt       Iki olcum arasindaki sure [saniye]
 */
Vec3 bdot_hesapla(Vec3 B_simdi, Vec3 B_onceki, float dt)
{
    Vec3 dBdt;
    dBdt.x = (B_simdi.x - B_onceki.x) / dt;
    dBdt.y = (B_simdi.y - B_onceki.y) / dt;
    dBdt.z = (B_simdi.z - B_onceki.z) / dt;
    return dBdt;
}
```

### 2. Gürültü Filtreli B-Dot Tahmini

Ham geri-fark türevi, magnetometre gürültüsünü **büyütme** eğilimindedir
(türev alma yüksek frekansları güçlendirir). Pratikte bir düşük geçiren
filtre (basit üstel hareketli ortalama - EMA) uygulamak, kontrolcünün
gürültüye tepki verip gereksiz yere enerji harcamasını önler:

```c
typedef struct {
    Vec3 dBdt_filtrelenmis;
    Vec3 B_onceki;
    int  ilk_calisma;
} BdotFiltre;

/**
 * alfa: filtre katsayisi (0-1 arasi). Kucuk alfa = daha guclu
 * filtreleme (daha yavas tepki), buyuk alfa = daha az filtreleme.
 * Tipik baslangic degeri: alfa = 0.2 - 0.4
 */
Vec3 bdot_filtrele(BdotFiltre *f, Vec3 B_simdi, float dt, float alfa)
{
    if (f->ilk_calisma) {
        f->B_onceki = B_simdi;
        f->dBdt_filtrelenmis = (Vec3){0, 0, 0};
        f->ilk_calisma = 0;
        return f->dBdt_filtrelenmis;
    }

    Vec3 dBdt_ham;
    dBdt_ham.x = (B_simdi.x - f->B_onceki.x) / dt;
    dBdt_ham.y = (B_simdi.y - f->B_onceki.y) / dt;
    dBdt_ham.z = (B_simdi.z - f->B_onceki.z) / dt;

    // EMA: yeni = alfa*ham + (1-alfa)*eski
    f->dBdt_filtrelenmis.x = alfa * dBdt_ham.x + (1 - alfa) * f->dBdt_filtrelenmis.x;
    f->dBdt_filtrelenmis.y = alfa * dBdt_ham.y + (1 - alfa) * f->dBdt_filtrelenmis.y;
    f->dBdt_filtrelenmis.z = alfa * dBdt_ham.z + (1 - alfa) * f->dBdt_filtrelenmis.z;

    f->B_onceki = B_simdi;
    return f->dBdt_filtrelenmis;
}
```

### 3. Klasik (Orantısal) B-Dot Kontrolcü

En yaygın ve en "yumuşak" varyant — dipol moment, $\dot{\vec{B}}$'nin
büyüklüğüyle **orantılıdır**:

```c
#define BDOT_KAZANC_K  50000.0f  // Tasarima gore ayarlanir (bkz. asagidaki bolum)

Vec3 dipol_momenti_hesapla(Vec3 dBdt, float k)
{
    Vec3 m;
    m.x = -k * dBdt.x;
    m.y = -k * dBdt.y;
    m.z = -k * dBdt.z;
    return m;
}
```

### 4. Bang-Bang B-Dot Varyantı

Bazı gerçek uçuş yazılımlarında (özellikle çok basit/düşük güçlü
mikrodenetleyicilerde), dipol moment orantısal yerine yalnızca
**işarete göre** sabit maksimum değerde komutlanır. Bu, doygunlaşmış
(saturated) bir bang-bang kontrolcüdür; daha hızlı sönümler ama daha
"sert" ve enerji-verimsizdir:

```c
Vec3 dipol_momenti_bangbang(Vec3 dBdt, float m_max)
{
    Vec3 m;
    m.x = (dBdt.x > 0) ? -m_max : (dBdt.x < 0 ? m_max : 0.0f);
    m.y = (dBdt.y > 0) ? -m_max : (dBdt.y < 0 ? m_max : 0.0f);
    m.z = (dBdt.z > 0) ? -m_max : (dBdt.z < 0 ? m_max : 0.0f);
    return m;
}
```

### 5. Akımdan (mA) Yön + PWM'e Dönüşüm

Dipol moment komutu fiziksel olarak bobin akımına, akım da PWM'e
çevrilmelidir. Bu, **işaretli bir değeri** (pozitif/negatif akım) PWM
donanımının anlayacağı **yön biti + pozitif duty cycle** çiftine ayıran
kritik adımdır:

```c
#define BOBIN_SARIM_SAYISI   200.0f   // N (turns)
#define BOBIN_ALANI_M2       0.008f   // A [m^2]
#define BOBIN_DIRENC_OHM     110.0f   // R_bobin [ohm]
#define SURUCU_KAYNAK_V      5.0f     // V_kaynak (H-kopru besleme gerilimi)
#define MAKS_BOBIN_AKIM_mA   50.0f    // Guvenlik siniri (clamp)
#define PWM_COZUNURLUK       255      // 8-bit PWM

typedef struct { uint8_t duty; uint8_t yon; } PwmKomut;

/**
 * m [A*m^2] -> I [mA] (isaretli) -> yon + PWM duty
 */
PwmKomut dipol_to_pwm(float m_tek_eksen)
{
    // 1) Dipol momentinden isaretli akima gec
    float I_mA = (m_tek_eksen / (BOBIN_SARIM_SAYISI * BOBIN_ALANI_M2)) * 1000.0f;

    // 2) Yonu isaretten cikar
    PwmKomut komut;
    komut.yon = (I_mA >= 0.0f) ? 1 : 0;   // 1 = ileri, 0 = geri

    // 3) Buyuklugu al ve guvenlik sinirina gore clamp'le
    float akim_mA = fabsf(I_mA);
    if (akim_mA > MAKS_BOBIN_AKIM_mA) akim_mA = MAKS_BOBIN_AKIM_mA;

    // 4) Akim -> Gerilim (Om Kanunu) -> Duty Cycle
    float gerilim = (akim_mA / 1000.0f) * BOBIN_DIRENC_OHM;
    float duty_yuzde = (gerilim / SURUCU_KAYNAK_V) * 100.0f;
    if (duty_yuzde > 100.0f) duty_yuzde = 100.0f;

    komut.duty = (uint8_t)((duty_yuzde / 100.0f) * PWM_COZUNURLUK);
    return komut;
}
```

> Bu dönüşümün tam matematiksel türetimi ve X/Y/Z 3 eksenli bir
> çalıştırma örneği için `pwm-ornekleri` deposundaki
> `examples/08_bdot_magnetorquer` klasörüne bakabilirsiniz.

### 6. Tam Kontrol Döngüsü (Uçtan Uca)

```c
BdotFiltre filtre = { .ilk_calisma = 1 };

void bdot_kontrol_dongusu(void)  // Ornegin 5 Hz'de (dt=0.2s) cagrilir
{
    const float dt = 0.2f;

    Vec3 B_simdi = magnetometre_oku();              // donanima ozel fonksiyon
    Vec3 dBdt = bdot_filtrele(&filtre, B_simdi, dt, 0.3f);

    Vec3 m_cmd = dipol_momenti_hesapla(dBdt, BDOT_KAZANC_K);

    PwmKomut kx = dipol_to_pwm(m_cmd.x);
    PwmKomut ky = dipol_to_pwm(m_cmd.y);
    PwmKomut kz = dipol_to_pwm(m_cmd.z);

    hw_set_yon(0, kx.yon);  hw_set_pwm_duty(0, kx.duty);
    hw_set_yon(1, ky.yon);  hw_set_pwm_duty(1, ky.duty);
    hw_set_yon(2, kz.yon);  hw_set_pwm_duty(2, kz.duty);
}
```

## Kazanç (k) Seçimi

$k$ kazancı çok küçük seçilirse sönümleme çok yavaş olur (dönüş
söndürülmeden pil/güç bütçesi tükenebilir); çok büyük seçilirse
kontrolcü gürültüye aşırı tepki verir ve bobinleri gereksiz yere
doygunluğa (saturasyona) sürükler. Literatürde sık kullanılan bir
başlangıç formülü:

$$k = 2 \, \omega_0 \, (1 + \sin\theta_{min}) \, J_{min}$$

burada $\omega_0$ yörünge açısal hızı, $\theta_{min}$ uydunun eylemsizlik
eksenleri ile manyetik alan arasındaki minimum açı, $J_{min}$ ise en
küçük eylemsizlik momentidir. Pratikte çoğu ekip bu formülü **başlangıç
noktası** olarak kullanır ve simülasyonla (gerçek yörünge, gerçek
eylemsizlik matrisi, gerçek sensör gürültüsüyle) ince ayar yapar.

## Sayısal Örnek (Uçtan Uca)

Formüllerin somutlaşması için, iki ardışık magnetometre örneğinden
başlayarak PWM komutuna kadar tam bir hesap zinciri:

**Girdi:** $B(t) = (19.4, -15.9, 36.1)\ \mu T$, $B(t-\Delta t) = (20.0, -15.0, 35.0)\ \mu T$, $\Delta t = 0.5\ s$, $k = 50000$.

| Adım | Formül | Sonuç (X ekseni) |
|---|---|---|
| 1. Türev | $\dot{B}_x = \Delta B_x/\Delta t$ | $(19.4-20.0)\text{e-6}/0.5 = -1.2\times10^{-6}\ T/s$ |
| 2. Dipol moment | $m_x = -k\dot{B}_x$ | $-50000\times(-1.2\text{e-}6) = 0.060\ A{\cdot}m^2$ |
| 3. Akım | $I_x = m_x/(N{\cdot}A) = m_x/(200\times0.008)$ | $37.5\ mA$ |
| 4. Yön | $\text{sign}(I_x)$ | pozitif $\rightarrow$ **ileri** |
| 5. Clamp | $\min(37.5,\ I_{max}{=}50)$ | $37.5\ mA$ (sınıra takılmadı) |
| 6. Gerilim | $V = I\cdot R_{bobin} = 0.0375\times110$ | $4.125\ V$ |
| 7. Duty | $D = (V/V_{kaynak})\times100 = (4.125/5)\times100$ | $\%82.5$ |
| 8. PWM (8-bit) | $D/100\times255$ | $\approx 210 / 255$ |

Aynı hesap Y ve Z eksenleri için tekrarlanır ve üç eksen **aynı anda,
bağımsız olarak** sürülür. Bu tablo, `pwm-ornekleri` deposundaki
`examples/08_bdot_magnetorquer/bdot_magnetorquer.c` dosyasının gerçek
çalıştırma çıktısıyla birebir örtüşür.

## Orantısal vs. Bang-Bang Karşılaştırması

| Özellik | Orantısal ($m=-k\dot{B}$) | Bang-Bang ($m=\mp m_{max}\cdot\text{sign}(\dot{B})$) |
|---|---|---|
| Sönümleme hızı | Daha yavaş, yumuşak | Daha hızlı (özellikle başlangıçta) |
| Enerji tüketimi | Daha verimli | Daha müsrif (sürekli maksimum akım) |
| Gürültüye tepki | Orantılı, ölçülü | Küçük gürültüde bile yön sıçraması yapabilir |
| Sönümleme sonu davranışı | Yumuşak yaklaşır, "titremez" | Sıfıra yakınken küçük bir sınır çevriminde (limit cycle) salınabilir |
| Uygulama karmaşıklığı | Çarpma + ölçekleme | Yalnızca işaret kontrolü (çok basit MCU'larda tercih edilir) |
| Tipik kullanım | Ana/nominal detumbling modu | Acil/ilk kurtarma modu, çok yüksek başlangıç $\omega$ |

Pratikte birçok uçuş yazılımı **hibrit** bir yaklaşım kullanır: açısal
hız yüksekken (büyük $|\dot{B}|$) bang-bang benzeri doygun davranış,
düşükken orantısal/yumuşak davranış sergileyen bir **doygunluklu
orantısal kontrolcü** (saturating proportional controller) — ki bu
zaten Bölüm 5'teki `clamp` adımıyla doğal olarak ortaya çıkar.

## Ölçüm-Aktüasyon Çakışması (Kritik Pratik Sorun)

Bu, ilk kez B-dot uygulayan ekiplerin sıkça gözden kaçırdığı ama
**gerçek donanımda algoritmayı tamamen bozabilecek** bir sorundur:

Magnetorquer bobinlerinden akım geçerken, bu bobinler **kendi
manyetik alanlarını** üretirler. Eğer magnetometre bu sırada (veya
hemen ardından, bobinin artık alanı sönmeden) okuma yaparsa, ölçtüğü
şey Dünya'nın gerçek alanı değil, **kendi aktüatörünün ürettiği
alanla kirlenmiş** bir değerdir. Bu durum kontrolcüyü yanlış yönde
besleyebilir ve sönümleme yerine osilasyona (hatta ıraksamaya) yol
açabilir.

**Standart çözüm — Ölç/Sür (sense/actuate) zaman bölmesi:**

```
|<---- Olcum penceresi ---->|<---- Aktuasyon penceresi ---->|
      (bobinler KAPALI)              (bobinler ACIK)
      magnetometre oku                 PWM uygula
      dB/dt hesapla                    (magnetometre okunmaz)
                                              |
                                              v
                                   bir sonraki donguye gec
```

Tipik bir uygulamada periyodun `%20-40`'ı ölçüme, kalanı aktüasyona
ayrılır (örneğin 1 saniyelik döngüde 200-400 ms ölçüm + 600-800 ms
sürme). Bazı gelişmiş tasarımlar, bobinlerin bilinen alanını
modelleyip ölçümden **çıkararak** (bias compensation) bu bölmeyi
gerektirmez, ancak bu ek kalibrasyon karmaşıklığı getirir. Basit
CubeSat projeleri için zaman bölmesi hem daha güvenilir hem daha
kolay uygulanabilirdir.

## Sönümleme Süresinin Kabaca Tahmini

Açısal hızın üstel olarak azaldığı basitleştirilmiş bir model
varsayarsak (sabit ortalama $|B|$ ve tek eksenli yaklaşık analiz ile):

$$\omega(t) \approx \omega_0\, e^{-t/\tau}, \qquad \tau \approx \frac{J}{k\,\bar{B}^2}$$

burada $J$ ilgili eksendeki eylemsizlik momenti, $\bar{B}$ yörünge
üzerindeki ortalama manyetik alan büyüklüğüdür. Bu formül **kaba bir
mertebe tahmini** içindir; gerçek sönümleme süresi yörünge eğimi,
başlangıç $\omega_0$, eylemsizlik matrisinin tam yapısı ve $k$
seçimine bağlı olarak belirgin şekilde değişir — kesin süre için
sayısal simülasyon (gerçek yörünge propagasyonu + IGRF model)
şarttır. Tipik bir 3U CubeSat için, birkaç derece/saniyelik başlangıç
hızından sönümlemeye kadar geçen süre genelde **birkaç saatten bir
kaç güne** kadar sürebilir.

## Sınırlamalar ve Ne Zaman Yetersiz Kalır

- **Manyetik alanla aynı hizadaki dönüş ekseni sönümlenemez**: Tork
  $\vec{m} \times \vec{B}$ olduğundan, açısal hız vektörü tam olarak
  $\vec{B}$ ile aynı yöndeyse üretilebilecek tork sıfıra yaklaşır. Bu
  durum yörünge boyunca $\vec{B}$ yönü sürekli değiştiği için genelde
  kendiliğinden çözülür, ancak sönümleme süresini uzatır.
  
- **Çok yüksek başlangıç açısal hızlarında** (fırlatma anomalisi vb.)
  B-dot yavaş kalabilir; bu durumlarda bazı tasarımlar reaksiyon
  tekerleği veya daha güçlü aktüatörlerle desteklenir.

- **B-dot yalnızca dönüşü söndürür, hassas yönelim sağlamaz.** Sönümleme
  tamamlandıktan sonra hassas işaretleme için TRIAD/QUEST gibi bir
  yönelim belirleme algoritmasına ve ince kontrol (PD/kuaterniyon
  kontrolcü) aşamasına geçilmesi gerekir.

## Sık Yapılan Hatalar

- **Ham türevi filtrelemeden kullanmak**: Geri-fark türevi gürültüyü
  büyütür; en azından basit bir EMA filtresi uygulamak önerilir.
- **Akım/duty sınırını (clamp) atlamak**: Hesaplanan dipol moment
  komutu her zaman bobinin/sürücünün maksimum sürekli akım değeriyle
  sınırlandırılmalıdır, aksi halde donanım termal hasar görebilir.
- **İşaretli değeri doğrudan PWM'e vermek**: PWM donanımı negatif değer
  kabul etmez; işaret (yön) ve büyüklük (duty) mutlaka ayrıştırılmalıdır.
- **Kontrol döngüsünü değişken periyotta çalıştırmak**: $dt$ değeri
  formüllerde doğrudan kullanıldığından, döngü sabit bir periyotta
  (timer interrupt/RTOS task) çalıştırılmalı ve gerçek $dt$ ölçülüp
  formüle verilmelidir.
- **Manyetometreyi kalibre etmeden kullanmak**: Bias/scale/soft-iron
  hataları düzeltilmezse, $\dot{\vec{B}}$ tahmini gerçek dönüşten değil
  sensör hatasından kaynaklanan sahte bir sinyal içerebilir.

## Referanslar

- Wisniewski, R. (2000). *Satellite Attitude Control Using Only
  Electromagnetic Actuation*.
- Stickler, A. C., & Alfriend, K. T. (1976). *Elementary Magnetic
  Attitude Control System*. Journal of Spacecraft and Rockets.
- Wertz, J. R. (Ed.). (1978). *Spacecraft Attitude Determination and
  Control*. Kluwer Academic Publishers. (B-dot ve genel ADCS teorisi
  için klasik referans kaynak.)
