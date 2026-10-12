# Kontrolcü tasarımı — teorik altyapı (ortak çalışma için)

> **Bu dosya ne:** İniş kontrolcüsü (klasik yöntemler) ve pekiştirmeli öğrenme
> (RL) çalışmalarının üstüne inşa edileceği ortak teorik zemin.
> - **Durum:** taslak, 2026-10-12. Repo koduyla çapraz kontrol edilmeli
>   (satır numaraları, parametre adları).
> - **Okuyucu:** Sefa (klasik kontrolcüler), Arda (RL).
> - **Cevapladığı üç soru:**
>   1. Simülasyondan ölçüm (sistem tanımlama) dışında kontrolcü tasarlamanın yolu var mı?
>   2. Segmentasyon kamerası kontrolcüye tam olarak nerede bağlanıyor?
>   3. Bugünkü P katsayılı yasa mantıklı mı?

---

## 1. Katmanlar: kamera nerede bağlanıyor

```
Kamera ──► Segmentasyon ──► Öznitelikler ──► REHBERLİK ──► TAKİP ──► PX4 iç döngü ──► Araç
 (RGB)      (maske)         ρ, (u,v),        (dış döngü)   (hız       (hız → açı →
                            view_bounded,    "nereye,      referansını  motor)
                            D tahmini        ne hızla?"    izle)
                                  ▲                                     ▲
                            EKF: h, vz, vxy                     DOKUNMUYORUZ
```

- **Algı:** maske → öznitelikler. Bugün kullanılanlar: site alan oranı `ρ`
  (sitenin kadrajda kapladığı oran), `view_bounded` (site kadraja sığıyor mu),
  site merkezi.
- **Rehberlik (dış döngü):** öznitelikler + EKF → hız referansı
  `(vx, vy, vz)_ref`. **Kamera kontrolcüye burada bağlanır.** "P katsayılı
  denklem" bu katmanda.
- **Takip:** hız referansını izleyen kontrolcü. Bugün dikeyde PI + ileri
  besleme. PID / LQI / MPC / Fuzzy karşılaştırması bu katmanda.
- **PX4 iç döngü:** hız → açı → motor. Değiştirmiyoruz; bizim için "tesis".
- **Koddaki yeri (bugün):** `emergency_landing_mode.hpp` içinde `onCandidate`
  (ρ ve `view_bounded`'ı günceller) ve `descentSpeed` (alan yasası ile irtifa
  yasası arasında seçim yapar). Tur 2 raporuna göre satır ~593–623; güncel
  koddan teyit edilmeli.

**Sonuç:** RL ile klasik kontrolcülerin ortak noktası, **öznitelik → hız
referansı** arayüzü. İkisi de aynı gözlemi alıp aynı eylemi üretirse aynı
tesiste, aynı senaryolarla adil karşılaştırılır (§5).

---

## 2. Kontrolcü tasarlamanın yolları

Bizim yolumuz (simülasyondan ölçüp model çıkarmak) tek yol değil. Ana aileler:

| Yol | Neye ihtiyaç duyar | Artısı | Eksisi | Bizde |
|---|---|---|---|---|
| **a) Sistem tanımlama + model tabanlı tasarım** | Ölçülmüş giriş/çıkış verisi (K5, K5-xy) | Model elde olunca her yöntem (PID, LQI, MPC, H∞) üstüne kurulur; hızlı Monte Carlo | Model ancak ölçülen koşulları kapsar | **Ana yol. Yapıldı:** z ve x,y tesisi doğrulandı |
| **b) İlk prensipler (fizik modeli)** | Kütle, itki, PX4 kaskad yapısı ve parametreleri | Veri gerektirmez, fiziksel anlamı açık | PX4'ün iç ayrıntıları ve gecikmeler tam bilinmez; doğrulamak için yine veri gerekir | Gerek yok; (a) zaten PX4'ün tamamını kara kutu olarak ölçüyor |
| **c) Doğrudan veri güdümlü (model çıkarmadan)** | Giriş/çıkış verisi | Model adımını atlar | Veri kalitesine çok duyarlı | İsteğe bağlı karşılaştırma (aşağıda) |
| **d) Benzetim içinde kara kutu optimizasyon** | Bir simülatör + maliyet fonksiyonu | Kontrolcü yapısından bağımsız, gerçek görevi doğrudan optimize eder | Çok sayıda koşu; aşırı uyma riski | **Ayar yöntemimiz** (surrogateopt, eşit bütçe) |
| **e) Öğrenme (RL, taklit)** | Simülatör veya kayıtlı veri | Doğrusal olmayan, karmaşık politikalar | Kararlılık garantisi yok; çok veri | Arda (RL). Taklit için K2 "uzman" verisi hazır |

**(c) hakkında iki somut yöntem:**
- **VRFT (Virtual Reference Feedback Tuning):** Tek bir veri kümesinden,
  model çıkarmadan, istenen kapalı çevrim davranışını taklit eden kontrolcüyü
  doğrudan seçer. Tek seferlik bir optimizasyon; PID ayarı için kullanılmış.
  Zayıf yanı: kapalı çevrim kararlılığını kendiliğinden garanti etmez.
- **DeePC (Data-enabled Predictive Control):** MPC'nin veri güdümlü eşi.
  Model yerine geçmiş giriş/çıkış verisini kullanarak geleceği tahmin eder.
  Gerçek nano-kuadkopterlerde konum kontrolünde denendi ve fizik modelli
  MPC ile karşılaştırıldı; PX4 SITL üzerinde de çalışmalar var.
- **Değeri:** Tezde "model tabanlı MPC ile veri güdümlü DeePC" karşılaştırması
  ilginç bir ek olur. Ana planı değiştirmeden sona eklenebilir.

**Hüküm:** (a) doğru temel. Elde ettiğimiz model yalnız klasik kontrolcüler
için değil; RL için de hızlı eğitim ortamı ve ayar için (d) simülatörü olarak
kullanılır. Yani temel ortak.

---

## 3. Kamerayı kontrolcüye bağlamanın yolları (görsel servo)

Literatürde iki ana yaklaşım var, bir de biyolojiden gelen üçüncüsü:

### 3.1 Konum tabanlı (PBVS)
Kameradan 3B konum/yükseklik **tahmin edilir**, kontrol bu tahmin üstünden
yapılır. Bugünkü sistemin yatayı (site konumu → `goto`) ve irtifa yasası
(EKF yüksekliği) bu sınıfta.
- **Artı:** Klasik kontrol doğrudan uygulanır.
- **Eksi:** Tahmin hatası doğrudan kontrol hatasına dönüşür. W5 bunun
  örneği: EKF yüksekliği ~4 m yanlış, temas 1.47 m/s.

### 3.2 Görüntü tabanlı (IBVS)
Kontrol doğrudan **görüntüdeki hata** üstünden yapılır; 3B tahmin yok.
- **Bizim öznitelikler:** site merkezinin kadraj merkezinden piksel sapması
  → `vx, vy`; alan oranı `ρ` → `vz`.
- **Kanonik yasa:** `v = −λ · L⁺ · e` (e: görüntü hatası, L: etkileşim
  matrisi). Yani IBVS'nin ders kitabı yasası **orantılı (P) bir yasa**.
- **Görüntü momentleri:** Alan ve ağırlık merkezi gibi moment öznitelikleri
  eksenleri birbirinden ayırır; bu yüzden döner kanatlı İHA iniş
  çalışmalarında yaygın.
- **Kadraj kısıtı:** Hedefi kadrajda tutmak IBVS iniş çalışmalarında açık
  bir tasarım hedefi; bariyer fonksiyonlu kontrolcülerle Gazebo'da
  çalışılmış. Bizim süzülme konisindeki kadraj kısıtıyla aynı problem.

### 3.3 Biyolojik: ıraksama ve tau
- **Iraksama:** `D = v/h` (alçalma hızı / yükseklik). Arılar dikey inişte
  bunu sabit tutar; temasta hız kendiliğinden sıfıra yaklaşır.
- **Tau (temasa kalan süre):** `τ = h/v = 1/D`. Tau teorisi, birçok canlının
  nesnelere bu değişkenle yaklaştığını öne sürüyor; tek kamera + IMU ile
  gerçek kuadrotorda iniş için uygulanmış.
- **Maskeden ıraksama:** Düz zemin ve aşağı bakan kamerada `ρ ∝ 1/h²`, bu
  yüzden `ρ̇/ρ = 2·v/h = 2D`. Yani **D, segmentasyon maskesinden
  ölçülebilir**; yükseklik bilinmeden. (karar paketi 01, Madde 3 bunu sınıyor.)
- **Önemli uyarı (de Croon):** Sabit kazançlı ıraksama kontrolünde, ölçüm
  gecikmesi ve gürültüsü yüzünden araç yere yaklaşınca **kendiliğinden
  salınmaya** başlar; kararlılık yalnız yüzeye uzaklığa bağlıdır. Çözüm:
  kazancı yükseklikle birlikte azaltmak. Bizde algı gecikmesi var
  (0.04–0.08 s + ≤10 Hz referans), bu doğrudan bizi ilgilendiriyor.

### 3.4 Karma
Yatayda IBVS (merkez hatası), dikeyde PBVS veya ıraksama gibi eksen başına
farklı yaklaşım. Pratikte en sık görülen.

---

## 4. Bugünkü P katsayılı yasa mantıklı mı?

**Kısa cevap: evet, mantıklı ve literatürle uyumlu; ama dört bilinen zayıf
noktası var.**

**Neden mantıklı:**
- IBVS'nin kanonik yasası zaten orantılı.
- Bugünkü irtifa yasası `v = clamp(0.35·h, 0.3, v_tavan)`, kırpma aralığında
  aslında **sabit ıraksama yasası** (`D = v/h = 0.35`). Yani arıların
  stratejisi, yalnız `h` kameradan değil EKF'den geliyor.

**Zayıf noktalar ve önerilen çareler:**

| # | Sorun | Bizde görüldüğü yer | Çare |
|---|---|---|---|
| 1 | `ρ` yükseklikle doğrusal değil (`ρ ∝ 1/h²`); sabit katsayıyla etkin kazanç yükseklikle çok değişir | Alan yasası `ceiling·(1−ρ)` | `ρ` yerine `1/√ρ` (yükseklikle orantılı, site boyutu sabitken) ya da `log ρ` kullanmak; veya kazanç programlama |
| 2 | Gecikme + sabit kazanç → alçakta salınım | Henüz gözlenmedi; gürültü/gecikme senaryolarında sınanmalı | Kazancı yükseklikle azaltmak (de Croon) |
| 3 | Ölçüm kaybı: site kadrajı taşınca `ρ` donuyor | **Veri setinde 3 sert temas (1.10–1.33 m/s):** COMMIT'te donmuş `ρ` ile alan yasası sabit hız verdi | Ölçüm yokken irtifa yasasına düşmek: `min(alan, irtifa)` |
| 4 | Saf P bozucu altında kalıcı hata bırakır; kadraj kısıtını bilmez | Yatayda rüzgâr; süzülme konisi | I terimi / ileri besleme; kısıt için MPC veya bariyer |

---

## 5. Ortak altyapı önerisi (klasik + RL)

Herkes aynı şeylerle çalışırsa sonuçlar karşılaştırılabilir:

1. **Aynı tesis.** Doğrulanmış Simulink modeli:
   - z: K ≈ 1.00, t90 0.36 s (OFFBOARD ve mod yolu aynı sonucu veriyor).
   - x, y: K 0.98, ölü zaman 0.24 s, zaman sabiti 0.50 s; rüzgârda K ≈ 0.95.
   - Model basit (eksen başına birinci mertebe + gecikme). RL için Python'a
     birebir taşınabilir; taşınan sürüm doğrulama tablosundaki sayıları
     üretmeli.
2. **Aynı arayüz.**
   - Gözlem: `ρ`, site merkezi `(u, v)`, `view_bounded`, `h_ekf`, `vz_ekf`,
     `vxy_ekf`, D tahmini, durum (VALIDATE/COMMIT).
   - Eylem: `(vx, vy, vz)_ref`, ≤ 10 Hz, hız sınırlarıyla kırpılmış.
3. **Aynı senaryolar:** S0 (yakın site, birincil), S0b (uzak site), gürültü,
   gecikme, rüzgâr, site değişimi, W5 (aynı gözlemciyle).
4. **Aynı ölçütler:** temas hızı (<1.0 / <0.5 oranı, ortanca, en büyük),
   yatay iniş hatası, süre, yol uzunluğu, kontrol eforu, kadraj ihlali,
   başarı oranı.
5. **Aynı veri bölmesi:** train/val/test (`_bolme/`). Ayar ve eğitim train'de,
   seçim val'da, rapor test'te.

---

## 6. Karar bekleyenler

1. Arayüz (§5.2) ikimiz için de uygun mu? Eksik gözlem var mı?
2. RL ortamı MATLAB'da mı (RL Toolbox, Simulink modeli doğrudan) yoksa
   Python'da mı (modelin Python eşi)?
3. Alan yasası için `1/√ρ` dönüşümü ve `min(alan, irtifa)` yedeği temel kola
   da eklensin mi, yoksa temel kol bugünkü haliyle mi kalsın?
4. DeePC ve VRFT ek karşılaştırma olarak plana girsin mi?

---

## Kaynaklar

- Chaumette & Hutchinson, "Visual servo control" (IBVS/PBVS temeli).
- Tahri & Chaumette, görüntü momentleriyle görsel servo.
- de Croon (2016), "Monocular distance estimation with optical flow
  maneuvers and efference copies: a stability-based strategy", *Bioinspiration
  & Biomimetics* 11(1). — https://arxiv.org/abs/1506.01153
- Ho, de Croon ve ark., "Adaptive control strategy for constant optical flow
  divergence landing". — https://arxiv.org/abs/1609.06767
- Alkowatly, Becerra, Holderbaum (2015), "Bio-inspired autonomous visual
  vertical control of a quadrotor UAV", *JGCD* 38(2). (tau teorisi)
- Campi, Savaresi ve ark., VRFT. — https://marco-campi.unibs.it/pdf-pszip/nonlinear-VRFT-TAC.pdf
- Coulson, Lygeros, Dörfler, "Data-Enabled Predictive Control: In the
  Shallows of the DeePC". — https://arxiv.org/abs/1811.05890
- Elokda ve ark., "Data-enabled predictive control for quadcopters", *IJRNC*
  31(18). — https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9291934/
- Zhang ve ark., kadraj kısıtlı sabit zamanlı IBVS ile hareketli platforma
  iniş. — https://pure.bit.edu.cn/en/publications/image-based-fixed-time-visual-servoing-control-for-uav-landing-on/
- Berkenkamp ve ark. (2016), "Safe controller optimization for quadrotors
  with Gaussian processes". — https://arxiv.org/abs/1509.01066
- Scheper & de Croon (2020), evrimleşmiş iniş yasası (BEKLEYEN B).
