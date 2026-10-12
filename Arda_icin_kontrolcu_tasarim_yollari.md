# Arda için: Kontrolcü nasıl tasarlanır, biz hangi yoldayız?

Selam Arda. Bu not, kontrolcü tarafında ne yaptığımızı ve bunun senin RL
çalışmanla nasıl birleştiğini basitçe anlatmak için. Repoya girmeyecek, sadece
okuman için.

---

## 1. Temel fikir: önce "tepkiyi" bilmen lazım

Bir arabaya hız sabitleyici tasarladığını düşün. Arabanın gaza nasıl tepki
verdiğini bilmeden iyi bir hız sabitleyici yapamazsın: Gaza basınca hemen mi
hızlanıyor, yarım saniye sonra mı? Yavaş yavaş mı, birden mi?

Drone için de aynı: "Şu hızla in" dediğimizde drone ne yapıyor? Kontrolcü
tasarlamanın bütün yolları, bu tepkiyi **nereden öğrendiğine** göre ayrılıyor.

Bir not: Drone'un motorlarını ve açısını PX4 zaten kontrol ediyor. Biz ona
sadece **hız komutu** veriyoruz. Yani bizim "araba" dediğimiz şey, PX4 dahil
bütün drone. İçine girmiyoruz, dışarıdan nasıl davrandığına bakıyoruz.

---

## 2. Bizim kullandığımız yol: ölç → modelle → tasarla

**Adım 1, ölç.** Gazebo'da drone'a "hızlan, dur, ters yöne hızlan" şeklinde
kare dalga hız komutları verdik ve ne yaptığını kaydettik.
- Dikeyde: K5 testleri.
- Yatayda: K5-xy testleri (23 uçuş).

**Adım 2, modelle.** Kayıtlardan basit formüller çıktı:

| Eksen | Ne yapıyor |
|---|---|
| Dikey (z) | Komutu neredeyse birebir izliyor; hedef hızın %90'ına 0.36 s'de ulaşıyor |
| Yatay (x, y) | Komuttan sonra 0.24 s hiç tepki vermiyor, sonra yavaş yavaş hızlanıyor (zaman sabiti 0.5 s) |

Yani yatay eksen dikeyden çok daha "uyuşuk". Rüzgâr bu davranışı değiştirmiyor,
sadece biraz zayıflatıyor.

**Adım 3, doğrula.** Veri setindeki gerçek inişlerin komutlarını bu formüllere
verdik. Formül, Gazebo'daki temas hızlarını 1–3 cm/s farkla tuttu; 4 m
platformdaki (W5) sert temas bile aynen çıktı. Yani bu formül güvenilir bir
**sanal drone**.

**Adım 4, tasarla.** Kontrolcüleri bu sanal drone üstünde tasarlayıp
ayarlıyoruz. Sanal drone'da bir iniş saniyenin altında sürüyor, Gazebo'da
dakikalar. Bu yüzden binlerce deneme yapabiliyoruz.

Bu yöntemin adı **sistem tanımlama + model tabanlı tasarım**. Mühendislikte en
yaygın yol.

---

## 3. Başka hangi yollar var?

**a) Fizik formülleriyle modellemek.** Drone'u ölçmek yerine kütlesinden,
motor gücünden, PX4'ün iç yapısından denklem kurmak. Arabayı hiç sürmeden
motor ve ağırlık bilgisinden hesaplamak gibi. Sorun: PX4'ün içindeki bütün
ayrıntıları bilmiyoruz, sonunda yine ölçüp kontrol etmek gerekiyor. Biz zaten
ölçtüğümüz için bize ek bir şey katmıyor.

**b) Model çıkarmadan doğrudan veriden tasarlamak.** Formül adımını atlayıp
kayıttan doğrudan "en iyi kontrolcü ayarı şu" sonucuna varmak.
- **VRFT:** Kayda bakıp "drone'un şöyle davranmasını istiyorum, bunu sağlayan
  ayar hangisi?" sorusunu tek hesapla çözüyor.
- **DeePC:** MPC'nin kardeşi. MPC formülle ileriyi tahmin eder; DeePC formül
  yerine geçmiş kayıtlara bakıp "geçen sefer böyle komut verince şöyle olmuştu"
  diye tahmin eder.
- İyi yanı model gerektirmemesi, kötü yanı verinin kalitesine çok bağlı olması.
  Tezde ek bir karşılaştırma olarak düşünüyoruz.

**c) Denemeyle ayarlamak.** Kontrolcünün yapısını sen belirlersin, ayarlarını
bir algoritma bulur: ayar seçer, iniş yapar, sonuca bakar, daha iyisini dener.
Biz bunu tasarım yöntemi olarak değil, **ayar yöntemi** olarak kullanıyoruz:
PID, LQI, MPC ve Fuzzy'nin ayarlarını eşit bütçeyle bu şekilde bulacağız ki
karşılaştırma adil olsun.

**d) Öğrenme (RL).** Senin yolun. Kontrolcünün yapısını bile önceden
belirlemiyorsun; ajan binlerce inişte deneme yanılmayla "hangi durumda hangi
komutu vermeli"yi kendisi öğreniyor.

### Özet tablo

| Yol | Drone'un tepkisini nereden öğreniyor | Kimde |
|---|---|---|
| Ölç + modelle | Ölçümden formül çıkararak | **Ana yol (Sefa)** |
| Fizik formülleri | Denklemlerden | Gerek yok |
| Doğrudan veriden (VRFT, DeePC) | Formülsüz, ham kayıttan | İsteğe bağlı ek |
| Denemeyle ayar | Tekrar tekrar deneyerek | Ayar yöntemi (Sefa) |
| Öğrenme (RL) | Deneme yanılmayla, kendi kendine | **Arda** |

---

## 4. Bu senin için ne demek?

Bu yollar birbirine rakip değil. Bizim çıkardığımız sanal drone senin için de
işe yarar:

- **Hızlı eğitim ortamı.** RL'in binlerce, on binlerce bölüme ihtiyacı var.
  Gazebo'da bu çok uzun sürer; sanal drone'da dakikalar. Formül çok basit
  (eksen başına "gecikme + yavaş yaklaşma"), istersen Python'a birebir
  taşınabilir.
- **Gerçekçi zorluklar eklenebilir.** Veri setinden gecikme, gürültü ve kare
  kaybı dağılımlarını çıkarıp her bölümde rastgele değiştirebiliriz. Bu,
  eğittiğin politikanın Gazebo'ya geçince bozulmamasına yardım eder.
- **Adil karşılaştırma.** Senin RL ajanın ile bizim kontrolcülerimiz aynı
  sanal drone'da, aynı senaryolarla, aynı ölçütlerle test edilirse sonuçlar
  doğrudan karşılaştırılabilir.

Bunun için tek şartımız: aynı şeyi görüp aynı şeyi üretmek.
- **Gözlem (ne görüyor):** site alan oranı ρ, site merkezinin kadrajdaki yeri,
  sitenin kadraja sığıp sığmadığı, EKF yüksekliği ve hızları, mod durumu.
- **Eylem (ne üretiyor):** x, y, z hız komutu, saniyede en fazla 10 kez.

---

## 5. Kamera nereye bağlanıyor? (sorduğun soru)

```
Kamera → Segmentasyon → ρ, site merkezi → [KONTROLCÜ / RL AJANI] → hız komutu → PX4
```

Kamera kontrolcüye, segmentasyondan çıkan özniteliklerin hız komutuna
dönüştüğü yerde bağlanıyor. Bugün kodda bu iş `descentSpeed` ve
`onCandidate` içinde. Senin RL ajanın da tam bu kutunun yerine geçecek.

**P katsayılı yasan mantıklı mı?** Evet. Kameradan gelen görüntüye göre
kontrol yapmanın ders kitabı yöntemi zaten orantılı (P) bir yasa. Hatta
bugünkü irtifa yasası `v = 0.35·h`, arıların inerken kullandığı stratejinin
aynısı: hızın yüksekliğe oranını sabit tutmak. Yere yaklaştıkça hız
kendiliğinden azalıyor.

Dikkat edilmesi gereken dört nokta var:
1. **ρ yükseklikle doğrusal değişmiyor** (yükseklik yarıya inince ρ dört
   katına çıkıyor). Sabit katsayıyla yasanın "sertliği" yükseklikle çok
   değişiyor. `1/√ρ` kullanmak bunu düzeltir, çünkü yükseklikle doğru orantılı.
2. **Gecikme + sabit katsayı** yere yakın salınıma yol açabilir. Literatürdeki
   çözüm: katsayıyı yükseklikle birlikte azaltmak.
3. **Site kadrajı taşınca ρ donuyor.** Veri setinde 3 sert temas (1.1–1.3 m/s)
   tam bu yüzden. Ölçüm yokken irtifa yasasına geçmek gerekiyor.
4. **Saf P rüzgâr altında yatayda kalıcı hata bırakır** ve kadraj sınırını
   bilmez. I terimi veya MPC gibi kısıt bilen bir yöntem gerekir.

Bunlar RL için de geçerli: Ajanın bu durumlarla (donmuş ρ, gecikme, rüzgâr)
eğitimde karşılaşması lazım, yoksa Gazebo'da şaşırır.

---

## 6. Birlikte karar vermemiz gerekenler

1. Yukarıdaki gözlem ve eylem listesi senin için uygun mu, eksik var mı?
2. RL ortamını nerede kuracaksın: MATLAB'da (RL Toolbox, Simulink modelini
   doğrudan kullanır) mı, Python'da (modelin Python eşi) mı?
3. Senaryo listesini (yakın site, uzak site, gürültü, rüzgâr, 4 m platform)
   ve ölçütleri (temas hızı, yatay hata, süre vb.) ortak kullanalım mı?

Daha ayrıntılı ve kaynaklı hali `KONTROLCU_TEORIK_ALTYAPI.md` dosyasında.
