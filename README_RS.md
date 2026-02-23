# LifeLink Pametni Sat - ESP32-S3

[🇬🇧 English documentation is available in README.md](README.md)

LifeLink je napredni prototip pametnog sata izgrađen na **ESP32-S3** platformi. Koristi **ESP-IDF** u kombinaciji sa grafičkom bibliotekom **LVGL** za iscrtavanje prelepog korisničkog interfejsa na okruglom AMOLED ekranu rezolucije 466x466 piksela. Prevenstveno je fokusiran na brigu o starijim i ugroženim licima, praćenje zdravstvenih parametara i brzo reagovanje u hitnim situacijama.

## Glavne Funkcionalnosti

- **Napredna Detekcija Pada**: Koristi QMI8658 IMU (Akcelerometar + Žiroskop) za otkrivanje naglih padova i jakih udaraca o tlo. Zahteva period zadržavanja u nepomičnom stanju i specifičnu promenu ugla nagiba nakon udara kako bi potvrdio pravi pad a izbegao lažne uzbune (prilikom npr. trčanja ili naglih pokreta ruke).
- **Simulacija Pada & Poništavanje**: Korisnici mogu lako testirati sistem simulacijom pada preko samog interfejsa sata. Pravi pad okida 15-sekundno odbrojavanje na ekranu; ako je greška ili korisniku nije potrebna pomoć, jednim dodirom po ekranu proces se poništava i prekidaju se hitne akcije.
- **Automatski GSM SMS Alarmi**: Komunicira sa GSM A6 Modulom kako bi asinhrono (u pozadini) poslao SMS upozorenja koja sadrže:
  - Precizne GPS koordinate formatirane kao direktan Google Maps link (Lokacija gde se osoba nalazi).
  - Otkucaje srca u sekundi akcidenta.
  - Informaciju da li je pad bio stvaran ili samo test/simulacija.
- **Zdravstveni Parametri Uživo**: Sistem redovno očitava brzinu pulsa i oksigenaciju krvi u procentima (SpO2) uz pomoć MAX30102 senzora na poleđini. Novi podaci se uvek sveže ažuriraju na početnom ekranu.
- **Interaktivni Korisnički Interfejs (LVGL)**: 
  - Dinamična statusna traka na vrhu ekrana sa indikatorima za GPS konekciju, GSM povezanost (sa promenom boje u zavisnosti od signala), status Baterije i Bluetooth Mreže.
  - Navigacija putem prevlačenja prsta po ekranu nalevo i nadesno (Meni gestovi).
  - Zaseban "Podešavanja ekran" sa ugrađenom namenskom uveličanom numeričkom tastaturom koja pojednostavljuje unos ili promenu telefonskog broja hitne službe ili bliskog lica (nije potrebna aplikacija na telefonu).
- **Pregled Senzora (Debug)**: Lako dostupan "DEBUG" prekidač i pogled implementiran pravo u UI sistem koji omogućava programerima uživo posmatranje X, Y, Z , i G sile, korisno zbog finog štelovanja parametara padova.

## Hardver

- **Mikrokontroler**: ESP32-S3
- **Displej**: Okrugli AMOLED ekran (466x466)
- **Mreža / Komunikacija**: GSM A6 Modul (Komunikacija bazirana na AT Komandama)
- **IMU Senzori**: QMI8658 (Praćenje pokreta i nagiba)
- **Senzori Zdravlja**: MAX30102 (Otkucaji srca i SpO2)
- **Power Management (Baterija i Struja)**: AXP2101

## Podešavanje i Pokretanje

Ovaj projekat je izgrađen i napisan u jezicima C i C++, preko [Espressif ESP-IDF frejmvorka](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/) (v5.x i vise je preporuka).

### 1. Konfiguracija
Setujte vaš procesor na ESP32-S3 i uđite u meni opcije kako bi uverili konfiguracije:
```bash
idf.py set-target esp32s3
idf.py menuconfig
```
### 2. Građenje arhitekture i Flešovanje
Kompilirajte kod i prebacite softver na mikrokontroler:
```bash
idf.py build
idf.py -p COMX flash monitor
```

 *(COMX podesite na port vašeg esp programatora)*

## Pregled i mapiranje Ekrana

1. **Glavni Skrin (Ekran 1)**: Brojčanik (Sat), glavni vitali i najosnovnije konektivne ikonice.
2. **Prikaz Senzora (Ekran 2)**: Test dugme za simulaciju pada bez prave povrede, uz Debug panel parametara žiroskopa za stručno lice.
3. **Podešavanja (Ekran 3)**: Prikaza ogromne numeričke tastature gde prstima svako može uneti pretplatnički broj mobilnog telefona i sačuvati podešavanje u obezbeđenu RAM particiju sata bez ometanja. 
4. **Ekran u hitnim situacijama (Ekran 4)**: Alarmantan crveni ekran, koji glasnim i krupnim tekstom nudi korisniku obaranje upozorenja ako on stoji i zapravo je dobro. 
