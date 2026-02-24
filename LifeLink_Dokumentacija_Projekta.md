# LIFELINK
**- Pametan sat koji čuva zdravlje, detektuje padove i automatski poziva pomoć -**

**Ime Tima:** [Ime Tima]  
**Autori:** [Ime Prezime]*  
Učenici IV-1 razreda  
Tehnička škola Pirot  

**Mentor:** BOBAN BLAGOJEVIĆ, dipl. ing. elektrotehnike, nastavnik elektro-grupe predmeta u Tehnička škola Pirot

---

## REZIME

Projekat LifeLink predstavlja napredni pametni sat baziran na ESP32-S3 platformi, osmišljen pre svega kao sigurnosni asistent za starija i ugrožena lica. LifeLink kombinuje procesorsku moć ESP-IDF radnog okruženja sa LVGL grafičkom bibliotekom, pružajući vizuelno privlačan korisnički interfejs na kružnom AMOLED ekranu rezolucije 466x466 piksela. Hardver i softver sata zajednički nadgledaju zdravstvene parametre i upozoravaju staratelje u hitnim situacijama.

Uređaj neprekidno meri vitalne parametre korisnika kao što su brzina pulsa (BPM) i nivo kiseonika u krvi (SpO2) uz pomoć MAX30102 medicinskog senzora. Njegova najbitnija funkcija je napredna detekcija pada – sat integriše QMI8658 inercijalni senzor (akcelerometar i žiroskop) sa posebnim softverskim algoritmom koji prepoznaje slobodan pad praćen jakim udarom i kasnijim mirovanjem, čime se efikasno filtriraju lažne uzbune (na primer usled naglih pokreta ruku). Kada detektuje pravi pad, uređaj prikazuje crveno obaveštenje i započinje odbrojavanje u trajanju od 15 sekundi. Ukoliko korisnik nije zaista povređen i ne treba mu pomoć, odbrojavanje može poništiti na dodir. U suprotnom, GSM A6 mrežni modul sata u potpunoj autonomiji (bez pametnog telefona) šalje SMS na programirani broj za hitne slučajeve. Poruka sadrži status zdravstvenih vitala u momentu pada, upozorenje, kao i direktan link na GPS lokaciju na Google mapama. 

Celokupan interakcijski meni je osmišljen jednostavno, podržavajući unos telefonskog broja direktno na uveličanoj tastaturi interfejsa. Autonomija ovog mrežnog modula i samostalan rad u integrisanom grafičkom okruženju čine LifeLink pouzdanim, inovativnim i korisnim rešenjem za unapređenje zdravstvene bezbednosti.

---

## PRINCIP RADA I PRAKTIČNA REALIZACIJA

### a.) Kratak opis
LifeLink sinhronizuje moćan ESP32-S3 mikrokontroler i periferni hardver uz naprednu ESP-IDF strukturu zadataka (FreeRTOS Tasks), i time upravlja paralelnim procesima na više jezgara. Na vrhu operativnog sistema, LVGL renders menije osetljive na dodir, kojim korisnici naviguju prevlačenjem prsta (swipe gestovi). Program u realnom vremenu šalje komande I2C interfejsu za očitavanje MPU žiroskopa i optičkog oksimetra na poleđini uređaja.

Za efikasnu i besprekornu komunikaciju sa spoljašnjim svetom, sat ima instaliran nezavisni GSM A6 modul. Prilikom registracije na baznu 2G GSM mrežu, on može povući iznenadne strujne pikove od čak 2 Ampera. Kako on ne bi oborio rad ostalih komponenti uređaja (tzv. "Brownout reset" pad napona), implementirali smo namenski AXP2101 menadžer snage u kombinaciji sa velikim polarizovanim kondenzatorima ne bi li nivoi napona baterije postali imuni na varijacije. Modul komunicira sa uređajem preko serijskog UART protokola, preko serije pouzdanih programerskih AT komandi.

### b.) Komponente

| Naziv komponente | Za šta se koristi? |
| --- | --- |
| **ESP32-S3** | Mikrokontroler koji pokreće OS, komunicira sa senzorima i vrši grafičku obradu. |
| **Okrugli AMOLED Ekran (466x466)** | Visokokvalitetni ekran osetljiv na dodir namenjen prikazu korisničkog interfjesa sata. |
| **QMI8658 IMU Senzor** | Detektovanje inercijalnih G sila udaraca, rotacionog nagiba i mirovanja korisnika. |
| **GSM A6 Modul** | Komunikacija na mobilnoj 2G mreži za automatsko slanje hitnih SMS logacija bez telefona. |
| **MAX30102 Senzor** | Optičko očitavanje otkucaja srca i zasićenosti kiseonika za nadgledanje bioloških funkcija. |
| **AXP2101 PMIC** | Regulacija strujne potrošnje i bezbedno upravljanje punjenjem Li-Ion baterije. |
| **Li-Ion Baterija (3.7V)** | Prenosivi izvor energije kapaciteta da obezbedi višiečasovni stabilni nezavisan rad. |

### c.) Šema sistema
*(Šematski prikaz rada LifeLink sistema / dijagram povezivanja MPU/ESP i GSM modula)*  
**Slika 4 – Praktična realizacija senzora LifeLink i blok šema komunikacije uređaja**

### d.) Programski kod

**Slika 6 - Isečak programiranog C koda za formatiranje i slanje SMS-a (FreeRTOS/UART):**
```c
// Slanje automatskog SMS-a prilikom akcidenta - deo funkcije slanja modula gsm_a6.c
void send_sos_sms(const char* phone_no, int hr, float latitude, float longitude) {
    char sms_payload[256];
    // Formatiranje SOS tekstualne poruke sa vitalnim parametrima i lokacijom
    snprintf(sms_payload, sizeof(sms_payload), 
        "Hitan Slucaj! Detektovan PAD!\n"
        "Puls: %d BPM\n"
        "Lokacija: https://maps.google.com/?q=%.6f,%.6f", 
        hr, latitude, longitude);
        
    // Izvrsavanje AT komandi preko hardverskog UART port komunikacije
    uart_write_bytes(UART_NUM_1, "AT+CMGF=1\r", 10);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    char at_cmd[64];
    snprintf(at_cmd, sizeof(at_cmd), "AT+CMGS=\"%s\"\r", phone_no);
    uart_write_bytes(UART_NUM_1, at_cmd, strlen(at_cmd));
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Zapocni upis u serijski port
    uart_write_bytes(UART_NUM_1, sms_payload, strlen(sms_payload));
    
    // Potvrdi i posalji komandu SMS poruke sa terminalnim karakterom 26(CTRL+Z)
    char ctrl_z = 26;
    uart_write_bytes(UART_NUM_1, &ctrl_z, 1);
}
```

---

## REZULTAT
Nakon lemljenja fizičkih sklopova, razrade C koda po preporučenoj ESP-IDF arhitekturi na dva procesorska jezgra i renderovanja LVGL menija, sproveli smo niz stres testova uređaja. Fokus testiranja bio je kalibrisanje osetljivosti detekcije pada QMI8658 modula. Algoritam je isfiltriran za sprečavanje lažnih alarma tako što uz brzu promenu ubrzanja (slobodan pad o tlo) sada nužno ispituje i potonje statično polje i preokrenut ugaoni vektor nagiba, uspešno izbegavajući greške prilikom mahutalja rukom ili jednostavnog guranja. 

Druga uspešna tačka leži u uvođenju filter kondenzatora paralelno u odnosu na GSM strujni terminal napajanja, koji je doprineo potpunoj mrežnoj pokrivenosti i stabilnosti signala gde AXP2101 regulator uspeva neoboreno da snabdeva modul pri konektovanjima i do 2A pikovane struje. Konačni rezultat je potpuno funkcionalan pametni sat impresivnog interfejsa koji, za razliku od modernih pametnih satova, nije vezan isključivo za mobilni telefon putem Bluetooth veze, već poseduje integrisan zdravstveni biološki nadzor i pravovremeno izveštava o incidentima na sopstveno GSM nezavisno prenešeno mobilno SMS rešenje, obezbeđujući na taj način visok stepen prenosive bezbednosti za najugroženije grupe.

## ZAKLJUČAK
Predstavljeni prototip LifeLink platforme pruža izuzetan potencijal. Njegova najveća prednost jeste modularan multifunkcionalan pristup malog faktora forme. Mogući dalji rad na projektu zasigurno obuhvata dublju optimizaciju upotrebe baterije - kreiranjem takozvanog "Deep Sleep" logičkog koraka gašenjem ekrana, izradu namenskog prilagođenog komercijalnog 3D kućišta koje će zaštititi celokupan sklop od vlage ili udaraca, kao i integraciju algoritama veštačke inteligencije (Edge Impulse/TinyML). Mogućnošću "treniranja" sitnih klasifikacionih neuronskih mreža na samom ESP32 hardveru bi se drastično povećala moć raspoznavanja udesa i predviđanja vrste skoka na osnovu bogatijih baza podataka (dataseta). Bez obzira na to što je trenutni uređaj prototipskog karaktera, efikasnost i samostalnost testiranih modula dokazuju njegovu visoku i spasonosnu nosivu primenjivost na realnom terenu.

## ZAHVALNICA
Zahvaljujemo se ...

## LITERATURA I REFERENCE
[1] ESP-IDF Framework platforma i FreeRTOS repozitorijumi – Espressif – https://docs.espressif.com  
[2] LVGL Graphic Library – Ogranak za vizuelizaciju i dizajn i menadžment – https://lvgl.io  
[3] GSM A6 Datasheet / AT Commands – Ai-Thinker dokumentacija  
[4] QMI8658C Attitude & Motion Detection IMU Datasheet i Data-tabela za C integraciju  
[5] MAX30102 High-Sensitivity Pulse Oximeter and Heart-Rate Sensor for Wearable Health uređaje  
[6] Google Maps URL Schema – Standardi povezivanja API okvira pretrage lokacije – https://developers.google.com/maps/documentation/urls/get-started
