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

Uređaj neprekidno meri vitalne parametre korisnika kao što su brzina pulsa (BPM) i nivo kiseonika u krvi (SpO2) uz pomoć MAX30102 medicinskog senzora. Njegova najbitnija funkcija je napredna detekcija pada – sat integriše QMI8658 inercijalni senzor (akcelerometar i žiroskop) sa posebnim softverskim algoritmom koji prepoznaje slobodan pad praćen jakim udarom i kasnijim mirovanjem, čime se efikasno filtriraju lažne uzbune (na primer usled naglih pokreta ruku). Kada detektuje pravi pad, uređaj prikazuje crveno obaveštenje i započinje odbrojavanje u trajanju od 5 sekundi. Ukoliko korisnik nije zaista povređen i ne treba mu pomoć, odbrojavanje može poništiti na dodir. U suprotnom, SIM800L GSM mrežni modul sata u potpunoj autonomiji (bez pametnog telefona) šalje SMS na programirani broj za hitne slučajeve. Poruka sadrži status zdravstvenih vitala u momentu pada, upozorenje, kao i direktan link na GPS lokaciju na Google mapama. 

Celokupan interakcijski meni je osmišljen jednostavno, podržavajući unos telefonskog broja direktno na uveličanoj tastaturi interfejsa. Autonomija ovog mrežnog modula i samostalan rad u integrisanom grafičkom okruženju čine LifeLink pouzdanim, inovativnim i korisnim rešenjem za unapređenje zdravstvene bezbednosti.

---

## PRINCIP RADA I PRAKTIČNA REALIZACIJA

### a.) Kratak opis
LifeLink sinhronizuje moćan ESP32-S3 mikrokontroler i periferni hardver uz naprednu ESP-IDF strukturu zadataka (FreeRTOS Tasks), i time upravlja paralelnim procesima na više jezgara. Na vrhu operativnog sistema, LVGL renders menije osetljive na dodir, kojim korisnici naviguju prevlačenjem prsta (swipe gestovi). Program u realnom vremenu šalje komande I2C interfejsu za očitavanje MPU žiroskopa i optičkog oksimetra na poleđini uređaja.

Za efikasnu i besprekornu komunikaciju sa spoljašnjim svetom, sat ima instaliran nezavisni SIM800L GSM modul. Ovaj kompaktan modul (svega 2.5×2.3 cm) napaja se direktno sa Li-Ion baterije (3.7–4.2V) bez potrebe za spoljnim boost konvertorom, čime se značajno poboljšava stabilnost napajanja. Prilikom registracije na baznu 2G GSM mrežu, modul može povući iznenadne strujne pikove od čak 2 Ampera. Za apsorbovanje tih pikova koriste se 1000µF elektrolitski i 100nF keramički kondenzator zalemljeni direktno na VCC i GND pinove modula. Modul komunicira sa uređajem preko serijskog UART protokola, koristeći standardne AT komande. Softver uključuje i automatski recovery mehanizam koji restartuje modul nakon 3 uzastopna neuspeha komunikacije.

### b.) Komponente

| Naziv komponente | Za šta se koristi? |
| --- | --- |
| **ESP32-S3** | Mikrokontroler koji pokreće OS, komunicira sa senzorima i vrši grafičku obradu. |
| **Okrugli AMOLED Ekran (466x466)** | Visokokvalitetni ekran osetljiv na dodir namenjen prikazu korisničkog interfjesa sata. |
| **QMI8658 IMU Senzor** | Detektovanje inercijalnih G sila udaraca, rotacionog nagiba i mirovanja korisnika. |
| **SIM800L GSM Modul** | Kompaktan Quad-band 2G modul (2.5×2.3 cm) za automatsko slanje hitnih SMS lokacija bez telefona. Napaja se direktno sa 3.7V Li-Ion baterije. |
| **MAX30102 Senzor** | Optičko očitavanje otkucaja srca i zasićenosti kiseonika za nadgledanje bioloških funkcija. |
| **AXP2101 PMIC** | Regulacija strujne potrošnje i bezbedno upravljanje punjenjem Li-Ion baterije. |
| **Li-Ion Baterija (3.7V)** | Prenosivi izvor energije kapaciteta da obezbedi višiečasovni stabilni nezavisan rad. |

### c.) Šema sistema
![alt text](components/DOCs/LifeLink-sch.jpg)  
**Slika 4 – Praktična realizacija senzora LifeLink i blok šema komunikacije uređaja**

### d.) Programski kod

**Slika 6 - Isečak programiranog C koda za formatiranje i slanje SMS-a (FreeRTOS/UART):**
```c
// Slanje automatskog SMS-a prilikom akcidenta - deo funkcije slanja modula gsm_a6.c (kompatibilno sa SIM800L)
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

Druga uspešna tačka leži u prelasku na SIM800L GSM modul koji se napaja direktno sa Li-Ion baterije (3.7–4.2V) bez potrebe za spoljnim boost konvertorom, uz uvođenje filter kondenzatora (1000µF elektrolitski + 100nF keramički) paralelno na VCC/GND pinove modula. Ovo je doprinelo potpunoj mrežnoj pokrivenosti i stabilnosti signala, eliminišući probleme sa gubitkom 5V napajanja koje je prethodni boost konvertor uzrokovao. Softverski automatski recovery mehanizam dodatno obezbeđuje pouzdanost sistema restartovanjem modula nakon uzastopnih neuspeha. Konačni rezultat je potpuno funkcionalan pametni sat impresivnog interfejsa koji, za razliku od modernih pametnih satova, nije vezan isključivo za mobilni telefon putem Bluetooth veze, već poseduje integrisan zdravstveni biološki nadzor i pravovremeno izveštava o incidentima na sopstveno GSM nezavisno prenešeno mobilno SMS rešenje, obezbeđujući na taj način visok stepen prenosive bezbednosti za najugroženije grupe.

---

## MOBILNA APLIKACIJA (LifeLink Companion App)

Pored autonomnog rada sata, razvijena je i prateća mobilna aplikacija korišćenjem **Flutter** frejmvorka, koja proširuje mogućnosti LifeLink sistema putem Bluetooth Low Energy (BLE) veze sa satom.

### a.) Funkcionalnosti Aplikacije

| Funkcionalnost | Opis |
| --- | --- |
| **Dashboard Uživo** | Prikazuje vitalne parametre u realnom vremenu: puls (BPM), oksigenacija krvi (SpO2), jačina udara (G-Force) i GPS lokacija. |
| **BLE Konekcija** | Automatsko ili manuelno povezivanje sa LifeLink satom putem BLE SPP protokola. Prikazuje status konekcije i nivo baterije sata. |
| **Detekcija Pada (Ogledalo)** | Aplikacija prima podatke sa sata i implementira sopstveni 3-fazni sistem: **Bezbedno** (zeleno) → **Upozorenje** (narandžasto) → **Alarm** (crveno). |
| **Odbrojavanje i Otkazivanje** | 5-sekundno odbrojavanje pre aktiviranja hitnih akcija, sa mogućnošću otkazivanja lažnog alarma. |
| **Automatski SOS Odgovor** | Po isteku odbrojavanja, izvršava konfigurisan odgovor na pad: poziv, SMS sa GPS lokacijom ili sistemski SOS. |
| **Podešavanja** | Konfiguracija hitnog kontakta (ime i broj), izbor akcije pada (poziv/SMS/SOS), trajanje odbrojavanja i MAC adresa uređaja. |
| **Mapa Lokacije** | Prikaz korisnikove lokacije na interaktivnoj mapi korišćenjem GPS-a telefona za pomoć spasiocima. |
| **Haptics & Zvučni Alarm** | Vibracija i zvučni alarm na telefonu prilikom detekcije pada za dodatno upozorenje. |

### b.) Tehnologije Mobilne Aplikacije

| Tehnologija | Namena |
| --- | --- |
| **Flutter (Dart)** | Cross-platform razvoj za Android, iOS i Windows iz jednog koda. |
| **flutter_blue_plus** | BLE komunikacija sa ESP32-S3 satom (servisni UUID: `4fafc201-...`). |
| **Provider** | State management za reaktivno ažuriranje korisničkog interfejsa. |
| **Geolocator** | Očitavanje GPS lokacije telefona za SOS poruke. |
| **flutter_map + latlong2** | Prikaz lokacije na OpenStreetMap mapi. |
| **shared_preferences** | Lokalno čuvanje korisničkih podešavanja (kontakt, akcija, MAC adresa). |
| **url_launcher / flutter_phone_direct_caller** | Pokretanje poziva ili SMS-a sa telefona. |
| **vibration / audioplayers** | Taktilni i zvučni alarm pri detekciji pada. |

### c.) Arhitektura Aplikacije

Aplikacija koristi **MVVM** (Model-View-ViewModel) šablon:
- **`BleService`** – Singleton servis za BLE skeniranje, konekciju i pretplatu na karakteristike.
- **`SensorProvider`** – Centralni ChangeNotifier koji parsira podatke sa sata (`STATUS G:X.XX P:XX S:XX B:XX Lat:XX Lon:XX`), upravlja stanjima alarma i pokreće hitne akcije.
- **`DashboardScreen`** – Glavni ekran sa prikazom metrika, statusom konekcije i mapom.
- **`SettingsScreen`** – Ekran za konfiguraciju hitnog kontakta, akcije i BLE uređaja.


## ZAKLJUČAK
Predstavljeni prototip LifeLink platforme pruža izuzetan potencijal. Njegova najveća prednost jeste modularan multifunkcionalan pristup malog faktora forme. Mogući dalji rad na projektu zasigurno obuhvata dublju optimizaciju upotrebe baterije - kreiranjem takozvanog "Deep Sleep" logičkog koraka gašenjem ekrana, izradu namenskog prilagođenog komercijalnog 3D kućišta koje će zaštititi celokupan sklop od vlage ili udaraca, kao i integraciju algoritama veštačke inteligencije (Edge Impulse/TinyML). Mogućnošću "treniranja" sitnih klasifikacionih neuronskih mreža na samom ESP32 hardveru bi se drastično povećala moć raspoznavanja udesa i predviđanja vrste skoka na osnovu bogatijih baza podataka (dataseta). Bez obzira na to što je trenutni uređaj prototipskog karaktera, efikasnost i samostalnost testiranih modula dokazuju njegovu visoku i spasonosnu nosivu primenjivost na realnom terenu.

## ZAHVALNICA
Zahvaljujemo se ...

## LITERATURA I REFERENCE
[1] ESP-IDF Framework platforma i FreeRTOS repozitorijumi – Espressif – https://docs.espressif.com  
[2] LVGL Graphic Library – Ogranak za vizuelizaciju i dizajn i menadžment – https://lvgl.io  
[3] SIM800L GSM Module Datasheet / AT Commands – SIMCom dokumentacija  
[4] QMI8658C Attitude & Motion Detection IMU Datasheet i Data-tabela za C integraciju  
[5] MAX30102 High-Sensitivity Pulse Oximeter and Heart-Rate Sensor for Wearable Health uređaje  
[6] Google Maps URL Schema – Standardi povezivanja API okvira pretrage lokacije – https://developers.google.com/maps/documentation/urls/get-started
