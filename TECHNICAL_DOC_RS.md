# Tehnička Dokumentacija - LifeLink [ESP32-S3]

Ovaj dokument je fokusiran na opis unutrašnjeg funkcionisanja LifeLink pametnog sata. Uključuje strukturu senzora, mrežnih stack-ova i komunikacije putem `I2C` sabirnice, kao i asinhrono upravljanje zadacima u FreeRTOS.

## Arhitektura Sistema i Menadžment Resursa (FreeRTOS)

Srce LifeLinka je moćan dvojezgarni `ESP32-S3` kontroler, gde se operacije dele koristeći ugrađeni FreeRTOS kako bi se održao interfejs na ekranima fluidno pri visokom frejmrejtu dok pozadinski zadaci obavljaju kritične akcije skeniranja tela.

**Raspodela Core/Taskova:**
- `app_main` dodeljuje se procesiranje na dnu i glavni LVGL interfejs (Ekran, GUI) koji komuniciraju sa LCD kontrolerom preko DMA SPI kanala.
- `sensor_read_task` (na višem prioritetu, Core 1) radi sinhronizaciju sa `QMI8658` preko `I2C`.
- `MAX30102` - C-Based driveri su implementirani kao asinhroni interrupti gde senzor pakuje podatke preko `FIFO` buffera a potom prosleđuje matricu do FFT operacije.
- `gsm_task` - GSM modem stoji na posebnom Task-u i isključivo non-blocking sluša `UART`, kako ne bi ometao crtanje interfejsa ukoliko padne konekcija sa operaterom.
- `ble_spp_server_task` – BlueTooth kanal je zadužen za slanje "Heartbeat" (Telemetry) logova svakih sekundu i prima interapte visokog prioriteta. 

### Hardverski Interrapti i `I2C` (Known Issues)

Zbog tight-loop prirode u senzorskim aplikacijama gde `MAX30102` konstantno upisuje podatke o optici na `I2C`, kao i Touch kontroler.

*Upozorenje implementatorima:*
Moguće je doći do poznatog "Interrupt Watchdog (WDT)" padanja modula, gde BlueTooth (`bt_controller_task`) zbog prevelikog prioriteta i lošije organizacije stare \`driver/i2c\` biblioteke u ESP-IDF v5.x dolazi do ISR preklapanja. Problem se rešava korišćenjem novije iteracije \`driver/i2c_master.h\`, kao i uvođenjem Mutex/Semafora za hardversko deljenje resursa `i2c` ili pauziranja senzorskog brzanja taska.  

## Analiza i Otkrivanje Pada (QMI8658 Advanced Logic)

Detekcija padova (Fall Detection) podeljena je na 3 state-machine faze koje ignorišu lažne trzaje. Algoritam ne reaguje isključivo na prebačen g-force (`< 0.6G za Pad, > 3.5G za udar`), već na:
1. `FREE_FALL` : Sistem detektuje potpun gubitak gravitacije (0 - 0.6G) uz periodičan nadzor. Zadržava referentne podatke pre uleta u ambis, za kasniju kalkulaciju orijentacije.
2. `IMPACT_DETECTED` : Nakon što G prevaziđe definisan threshold (> 3.5G MAX). Ako ne, State Machine se resetuje `500ms` bez reakcija.
3. `STILLNESS & ANGLE CHECK` (Verifikaciji Mirnoće): Padanje se mora završiti sa `STILLNESS` uslovom u trajanju od barem 5 sekundi a potom sledi kalkulisanje kosinusa (Dot-Product ugla) prvobitnog vektora sa sadašnjim iznosom `ref_ax`, `ref_ay`. Sat zahteva da se promena ugla desi preko **60 stepeni**.


## Komunikacija i SOS Prijavljivanje (SIM800L GSM & GPS LC76G)

Nakon verifikovanog pada, sistem ispaljuje asinhroni task koji preko hardverskog SIM800L modula (uz MicroSIM na 2G GPRS mreži) upisanom broju isporučuje preformatirani URL i rezultate analitike padanja i vitala - **Lokacija (`$GNGGA` `$GNRMC` format od LC76G)** sa Google Maps string šablonom.

Struktura rešavanja Modula (Posebna Sekcija u README_RS.md).
- Čip radi isključivo tako što prima i pinguje AT command-base protokole. Modifikovan je tako da se pali nasilnim *pulse*-ovanjem GPIO pina umesto manuelnog pritiska dugmeta.
- Oprezni na 2G mreže i `[+CREG: 1,3]` Registration Denied koji zahteva instaliranje 1000µF elektrolitskog i 100nF keramičkog kondenzatora direktno na VCC/GND pinove SIM800L modula radi apsorbovanja 2A strujnih pikova. SIM800L se napaja direktno sa Li-Ion baterije (3.7–4.2V) bez potrebe za boost konvertorom. 
- *SMS Encoding* je čisti GSM (Text-Mode) podešen sa `AT+CMGF=1` i zahteva da `CTRL+Z (ASCII 26 - Substitut)` potvdi odlazak paketa iz TX buffera. 

**Preklapanje Ekran-C (LVGL) i C++ senzora** 
Svi pozivi ka `ui_Label_setText()` za LCD displej MORAJU proći `example_lvgl_lock()` Semaphore pozive na `lvgl_mux` iz FreeRTOS-a zbog rigidne prirode frejmvorka. 

## Upravljanje Pametnim Napajanjem i Displejom (AXP2101)
`axp_get_batt_percent` implementiran u `PMU` petlji, a u kombinaciji sa kapacitivnim tasterom proverava statiku ruku na `CST92xx` staklu. Nakon 15s bez ikakvog prekida odozdo, MCU pauzira crtanje sa `esp_lcd_panel_disp_on_off` (AMOLED se "gasi" i resetuje, tako crna boja efektivno gasi pixel), a potom ukida signal na LCD Backlight-u za deep-sleep efekte ekrana.

## Prateća Mobilna Aplikacija (Flutter Companion)

Flutter bazirana cross-platform aplikacija koja se putem BLE SPP protokola povezuje sa LifeLink satom i proširuje sistem sa dodatnim mogućnostima na mobilnom telefonu.

**BLE Protokol Komunikacije:**
- Servisni UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- Karakteristika UUID: `beb5483e-36e1-4688-b7f5-ea07361b26a8`
- Format podataka: `STATUS G:<g_force> P:<heartRate> S:<spo2> B:<battery> Lat:<lat> Lon:<lon>`

**Arhitektura:**
- `BleService` – Singleton za BLE scan/connect/subscribe sa StreamController-ima za reaktivne podatke.
- `SensorProvider` (ChangeNotifier) – Parsira BLE podatke, upravlja 3-faznim alarmom (Safe/Warning/Alarm), pokreće odbrojavanje i izvršava hitne akcije (Call/SMS/SOS).
- `DashboardScreen` – Dashboard sa metrikama, mapom i status karticama sa boja-kodiranim stanjima (Cyan/Amber/Red).
- `SettingsScreen` – Konfiguracija hitnog kontakta, akcije pada, BLE uređaja i dozvola.

**Hitne Akcije po isteku odbrojavanja:**
- `FallAction.call` → Direktan poziv hitnom kontaktu preko `flutter_phone_direct_caller`
- `FallAction.sms` → SMS sa GPS koordinatama preko `url_launcher` (`sms:` URI)
- `FallAction.sos` → Android SOS intent putem `android_intent_plus`

## Reference i Literatura

1. **ESP-IDF Programming Guide** - Zvanična dokumentacija za ESP32-S3, upravljanje zadacima, nove I2C Master drajvere i Interrupt Watchdog (WDT).
2. **FreeRTOS API Reference** - Dokumentacija za arhitekturu baziranu na prekidima i mehanizme deljenja (Mutex, Semaphore) pri korišćenju i preklapanju I2C sabirnice i ekrana (`lvgl_mux`).
3. **LVGL (Light and Versatile Graphics Library)** - Zvanična dokumentacija o portovanju prikaza, radu sa framebuffer-ima, integraciji na ESP32 (LCD kontroler, DMA i uslovi thread-safe interakcije).
4. **QMI8658 6-Axis IMU Datasheet** - Specifikacija hardvera inercijalne jedinice, analize pragova slobodnog pada ("FREE_FALL") i kalkulisanja promena ugla na osnovu ubrzanja i žiroskopskih vrednosti.
5. **MAX30102 Datasheet** - Implementacija FIFO bafera, obrade sirovih IR/Red odziva za računanje SpO2 i pulsa u C drajveru uz pomoć prepoznavanja vrhova impulsa.
6. **SIM800L GSM Module AT Commands** - Priručnik za komandovanje modemom (SMS Text mode `AT+CMGF=1`, slanje lokacija) i adresiranje prekomerne potrošnje pri registraciji u GPRS mrežu (CME Errors i CREG proces). SIM800L radi na 3.7–4.2V i kompatibilan je sa standardnim AT komandama.
7. **Quectel LC76G GNSS Module Protocol Specification** - Dekodiranje NMEA standardizovanih rečenica (`$GNGGA`, `$GNRMC`) radi vađenja i prosleđivanja preciznih geografskih koordinata i formiranja Google Maps linkova.
8. **AXP2101 PMIC Datasheet** - Podešavanje limita punjenja, čitanje postotka baterije preko Fuel Gauge algoritama i upravljanje dubokim i sleep gasenjem celog sistema.
