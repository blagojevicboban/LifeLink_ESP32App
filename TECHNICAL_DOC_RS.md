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


## Komunikacija i SOS Prijavljivanje (GSM A6 & GPS LC76G)

Nakon verifikovanog pada, sistem ispaljuje asinhroni task koji preko hardverskog modula (A6 + MicroSIM na 2G GPRS mreži) upisanom broju isporučuje preformatirani URL i rezultate analitike padanja i vitala - **Lokacija (`$GNGGA` `$GNRMC` format od LC76G)** sa Google Maps string šablonom.

Struktura rešavanja Modula (Posebna Sekcija u README_RS.md).
- Čip radi isključivo tako što prima i pinguje AT command-base protokole. Modifikovan je tako da se pali nasilnim *pulse*-ovanjem GPIO pina umesto manuelnog pritiska dugmeta.
- Oprezni na 2G mreže i `[+CREG: 1,3]` Registration Denied koji zahteva instaliranje 1000µF kondenzatora pre RF logike na ploču usled 2+ Ampera vuče modema zbog lošijeg rešenja regulacije 3.3V od strane S3 pločice pod inercijom LVGL ekrana sa svetlosnim intenzitetom pozadinske grafike. 
- *SMS Encoding* je čisti GSM (Text-Mode) podešen sa `AT+CMGF=1` i zahteva da `CTRL+Z (ASCII 26 - Substitut)` potvdi odlazak paketa iz TX buffera. 

**Preklapanje Ekran-C (LVGL) i C++ senzora** 
Svi pozivi ka `ui_Label_setText()` za LCD displej MORAJU proći `example_lvgl_lock()` Semaphore pozive na `lvgl_mux` iz FreeRTOS-a zbog rigidne prirode frejmvorka. 

## Upravljanje Pametnim Napajanjem i Displejom (AXP2101)
`axp_get_batt_percent` implementiran u `PMU` petlji, a u kombinaciji sa kapacitivnim tasterom proverava statiku ruku na `CST92xx` staklu. Nakon 15s bez ikakvog prekida odozdo, MCU pauzira crtanje sa `esp_lcd_panel_disp_on_off` (AMOLED se "gasi" i resetuje, tako crna boja efektivno gasi pixel), a potom ukida signal na LCD Backlight-u za deep-sleep efekte ekrana.
