# LifeLink - Korisničko uputstvo (Uputstvo za upotrebu)

Dobrodošli u uputstvo za korišćenje vašeg pametnog sata **LifeLink**, kreiranog na moćnoj platformi ESP32-S3! Ovaj pametni sat ima okrugli stakleni AMOLED ekran na dodir i napravljen je prevashodno kao vaš čuvar u sferama zdravlja i bezbednosti. 

Zašto LifeLink štiti baš vas? Putem svojih moćnih ugrađenih senzora, on stalno "sluša" vaše srce, krvotok, ali i svaku promenu u brzini kojom se krećete do poda — sprečavajući lažne trzaje ruke, on garantuje da zaista zove pomoć kada padnete i povredite se.

---

## Osnovna Navigacija

Vaš ekran radi na bazi povlačenja prsta (**Swipe**):

1. **Glavni Početni Ekran (Prikaz Vremena i Parametara)**
   Kada otključate i dodirnete uređaj, stižete ovde. Ovaj ekran je dom i kontrolna tabla: 
   - Vaši podaci (HR / SpO2 u procentima), Otkucaji i Kiseonik, sve stoje ispred vašeg pogleda.
   - Iznad teksta nalaze se veoma male ikonice koje mogu promeniti boju u (Crvena, Narandžasta i Zelena). To su statusi: `GPS` konekcije (da li LifeLink "Zna" lokaciju npr. da niste u tunelu), status `GSM` antene (da li hvata signal) i slično.

2. **Ekran "Debug / Simulacija pada" (Prevlačenje na levo)**
   - Prevučite (Swipe) sa levog ruba sata ka centru prstom. 
   - Ovde se nalazi ikonica "Debug" stanica i ogromno dugme za aktiviranje "Simuliraj PAD". Iako je sistem za lažne padove odličan da ignoriše vaše obične pokrete, za ljude na radnom mestu idealno je pritisnuti dugme pre eksperimentisanja ili penjanja samo da aktivirate proces uzbune za svaki slučaj. 

3. **Ekran za unošenje "Kontakta Poverenja" (Prevlačenje ispod ili pored Glavnog)**
   - Kada nastavite kretanje kroz menije sata, naići čete na uveličanu tastaturu koja olakšava kucanje brojčanih kontakata čak i pacijentima. 
   - Prosto unesite broj koji hitne poruke i mapirani GPS (Google Maps) link do vas treba da stigne i stistnite (Sačuvaj). Broj biva smešten duboko u sat i bezbedan je od brisanja ukoliko ga ne prebrišete ponovo, i ostaće vezan čak i kad se isprazni baterija.

---

## Gledanje na Hitni Ekran (SOS)
Kada uređaj (ili simulirano vi) okinete pad – dešava se zloslutni **Crveni ekran "DA LI STE PALI?"**. 

Na ovom prozoru sat odbrojava 15 sekundi do katastrofe — poziva na akciju! Zašto 15 sekundi? Ako niste pali već ispustili sat, možete mirno prevući prst po crvenom ekranu jednom i on će odmah zaustaviti brojanje, prigušiti dramu i sat se vraća na bezbedan Glavni Početni ekran da meri dalje bez straha u okruženju! U suprotnom, poruke "POZIVANJE POMOĆI!!!" biće ispaljena automatski prekonfigureisanim brojevima na SIM mreži.  

Uživajte saznanjima da vas tehnologja ne posmatra već čuva.
