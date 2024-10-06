# Co je nové

V tomto souboru jsme rozšířili příklad z hodiny 15 o funkce:
- lokalizuj_xy
- lokalizuj_uhel

Následně jsme začali pracovat na integraci do existujícího stavového automatu a dostali jsme se k tomu, že by bylo dobré si statový automat rozšířit. To už jsme v hodině nestihli, a tak jsem to v kódu dodělala offline.

Přibyly stavy "st_lokalizuj_xy", "st_vypocti_uhel".

# Co se změnilo oproti hodině
- V hodině jsme se už dohodli, že příkazy vyjmeme z třídy Robot. Tedy jsem je teď umazala

- V hodině jsem měli výpis stavů pokaždé, kdy se pustil. Když program běžel, tak to každých 5 ms vypisovalo stav a bylo to nepřehledné. Přesunula jsem tedy výpis stavu jen když se změní. 

- Přesunula jsem funkci "reaguj_na_krizovatku" do souboru live.py, protože to je něco, co máte za úkol si napsat

- v "cely_projeky.py" jsem přejmenovala třídu Konstanty jen na "K", abychom ušetřili znaky

- Funkce Obrazovka.pis už nemá druhý argument True/False a vypisuje se zárověň do terminálu, tak na displej robota

# Jak otestovat

Když kód pustíme a robota si dáte na čáru, můžete sledovat, v jakém stavu se nachází. Když pak robot bude detekovat křižovatku, tak přejde do několika stavů lokalizace. To, že robot zareagoval na křižovatku, je zatím simulované jen tak, že musíte podržet tlačítko B na microbitu.
