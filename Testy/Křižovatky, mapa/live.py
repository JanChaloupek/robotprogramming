from cely_projekt import Robot, Konstanty, Obrazovka

from microbit import button_a, sleep
from math import sin, cos

def lokalizuj_uhel(uhel, prikaz):
    # lokalizuj pozici po zmene smeru robota
    if prikaz == Konstanty.VLEVO:
        uhel += Konstanty.PI/2
        if uhel>Konstanty.PI:
            uhel -= 2*Konstanty.PI
    if prikaz == Konstanty.VPRAVO:
        uhel -= Konstanty.PI/2
        if uhel<=Konstanty.PI:
            uhel += 2*Konstanty.PI
    return uhel

def lokalizuj_xy(x, y, uhel, prikaz):
    # lokalizuj pozici po pohybu danym smerem
    if prikaz == Konstanty.VZAD:        # pokud byl posledni prikaz couvej, musis odecist cestu
        x -= cos(uhel)
        y -= sin(uhel)
    else:                               # v ostatnich pripadech jsme jeli rovne v zadanem smeru
        x += cos(uhel)
        y += sin(uhel)
    return x, y

def print_lokalizace(kde):
    print("lokalizace" + kde + ":", x, y, uhel / Konstanty.PI * 180)

if __name__ == "__main__":

    st_start = "start"
    st_jed_po_care = "jed_po_care"
    st_krizovatka = "krizovatka"
    st_reaguj_na_krizovatku = "reaguj_na_krizovatku"
    st_lokalizuj_po_prikazu = "lokalizuj_po_prikazu"
    st_exit = "exit"

    aktualni_stav = st_start

    x = 0
    y = 0
    uhel = 0

    prikazy = [Konstanty.ROVNE, Konstanty.VLEVO, Konstanty.ROVNE, Konstanty.ROVNE, Konstanty.VLEVO, Konstanty.ROVNE, Konstanty.ROVNE, ]
    index_prikazu = -1          # jsme pred prvnim prikazem (jedeme rovne i kdyz nam to nikdo vyslovne nerekl)

    while not button_a.was_pressed():

        if aktualni_stav == st_start:
            Obrazovka.pis(aktualni_stav, False)
            print_lokalizace("0")
            aktualni_stav = st_jed_po_care

        elif aktualni_stav == st_jed_po_care:
            Obrazovka.pis(aktualni_stav, False)
            aktualni_stav = st_krizovatka

        elif aktualni_stav == st_krizovatka:
            Obrazovka.pis(aktualni_stav, False)
            if index_prikazu < 0:
                prikaz = Konstanty.ROVNE
            else:
                prikaz = prikazy[index_prikazu]
            x, y = lokalizuj_xy(x, y, uhel, prikaz)
            print_lokalizace("1")
            aktualni_stav = st_reaguj_na_krizovatku

        elif aktualni_stav == st_reaguj_na_krizovatku:
            Obrazovka.pis(aktualni_stav, False)
            index_prikazu += 1
            if  index_prikazu < len(prikazy)-1:                  # jeste mame dalsi prikazy
                prikaz = prikazy[index_prikazu]
                print("prikaz:", prikaz)                         # proved nejak prikaz
                aktualni_stav = st_lokalizuj_po_prikazu
            else:
                print("Uz nemame zadne prikazy")
                aktualni_stav = st_exit

        elif aktualni_stav == st_lokalizuj_po_prikazu:
            Obrazovka.pis(aktualni_stav, False)
            prikaz = prikazy[index_prikazu]
            uhel = lokalizuj_uhel(uhel, prikaz)
            print_lokalizace("2")
            aktualni_stav = st_jed_po_care

        elif aktualni_stav == st_exit:
            Obrazovka.pis(aktualni_stav, False)
            break

        sleep(100)

