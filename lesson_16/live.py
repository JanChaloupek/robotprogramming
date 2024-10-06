from cely_projekt import Robot, K, Obrazovka

from microbit import button_a, button_b, sleep
from math import sin, cos

def lokalizuj_xy(x, y, uhel, prikaz):

    if prikaz == K.VZAD:
        x_nove = cos(uhel) * (-1) + x
        y_nove = sin(uhel) * (-1) + y
    else:
        x_nove = cos(uhel) * 1 + x
        y_nove = sin(uhel) * 1 + y

    return x_nove, y_nove

def lokalizuj_uhel(uhel, prikaz):
    uhel_novy = uhel

    if prikaz == K.VLEVO:
        uhel_novy = uhel + K.PI/2
    elif prikaz == K.VPRAVO:
        uhel_novy = uhel - K.PI/2

    return uhel_novy

def reaguj_na_krizovatku(prikaz):
    # vas ukol
    if button_b.is_pressed(): # jen neco, abych simulovala funkci
        return True
    else:
        return False

if __name__ == "__main__":

    prikazy = [K.ROVNE, K.VLEVO]
    index_prikazu = 0

    robot = Robot(0.15, 0.067, False)

    st_start = "start"
    st_jed_po_care = "jed_po_care"
    st_lokalizuj_xy = "lokalizuj_xy"
    st_krizovatka = "krizovatka"
    st_vypocti_uhel = "vypocti_uhel"
    st_exit = "exit"

    aktualni_stav = st_start
    Obrazovka.pis(aktualni_stav)

    x = 0
    y = 0
    uhel = 0

    while not button_a.was_pressed():

        if aktualni_stav == st_start:
            if robot.inicializuj():
                aktualni_stav = st_jed_po_care
                Obrazovka.pis(aktualni_stav)

        elif aktualni_stav == st_jed_po_care:
            robot.jed_po_care(0.1, 0.5)
            situace = robot.vycti_senzory_cary()

            if situace == K.KRIZOVATKA:
                aktualni_stav = st_lokalizuj_xy
                Obrazovka.pis(aktualni_stav)
            elif situace == K.CARA:
                aktualni_stav = st_jed_po_care
            elif situace == K.ZTRACEN:
                aktualni_stav = st_exit
                Obrazovka.pis(aktualni_stav)

        elif aktualni_stav == st_lokalizuj_xy:
            minuly_prikaz = ""
            if index_prikazu == 0:
                minuly_prikaz = K.ROVNE
            else:
                minuly_prikaz = prikazy[index_prikazu]

            x, y = lokalizuj_xy(x, y, uhel, minuly_prikaz)
            aktualni_stav = st_krizovatka
            Obrazovka.pis(aktualni_stav)

        elif aktualni_stav == st_krizovatka:
            # funkce reaguj_na_krizovatku je vas domaci ukol
            zaregoval = reaguj_na_krizovatku(prikazy[index_prikazu])
            if zaregoval:
                aktualni_stav = st_vypocti_uhel
                Obrazovka.pis(aktualni_stav)

        elif aktualni_stav == st_vypocti_uhel:
            uhel = lokalizuj_uhel(uhel, prikazy[index_prikazu])
            index_prikazu += 1
            if index_prikazu < len(prikazy):
                aktualni_stav = st_jed_po_care
                Obrazovka.pis(aktualni_stav)
            else:
                aktualni_stav = st_exit
                Obrazovka.pis(aktualni_stav)

        elif aktualni_stav == st_exit:
            robot.jed(0, 0)
            break

        robot.aktualizuj_se()
        sleep(5)


