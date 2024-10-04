from microbit import i2c, sleep
from microbit import pin14, pin15

from utime import ticks_us, ticks_diff

class Konstanty:
    NEDEFINOVANO = "nedefinovano"

    LEVY = "levy"
    PRAVY = "pravy"

    DOPREDU = "dopredu"
    DOZADU = "dozadu"

    ENKODER = "enkoder"
    PR_ENKODER = PRAVY + "_" + ENKODER
    LV_ENKODER = LEVY + "_" + ENKODER

    IR = "IR"
    PR_IR = PRAVY + "_" + IR
    LV_IR = LEVY + "_" + IR

    SENZOR_CARY = "senzor_cary"
    PR_S_CARY = PRAVY + "_" + SENZOR_CARY
    LV_S_CARY = LEVY + "_" + SENZOR_CARY
    PROS_S_CARY = "prostredni_" + SENZOR_CARY

    PI = 3.14159265359

class Senzory:

    def __init__(self, nova_verze=True, debug=False):
        self.nova_verze = nova_verze
        self.DEBUG = debug

    def precti_senzory(self):
        surova_data_byte = i2c.read(0x38, 1)
        if self.DEBUG:
            print("surova data", surova_data_byte)
        bitove_pole = self.__byte_na_bity(surova_data_byte)

        senzoricka_data = {}

        if not self.nova_verze:
            senzoricka_data[Konstanty.LV_ENKODER] = bitove_pole[9]
            senzoricka_data[Konstanty.PR_ENKODER] = bitove_pole[8] #TODO pretipuj taky, ale otestuj!

        senzoricka_data[Konstanty.LV_S_CARY] = bool(int(bitove_pole[7]))
        senzoricka_data[Konstanty.PROS_S_CARY] = bool(int(bitove_pole[6]))
        senzoricka_data[Konstanty.PR_S_CARY] = bool(int(bitove_pole[5]))
        senzoricka_data[Konstanty.LV_IR] = bool(bool(bitove_pole[4]))
        senzoricka_data[Konstanty.PR_IR] = bool(bool(bitove_pole[3]))

        return senzoricka_data

    def __byte_na_bity(self, data_bytes):

        data_int = int.from_bytes(data_bytes, "big")
        bit_pole_string = bin(data_int)

        if self.DEBUG:
            print("data_int", data_int)
            print("bit pole", bit_pole_string)

        return bit_pole_string

class Enkoder:

    def __init__(self, jmeno, perioda_rychlosti=1, nova_verze=True, debug=False):
        self.__jmeno = jmeno
        self.__perioda_rychlosti = perioda_rychlosti*1000000  # na us

        self.__nova_verze = nova_verze
        self.__tiky = 0
        self.__posledni_hodnota = -1
        self.__tiky_na_otocku = 40
        self.__nova_verze = nova_verze
        self.__DEBUG = debug
        self.__inicializovano = False
        self.__cas_posledni_rychlosti = ticks_us()
        self.__radiany_za_sekundu = 0

        if not self.__nova_verze:
            self.__senzory = Senzory(False, debug)

    def inicializuj(self):
        self.__posledni_hodnota = self.__aktualni_hodnota()
        self.__inicializovano = True

    def __aktualni_hodnota(self):
        if self.__nova_verze:
            if self.__jmeno == Konstanty.PR_ENKODER:
                return pin15.read_digital()
            elif self.__jmeno == Konstanty.LV_ENKODER:
                return pin14.read_digital()
            else:
                return -2
        else:
            senzoricka_data = self.__senzory.precti_senzory()

            if self.__jmeno == Konstanty.LV_ENKODER or self.__jmeno == Konstanty.PR_ENKODER:
                return int(senzoricka_data[self.__jmeno])
            else:
                return -2

    # drive se tato metoda jmenovala pocet_tiku
    def aktualizuj_se(self):
        if self.__DEBUG:
            print("v aktualizuj", self.__tiky)
        if self.__posledni_hodnota == -1:
            if self.__DEBUG:
                print("posledni_hodnota neni nastavena", self.__posledni_hodnota)
            return -1

        aktualni_enkoder = self.__aktualni_hodnota()
        if self.__DEBUG:
            print("aktualni enkoder", aktualni_enkoder)

        if aktualni_enkoder >= 0:  # nenastaly zadne chyby
            if self.__posledni_hodnota != aktualni_enkoder:
                self.__posledni_hodnota = aktualni_enkoder
                self.__tiky += 1
        else:
            return aktualni_enkoder

        return 0

    def __us_na_s(self, cas):
        return cas/1000000

    def vypocti_rychlost(self):
        cas_ted = ticks_us()
        interval_us = ticks_diff(cas_ted, self.__cas_posledni_rychlosti)
        if interval_us >= self.__perioda_rychlosti:
            interval_s = self.__us_na_s(interval_us)
            otacky = self.__tiky/self.__tiky_na_otocku
            radiany = otacky * 2 * Konstanty.PI
            self.__radiany_za_sekundu = radiany / interval_s
            self.__tiky = 0
            self.__cas_posledni_rychlosti = cas_ted

        return self.__radiany_za_sekundu

class Motor:
    def __init__(self, jmeno, prumer_kola, nova_verze=True, debug=False):
        if jmeno == Konstanty.LEVY:
            self.__kanal_dopredu = b"\x05"
            self.__kanal_dozadu = b"\x04"
        elif jmeno == Konstanty.PRAVY:
            self.__kanal_dopredu = b"\x03"
            self.__kanal_dozadu = b"\x02"
        else:
            raise AttributeError("spatne jmeno motoru, musi byt \"levy\" a nebo \"pravy\", zadane jmeno je" + str(jmeno))

        self.__DEBUG = debug
        self.__jmeno = jmeno
        self.__prumer_kola = prumer_kola
        self.__enkoder = Enkoder(jmeno + "_enkoder", 1, nova_verze, debug)
        self.__smer = Konstanty.NEDEFINOVANO
        self.__inicializovano = False
        self.__rychlost_byla_zadana = False
        self.__min_pwm = 0
        self.__perioda_regulace = 1000000 #v microsekundach
        self.__cas_posledni_regulace = 0
        self.aktualni_rychlost = 0

    def inicializuj(self):
        # self.enkoder.inicializuj()
        # probud cip motoru
        i2c.write(0x70, b"\x00\x01")
        i2c.write(0x70, b"\xE8\xAA")

        self.__enkoder.inicializuj()

        if self.__jmeno == Konstanty.LEVY:
            self.__limity = [4.705, 12.37498]
            self.__primky_par_a = [17.07965]
            self.__primky_par_b = [28.63963]
        else:
            self.__limity = [4.520269, 8.57154]
            self.__primky_par_a = [20.98109]
            self.__primky_par_b = [60.15985]

        self.__inicializovano = True

        self.__cas_posledni_regulace = ticks_us()

    def jed_doprednou_rychlosti(self, v: float):
        """
        Rozjede motor pozadovanou doprednou rychlosti
        """
        if not self.__inicializovano:
            return -1

        self.__pozadovana_uhlova_r_kola = self.__dopredna_na_uhlovou(v)
        if self.__DEBUG:
            print("pozadovana uhlova", self.__pozadovana_uhlova_r_kola)

        self.__rychlost_byla_zadana = True

        # pouziji funkci abs pro vypocteni absolutni hodnoty
        # PWM je vzdy pozitivni
        # znamenko uhlove rychlosti ovlivni smer
        prvni_PWM = self.__uhlova_na_PWM(abs(self.__pozadovana_uhlova_r_kola))
        if self.__DEBUG:
            print("prvni_PWM", prvni_PWM)

        if self.__pozadovana_uhlova_r_kola > 0:
            self.__smer = Konstanty.DOPREDU
        elif self.__pozadovana_uhlova_r_kola < 0:
            self.__smer = Konstanty.DOZADU
        else: # = 0
            self.__smer == Konstanty.NEDEFINOVANO

        return self.__jed_PWM(prvni_PWM)


    #lekce 8, slidy 8 - 11
    def __dopredna_na_uhlovou(self, v: float):
        """
        Prepocita doprednou rychlost kola na uhlovou
        """
        return v/(self.__prumer_kola/2)

    def __uhlova_na_PWM(self, uhlova):
        """
        Prepocte uhlovou rychlost na PWM s vyuzitim dat z kalibrace
        """

        a, b = self.__najdi_spravne_parametery(uhlova)
        return int(a*uhlova + b)

    def __najdi_spravne_parametery(self, uhlova):
        # TODO
        return self.__primky_par_a[0], self.__primky_par_b[0]

    # DU 5 pokrocily/DU 7 zacatecnici
    # jed(motor, smer, rychlost)
    def __jed_PWM(self, PWM):
        je_vse_ok = -2
        if self.__smer == Konstanty.DOPREDU:
            je_vse_ok  = self.__nastav_PWM_kanaly(self.__kanal_dopredu, self.__kanal_dozadu, PWM)
        elif self.__smer == Konstanty.DOZADU:
            je_vse_ok  = self.__nastav_PWM_kanaly(self.__kanal_dozadu, self.__kanal_dopredu, PWM)
        elif self.__smer == Konstanty.NEDEFINOVANO:
            if PWM == 0:
                je_vse_ok = self.__nastav_PWM_kanaly(self.__kanal_dozadu, self.__kanal_dopredu, PWM)
            else:
                je_vse_ok = -1
        else:
            je_vse_ok = -3

        return je_vse_ok

    def __nastav_PWM_kanaly(self, kanal_on, kanal_off, PWM):
        # TODO zkontroluj, ze motor byl inicializovan
        i2c.write(0x70, kanal_off + bytes([0]))
        i2c.write(0x70, kanal_on + bytes([PWM]))
        self.__PWM = PWM
        return 0

    def aktualizuj_se(self):
        self.__enkoder.aktualizuj_se()
        cas_ted = ticks_us()
        cas_rozdil = ticks_diff(cas_ted, self.__cas_posledni_regulace)
        navratova_hodnota = 0
        if cas_rozdil > self.__perioda_regulace:
            navratova_hodnota = self.__reguluj_otacky()
            self.__cas_posledni_regulace = cas_ted

        return navratova_hodnota

    # ukazka v pridanem materialu k hodine 8
    def __reguluj_otacky(self):

        if not self.__inicializovano:
            return -1

        if not self.__rychlost_byla_zadana:
            return -2

        P = 6

        self.aktualni_rychlost = self.__enkoder.vypocti_rychlost()
        # aktualni_rychlost bude vzdy pozitivni
        # musim tedy kombinovat se smerem, kterym se pohybuji
        if self.__pozadovana_uhlova_r_kola < 0:
            self.aktualni_rychlost *= -1

        error = self.__pozadovana_uhlova_r_kola - self.aktualni_rychlost
        akcni_zasah = P*error
        return self.__zmen_PWM_o(akcni_zasah)

    def __zmen_PWM_o(self, akcni_zasah):
        # error a tim padem i akcni_zasah muze byt jak pozitivni tak negativni, nezavisle na smeru
        # priklad, pozadovana = 5, aktualni = 2
        # smer = dopredu a error > 0 => musim zrychlit
        # priklad, pozadovana 5, aktualni 10
        # smer = dopredu a error < 0 => musim zpomalit
        # priklad, pozadovana -5, aktualni -2
        # smer = dozadu a error < 0 => musim zrychlist
        # priklad, pozadovana -5, aktualni -7
        # smer= dozadu a error>0 => musim zpomalit

        akcni_zasah = int(akcni_zasah) #PWM je v celych cislech

        if self.__smer == Konstanty.DOZADU:
            # prohod logiku aby pozitivni akcni_zasah u jizdy dozadu
            # take znamenal zrychli a ne zpomal, viz priklad nahore
            akcni_zasah *= -1

        nove_PWM = self.__PWM + akcni_zasah

        if nove_PWM > 255:
            nove_PWM = 255

        # toto nastane ve chvili, kdy by akcni_zasah zpomaleni byl
        # vetsi nez aktualni rychlost
        # myslenka je, ze chceme zpomalit co nejvice muzeme, tedy v nasem pripade zastavit
        # rozjet motor na opacnou stranu za me z pohledu rizeni rychlosti otaceni nedava smysl
        # ale muzeme se o tom pobavit :)

        if nove_PWM < 0:
            nove_PWM  = 0

        return self.__jed_PWM(nove_PWM)

class Robot:

    def __init__(self, rozchod_kol: float, prumer_kola: float, nova_verze=True):
        """
        Konstruktor tridy
        """
        self.__d = rozchod_kol/2
        self.__prumer_kola = prumer_kola

        self.__levy_motor = Motor(Konstanty.LEVY, self.__prumer_kola, nova_verze)
        self.__pravy_motor = Motor(Konstanty.PRAVY, self.__prumer_kola, nova_verze)
        self.__inicializovano = False
        self.__cas_minule_reg = ticks_us()
        self.__perioda_regulace = 1000000

    def inicializuj(self):
        i2c.init(400000)
        self.__levy_motor.inicializuj()
        self.__pravy_motor.inicializuj()
        self.__inicializovano = True

    # pokrocily ukol 7
    def jed(self, dopredna_rychlost: float, uhlova_rychlost: float):
        """Pohybuj se zadanym  pohybem slozenym z dopredne rychlosti v a uhlove rychlosti"""

        if not self.__inicializovano:
            return -1

        self.__dopredna_rychlost = dopredna_rychlost
        self.__uhlova_rychlost = uhlova_rychlost
        # kinematika diferencionalniho podvozku - lekce 7
        dopr_rychlost_leve = dopredna_rychlost - self.__d * uhlova_rychlost
        dopr_rychlost_prave = dopredna_rychlost + self.__d * uhlova_rychlost

        # nevolam povely i2c rovnou - to bych rozbijela zapouzdreni trid
        # vyuziji funkce tridy motor
        self.__levy_motor.jed_doprednou_rychlosti(dopr_rychlost_leve)
        self.__pravy_motor.jed_doprednou_rychlosti(dopr_rychlost_prave)

        return 0

    def __aktualni_rychlost(self):
        levy_r = self.__levy_motor.aktualni_rychlost * self.__prumer_kola/2
        pravy_r = self.__pravy_motor.aktualni_rychlost * self.__prumer_kola/2


        omega = (pravy_r - levy_r)/ (2 * self.__d)
        v = levy_r + self.__d * omega
        print("aktualizuju", levy_r, pravy_r, v, omega)
        return v, omega

    def aktualizuj_se(self):
        self.__levy_motor.aktualizuj_se()
        self.__pravy_motor.aktualizuj_se()

        if ticks_diff(ticks_us(), self.__cas_minule_reg) > self.__perioda_regulace:
            self.__cas_minule_reg = ticks_us()
            #self.__reguluj()

    def __reguluj(self):

        v, omega = self.__aktualni_rychlost()
        error_omega = self.__uhlova_rychlost - omega
        error_v = self.__dopredna_rychlost - v

        P_om = 1
        P_v = 1
        u_om = P_om * error_omega
        u_v = P_v * error_v
        print("reguluju", u_v, u_om)
        self.jed(u_v, u_om)
