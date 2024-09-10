from microbit import sleep
from utime import ticks_ms, ticks_us
#from time import sleep, monotonic, perf_counter_ns, process_time_ns


#def ticks_ms():
#    return perf_counter_ns() // 1_000_000

#def ticks_us():
#    return perf_counter_ns() // 1_000

TESTS = 500
data0 = [0] * TESTS
data1 = [0] * TESTS
data2 = [0] * TESTS
data3 = [0] * TESTS
data4 = [0] * TESTS
dataIndex = 0

class SpeedTicks:
    LIMIT = 100                                             # pocet desetin pro ktere si pamatujeme hodnoty

    def getTimes(self):
        return self.__times

    def getTicks(self):
        return self.__ticks


    def __init__(self):
        self.__countValues = -1                             # ani prvni hodnotu nechci brat jako správnou (bude ignorovana)
        self.__times = [0] * self.LIMIT
        self.__ticks = [0] * self.LIMIT
        self.__index = -1                                   # prvni hodnota bude ulozena do indexu 0 (coz je -1 + 1)
        self.__lastTime = -1

    def getNewIndex(self, time: int):                       # Zjisti z casu jestli uz muzeme ulozit data do dalsiho indexu
        newTime = int(time / 10000)                         # cas na setiny sekundy
        if newTime == self.__lastTime:                      # je stejny jako naposledy ulozeny?
            return -1                                       # ano -> jeste nebudeme ukladat
        else:
            self.__lastTime = newTime                       # ne  -> tak uz musime ulozit
            return (self.__index + 1) % self.LIMIT          #        vratime o 1 vetsi index nez minule (modulo LIMIT = cyklicky seznam)

    def nextValues(self, newIndex, time, ticks):            # Ulozime nove hodnoty do pole
        if self.__countValues < self.LIMIT:                 # jeste nemame vsechny hodnoty pole zaplnene?
            self.__countValues += 1                         # ano -> pricteme ze mame dalsi hodnotu
        self.__index = newIndex                             # zapamatujeme si novy index
        self.__times[newIndex] = time                   #  ulozime cas do tohoto indexu
        self.__ticks[newIndex] = ticks
#        print("debug-nextValues: ", self.__index, self.__countValues, time, ticks)

    def update(self, ticks, time):                          # Nech nas pracovat. Musime volat dostatecne casto (minimalne jednou za desetinu sekundy)
        # time = ticks_us()                                   # precteme cas v us
        newIndex = self.getNewIndex(time)                   # ziskej novy index (pokud uz uplynulo dost casu)
        if newIndex >= 0:                                   # mame novy index?
            self.nextValues(newIndex, time, ticks)          # ano -> ulozime nove hodnoty pro tento index

    def calculate(self, count=10, offset=0):
        if count < 2:                                       # osetri nevhodny pocet dat z kterych mame pocitat rychlost
            count = 10                                      # musime to spocitat alespon ze 2 hodnot (pokud je mensi, spravime si to na defaultni hodnotu)

        if count >= (self.__countValues - offset):          # jeste musime zkontrolovat jestli uz mame dost namerenych honot na to co pozadujeme
            count = self.__countValues - offset - 1         # nemame -> orizneme pocet na mensi hodnotu

        if count < 2:                                       # i po oriznuti musime mit alespon 2 hodnoty
#            print("debug-calculate: malo hodnot ", self.__countValues, count, offset)
            return 0                                        # nemame -> predpokladame ze na zacatku stojime (je nulova rychlost)

        speed0 = self.__calculate(count, offset)            # spocteme rychlost z pozadovanych dat
        speed1 = self.__calculate(count, offset+1)          # spocteme rychlost s o 1 starsich dat
        speed = (speed0 + speed1) / 2                       # prumer z techto 2 hodnot
#        print("debug-calculate: speed - ", speed, speed0, speed1)
        return  speed

    def __calculate(self, count=10, offset=0):              # Spocti rychlost tiku (tick/s), pouzij k tomu data dlouha count (setin sekundy) a posun se do minulosti o offset (desetinsekundy)
        global data1, data2, dataIndex

        data0[dataIndex] = self.__countValues * 1000
        data0[dataIndex] += count

        endIndex = (self.__index - offset) % self.LIMIT     # koncovy index pro data (kde konci casove okno)
        startIndex = (endIndex - count + 1) % self.LIMIT    # spocteme pocatecni index (pro data pred count setinami)

        diffTimes = self.__times[endIndex] - self.__times[startIndex] # spocti rozdil tiku za cca pocet setin
        diffTicks = self.__ticks[endIndex] - self.__ticks[startIndex] # spocti rozdil casu za cca pocet setin

#        print("debug-calculate: ", count, offset, endIndex, startIndex, self.__countValues)
        data1[dataIndex] = diffTimes
        data2[dataIndex] = diffTicks

        return 1_000_000 * diffTicks / diffTimes              # rychlost = pocet tiku za 1s (tj. za 1_000_000 us)

def main():
    global data1, data2, data3, data4, dataIndex
    speedTicks = SpeedTicks()
    while dataIndex < TESTS:
        acttime = ticks_us()
        speedTicks.update(int(acttime/10_000), acttime)
        data3[dataIndex] = speedTicks.calculate(30)
        endtime = ticks_us()
        data4[dataIndex] = endtime-acttime #ticks_ms()
        dataIndex += 1
        sleep(3)
#        sleep(3/1000)

    print("-data----------")
    for i in range(TESTS):
        print(data0[i], data1[i], data2[i], data3[i], data4[i])

#    print("-measure------")
#    # Open the file in write mode (or append mode if you don't want to overwrite)
#    with open('outputtesttime.txt', 'w') as file:
#        file.write("cokoli3\n")
#        for i in range(speedTicks.LIMIT):  # Assuming speedTicks.LIMIT defines the loop range
#            # Write the formatted string to the file
#            file.write(f"{speedTicks.getTimes()[i]} {speedTicks.getTicks()[i]}\n")

if __name__ == "__main__":
    main()
