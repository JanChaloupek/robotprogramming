# Co je nové

V hodině jsem se seznámili se stavovými automaty a v rámci živé ukázky jsme programovali jednoduchý statový automat za pomocí if-elif-else. 

Využili jsme nových funkcí z cely_projekt.py:
- Robot.vycti_senzory_cary
- Robot.jed_po_care
- Obrazovka.pis

Také do cely_projekt.py přibylo jedno možné řešení, jak implementovat světla a to pomocí dědičnosti.

# Co se změnilo oproti hodině

- místo falešného robot.reaguje_na_krizovatku je zanecháno robot.jed(0,0)

# Jak otestovat

Robot pojede po čáře a bude vypisovat stav než narazí na křižovatku, kdy pak přejde do stavu st_krizovatka a náslědně do st_exit.

