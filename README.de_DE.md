[<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/a/ae/Flag_of_the_United_Kingdom.svg/100px-Flag_of_the_United_Kingdom.svg.png" height="30">](README.md)
[<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/a/a4/Flag_of_the_United_States.svg/100px-Flag_of_the_United_States.svg.png" height="30">](README.md)
[<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/ba/Flag_of_Germany.svg/100px-Flag_of_Germany.svg.png" height="30">](README.de_DE.md)

## Wir sind dauerhaft in die RF1000community umgezogen. 
Dort gibt es zwei Mod-Pakete: community_development und community_stable.  
Idealerweise nutzt ihr [community_stable](https://github.com/RF1000community/Repetier-Firmware).  
In community_development findet man den aktuellsten Mod, der sauber funktionieren sollte.
In community_stable findet man die Version, die als offizieller Mod gilt, eine community_development über welche länger niemand meckern konnte.

## Wenn du von Version from 1.37r oder früher updatest, mache bitte einen neuen M303 PID-Autotune bei allen Heizelementen!  
- RF2000/RF1000 Extruder 1: `M303 P0 X0 S230 R10 J1`
- RF2000 Extruder 2: `M303 P1 X0 S230 R10 J1`
- RF2000 Heizbett: `M303 P2 X0 S230 R15 J3`
- RF1000 Heizbett: `M303 P1 X0 S230 R15 J3`  
Starte mit EEPROM-Werten `PID drive min = 5` and `PID drive max = 100`  
Oder setze #define PID_CONTROL_DRIVE_MIN_LIMIT_FACTOR zu 10.0f in der configuration.h um die alte Version des PID Reglers zu nutzen.

## Bitte nutzt die Firmware von RF1000community!

# Hier ist mein persönlicher Testplatz.