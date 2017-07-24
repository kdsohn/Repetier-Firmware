[<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/a/ae/Flag_of_the_United_Kingdom.svg/100px-Flag_of_the_United_Kingdom.svg.png" height="30">](README.md)
[<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/a/a4/Flag_of_the_United_States.svg/100px-Flag_of_the_United_States.svg.png" height="30">](README.md)
[<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/ba/Flag_of_Germany.svg/100px-Flag_of_Germany.svg.png" height="30">](README.de_DE.md)

## We permanently moved to the RF1000community. 
There are now two Branches: community_development and community_stable.  
You should use [community_stable](https://github.com/RF1000community/Repetier-Firmware).  
In community_development you find the released version of the mod which should be working good.
In community_stable you find a bit older version about which nobody complained for a long time.

## If you upgrade to this Version from 1.37r or earlier please do a fresh M303 PID-Autotune on all Heaters!  
- RF2000/RF1000 extruder 1: `M303 P0 X0 S230 R10 J1`
- RF2000 extruder 2: `M303 P1 X0 S230 R10 J1`
- RF2000 heated bed: `M303 P2 X0 S70 R15 J3`
- RF1000 heated bed: `M303 P1 X0 S70 R15 J3`  
And start with EEPROM-values `PID drive min = 5` and `PID drive max = 100`  
Or set #define PID_CONTROL_DRIVE_MIN_LIMIT_FACTOR to 10.0f in configuration.h to use the old version PID-control.

## Please look for and download the Firmware from RF1000community!

# This is my personal Testingspace!
