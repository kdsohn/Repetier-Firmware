T0
M140 S100 ;Bettemperatur des Tests (Offset ~ -0.0015 mm/°C)
M104 S230 ;Bettemperatur des Tests (Offset ~ -0.001 mm/°C)

M116 ;Warte auf Heizung/Kühlung +-2°C

;;;; Alt: Wurde durch M3900 R1, das Auto-Matrix-Leveling ersetzt.
;;;;    ;Heizbett in Mitte auf korrekte Höhe leveln:
;;;;    M3013 P1
;;;;    M3900 X5 Y7 S100 P0 ;messen Mitte
;;;;    M3900 X6 Y6 S50 P0 ;korrektiv nachmessen Mitte
;;;;    M3900 X7 Y5 S50 P0 ;korrektiv nachmessen Mitte
;;;;    M3013 P1

;;;;    ;Heizbett Ecken Autoleveling/Lagekorrektur:
;;;;    M3900 X2 Y2 S90 P100 ;Ecke verbiegen vorne links
;;;;    M3900 X9 Y2 ;Ecke verbiegen vorne rechts
;;;;    M3900 X2 Y12 ;Ecke verbiegen hinten links
;;;;    M3900 X9 Y12 ;Ecke verbiegen hinten rechts

;Neu: [R1 = Auto-Matrix-Leveling] ersetzt das hier drüber.
; Kann im Druckermenü oder per G-Code gestartet werden. Sollte mit Temperatur gemacht werden.
M3900 R1 
M3013 P1

M104 T0 S0 ;Heizungen alle aus
M104 T1 S0 ;Heizungen alle aus
M140 S0 ;Heizungen alle aus
M84 ;Schrittmotoren aus

; optional: Speichern der Matrix auf Position 3
;M3902 S3
; optional: Auswählen der korrigierten Matrix auf Position 3
;M3009 S3
; optional: Ausgeben der Matrix auf Position 3 in Millimetern
;M3013 P1

M300 S440 P500 ;Piepton

