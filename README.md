# Energy_Counter
Programm fuer ESP32 zur Auslesung der Energiewerte meines Hauses:

- Strom: Nutzung des SML-Protokolls der Stromwerte aus meinem ISKRA MT681 Zaehlers 

- Gas: Ablesung des silbernen Punktes bei der Zahl 6 auf dem ITRON Gaszaehlers mit Hilfe einer erweiterten TCRT5000 Lichtschranke

- Wasser: Ablesung des drehenden Rades eines Standardwasserzaehlers mit Hilfe einer erweiterten TCRT5000 Lichtschranke

Umbau TCRT5000 mit 2 IR-Dioden zur Erkennung des kompletten Durchlaufs eines Punktes am Gas- oder Wasserzaehler :

![TCRT5000 doppelt](https://user-images.githubusercontent.com/76279852/213930673-a9e1f557-3765-40b4-a3f6-ac5b35fe5bd1.jpg)
