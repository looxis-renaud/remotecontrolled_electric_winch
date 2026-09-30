# Motor-Sensoren testen: Temperaturfühler und 3 Hall-Sensoren

> Ausnahmsweise auf Deutsch. Gilt für den QS Motor 12kW 260 V4 der Winde mit 6-poligem Sensorstecker (3 Hall-Sensoren + Temperaturfühler). Hintergrund und Einbindung im VESC: [vesc/readme.md](../../vesc/readme.md) ("Background: sensorless vs. hall sensors", "Recommendation: connect and use the hall sensors") und [WINCH-17](../../features/WINCH-17-hall-sensors-foc.md).

Benötigt: Multimeter, Labornetzteil, 3 Widerstände (4,7 bis 10 kΩ), am besten auch 3 LEDs mit je 1 kΩ Vorwiderstand.

## Pinbelegung

Der Motor hat **zwei Sensorsätze** (je 3 Hall-Sensoren + Temperaturfühler), einer davon ist Reserve. In Benutzung ist **Satz 1** mit selbst angebrachtem Stecker. Satz 2 hat noch den Original-QS-Stecker, war nie angeschlossen und ist ungeprüft.

Belegung Satz 1 (von Etienne bestätigt, 2026-09-30):

| Kabelfarbe am Motor | Kabelfarbe am Stecker | Funktion |
|---|---|---|
| Gelb | Gelb | Hall A |
| Grün | Weiß | Hall B |
| Blau | Rot | Hall C |
| Schwarz | Schwarz | GND für Temperaturfühler und Hall (gemeinsamer Minuspol) |
| Rot | Grün | +5 V (Hall-Versorgung) |
| Silber (im QS-Handbuch "transparent") | Blau | Temperaturfühler |

⚠ **Die Farben am Stecker passen nicht zu den Farben am Motor.** Rot am Stecker ist ein Hall-Signal, Grün am Stecker ist +5 V. Die Farbangaben in den Schritten unten beziehen sich auf die **Kabelfarben am Motor**. Wer am Stecker misst, muss sie über die Tabelle übersetzen.

Weitere Motordaten: [README.md](README.md).

## Vorbereitung

- **VESC ausschalten und Batterie abklemmen.** Dann den Motor komplett vom VESC trennen: Sensorstecker abziehen, die drei dicken Phasenkabel abstecken. Die Phasenkabel dürfen sich nicht berühren und nicht offen herumliegen.
- Die Winde sichern, sodass sich die Trommel gefahrlos von Hand drehen lässt (Seil entlastet). Beim Nabenmotor dreht sich das Gehäuse mit der Trommel, die Achse steht fest.
- Die drei Widerstände (4,7 bis 10 kΩ) werden als Pull-ups benötigt. Die Hall-Ausgänge sind meist Open-Collector, also ohne eigenen Pull-up. Ohne Widerstand lässt sich der High-Zustand nicht zuverlässig messen.
- Klemmleisten, Krokoklemmen oder Dupont-Stecker erleichtern das Greifen der einzelnen Kabel.

## Schritt 1: Kurzschluss- und Isolationsprüfung (stromlos)

Multimeter auf Widerstand oder Durchgang stellen:

- Zwischen je zwei der 6 Pins darf kein Kurzschluss sein. Ausnahme: Zwischen Rot und Schwarz kann ein endlicher Widerstand oder ein Diodenverhalten auftreten, das ist normal.
- Zwischen jedem Pin und dem Motorgehäuse sowie den Phasenkabeln sollte der Widerstand praktisch unendlich sein (OL).

## Schritt 2: Temperaturfühler prüfen

Nicht mit dem Netzteil speisen, das erwärmt den Sensor und kann ihn beschädigen. Multimeter auf Widerstand stellen und zwischen Schwarz und dem Kandidaten-Pin messen. Der richtige Pin zeigt einen Widerstand im Bereich von etwa 0,3 bis 100 kΩ.

Bei Raumtemperatur (ca. 25 °C) verrät der Wert den Sensortyp:

| Messwert bei ca. 25 °C | Sensortyp | Verhalten beim Erwärmen |
|---|---|---|
| ca. 10 kΩ | NTC 10k (meist B3380/B3950) | Widerstand sinkt |
| ca. 1,0 kΩ | KTY83-122 | Widerstand steigt (ca. 0,8 % pro °C) |
| ca. 1,1 kΩ | PT1000 | Widerstand steigt (ca. 0,4 % pro °C) |
| ca. 0,6 kΩ | KTY84-130 | Widerstand steigt |

KTY83 und PT1000 liegen bei Raumtemperatur nah beieinander. Unterscheiden lassen sie sich über die Änderung beim Erwärmen (KTY83 steigt etwa doppelt so stark) oder über das Datenblatt / die Bestellangaben von QS.

**Messung an der Winde (Etienne, 2026-09-30):** 0,97 kΩ bei ca. 25 °C (Satz 1, Silber/Blau gegen Schwarz). Das passt zum KTY83-122, den auch das QS-Handbuch nennt. Der Erwärmungstest steht noch aus.

Zur Kontrolle den Motor erwärmen, z. B. mit einem Föhn auf das Gehäuse. Weil der Fühler in der Wicklung sitzt, reagiert er träge. Richtwerte für einen 10k-NTC (B3950): 0 °C ≈ 33 kΩ, 40 °C ≈ 5,3 kΩ, 60 °C ≈ 2,5 kΩ.

Ein Fehler liegt vor bei:

- OL oder unendlichem Widerstand (Fühler oder Kabel unterbrochen),
- nahe 0 Ω (Kurzschluss),
- einem Wert, der sich beim Erwärmen nicht ändert.

Den Sensortyp brauchst du später im VESC Tool: Motor Settings, Einstellung `m_motor_temp_sens_type`. Zu finden unter Motor Settings → General → Advanced → Motor Temperature Sensor Type. **An der Winde ist seit 2026-09-30 KTY83/122 eingestellt** (vorher NTC 10k, was einen dauerhaften Übertemperatur-Fehler ausgelöst hat), siehe [WINCH-17](../../features/WINCH-17-hall-sensors-foc.md).

## Schritt 3: Hall-Sensoren prüfen

### Aufbau

1. Netzteil auf 5,0 V einstellen (übliche VESC-Sensorversorgung) und die Strombegrenzung auf ca. 30 mA setzen.
2. Plus an Rot, Minus an Schwarz anschließen.
3. Je einen Widerstand (4,7 bis 10 kΩ) zwischen Rot und jeden Signalpin (Blau, Grün, Gelb) klemmen oder löten.
4. Netzteil einschalten. Für drei Sensoren sind grob 10 bis 30 mA normal. Geht das Netzteil sofort in die Strombegrenzung, sofort abschalten und Polarität und Kurzschlüsse prüfen.

### Funktionstest pro Sensor

Multimeter auf DC-Spannung, schwarze Prüfspitze an Schwarz (GND), rote Prüfspitze nacheinander an Blau, Grün, Gelb. Dabei die Trommel langsam von Hand drehen.

- Jeder Kanal muss zwischen Low (ca. 0 bis 0,5 V) und High (ca. 4,5 bis 5 V) umschalten.
- Pro Umdrehung schaltet jeder Kanal so oft komplett durch, wie der Motor Polpaare hat. **Der QS-Motor hat 32 Magnete = 16 Polpaare, also 16 volle Zyklen pro Umdrehung** (laut Config `si_motor_poles = 32`).
- Bleibt ein Kanal immer Low, immer High oder wackelt er unruhig, ist der Sensor oder seine Leitung defekt.

### Phasenlage prüfen

Die drei Signale müssen zueinander um 120° elektrisch versetzt sein, sonst funktioniert die Kommutierung nicht.

Bei 16 Polpaaren gibt es 16 × 6 = 96 Zustandswechsel pro Umdrehung, also etwa alle 3,75°. Von Hand ist das fein. **Am einfachsten geht es deshalb mit drei LEDs:** je eine LED mit 1 kΩ Vorwiderstand zwischen Rot (+5 V) und den Signalpin schalten. Die LED leuchtet, wenn der Ausgang Low ist. So siehst du alle drei Kanäle gleichzeitig.

Vorgehen:

1. Die Trommel in kleinen Schritten drehen und bei jeder Position alle drei Kanäle ablesen (0 = Low, 1 = High). Eine Markierung mit Klebeband auf der Trommel hilft bei der Orientierung.
2. Die Zustände in eine Tabelle eintragen.
3. Zwei Regeln prüfen:
   - Der Zustand 000 oder 111 darf nie vorkommen.
   - Zwischen zwei aufeinanderfolgenden Zuständen ändert sich immer genau ein Bit.

Eine gültige Sequenz sieht z. B. so aus: 001 → 011 → 010 → 110 → 100 → 101 → wieder 001. Reihenfolge, Richtung oder Bitzuordnung dürfen anders sein, entscheidend sind die zwei Regeln.

## Schritt 4: Abschlusstest am VESC

Sind die Messungen in Ordnung, den Sensorstecker am Sensor-Port des VESC anschließen. Achtung: Das Trampa-Handbuch im Repo beschreibt die neuere MKVI-Revision. Lage und Belegung des Ports am eingebauten Controller vor Ort prüfen (siehe [vesc/readme.md](../../vesc/readme.md)).

Dann im VESC Tool die Hall-Erkennung durchführen, wie in [vesc/readme.md](../../vesc/readme.md) unter "Recommendation: connect and use the hall sensors" beschrieben. **Der Motor dreht sich dabei selbst: Seil entlastet, niemand an der Trommel.**

- Eine erfolgreiche Erkennung erzeugt die Hall-Tabelle: Einträge 1 bis 6 haben Werte, nur 0 und 7 stehen auf 255.
- Die Zuordnung der drei Signalkabel zu H1, H2 und H3 ist unkritisch, weil die Erkennung die Reihenfolge selbst ermittelt.
- In den Echtzeitdaten die Motortemperatur prüfen: Bei kaltem Motor muss sie etwa der Umgebungstemperatur entsprechen.

## Fehlersuche

| Symptom | Wahrscheinliche Ursache |
|---|---|
| Netzteil geht in die Strombegrenzung | Rot/Schwarz vertauscht oder Kurzschluss im Kabel |
| Ein Kanal bleibt konstant | Sensor oder Leitung defekt, oder Pull-up fehlt |
| Alle Kanäle schwingen ungleichmäßig oder rauschen | Wackelkontakt im Stecker oder fehlender Pull-up |
| Zustand 000 oder 111 tritt auf | Sensor defekt oder falsch platziert |
| Temperaturwert OL oder ändert sich nicht | Fühlerleitung unterbrochen oder falscher Pin |
| Temperatur im VESC Tool unplausibel, Messung aber in Ordnung | Falscher Sensortyp (`m_motor_temp_sens_type`) eingestellt |

## Messprotokoll

Ergebnisse bitte mit Datum in [WINCH-17](../../features/WINCH-17-hall-sensors-foc.md) eintragen (Temperaturfühler: Pin, Messwert, Temperatur, Sensortyp; Hall: Kanäle, Sequenz, Auffälligkeiten).
