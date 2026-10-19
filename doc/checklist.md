# Checkliste Windenschlepp

Zum Ausdrucken/Laminieren, siehe [WINCH-24](../features/WINCH-24-printable-checklist.md). Details und Hintergründe: [README](../README.md), [WINCH-09 Handbuch](../features/WINCH-09-user-maintenance-manual.md), [vesc/vesc-tool-guide.md](../vesc/vesc-tool-guide.md).

⚠ = sicherheitsrelevant (siehe CLAUDE.md "Rewind-Regel" / "Pull-out-Regel").

## 1. Am Tag vorher

- [ ] 11 V LiPo für Kamera-Empfänger + Monitor (Cockpit) geladen
- [ ] Vario geladen
- [ ] 60 V Windenbatterie geladen
- [ ] Akku Akkuschrauber geladen
- [ ] Fernbedienung(en) geladen

## 2. Am gleichen Tag, vor dem Losfahren

**Mitnehmen:**

- [ ] Winde
- [ ] Bremsschirm (Drogue) für das Seil
- [ ] Windenbatterie
- [ ] Controller (VESC) / Empfängereinheit
- [ ] Fernbedienung(en); Admin-Fernbedienung (rot) bei mehreren Piloten
- [ ] Spanngurte, Erdnägel/Schrauben und Akkuschrauber (Winde am Boden fixieren)
- [ ] Antenne (Empfänger)
- [ ] Kamera (Winde, von VESC versorgt), Kamera-Empfänger + Monitor (Cockpit), 11 V LiPo dafür
- [ ] Gleitschirm, Gurtzeug mit Reserve, Cockpit

**Beachten:**

- [ ] Wind / Wetter / Luftraum geprüft
- [ ] Bei mehreren Piloten: für jeden Piloten eigene Fernbedienung mit eigener ID und eigenem `myMaxPull` (Startgewicht)
- [ ] Helfer eingewiesen (falls vorhanden)

## 3. Aufbau der Winde und Vorbereiten zum Ausziehen des Seils

- [ ] Winde mit Spanngurten (und ggf. Erdnägeln) am Boden fixieren
- [ ] Antenne aufstellen und anschließen
- [ ] Controller (VESC) und Empfänger einschalten
- [ ] VESC-LED nicht rot (sonst Fehler auslesen, siehe [vesc/vesc-tool-guide.md](../vesc/vesc-tool-guide.md))
- [ ] Sensorkabel (Hall + Motortemperatur) eingesteckt (sonst meldet der VESC Übertemperatur und gibt keine Leistung)
- [ ] ⚠ Potentiometer ganz links (Aus-Endstellung)
- [ ] Fernbedienung einschalten, Verbindung OK (RSSI im OLED), Windenmodus **Softbremse "B -1" (−7 kg)**
- [ ] Kamera und Monitor an, Bild der Trommel sichtbar

## 4. Während des Ausziehens und am Startplatz, vor dem Start

- [ ] ⚠ Softbremse aktiv (B -1), **niemals Neutral** (Neutral → Trommel läuft über → Seilwickel)
- [ ] Seil mit dem Bremsschirm gleichmäßig gehend ausziehen
- [ ] Seillage auf der Trommel prüfen (Monitor)
- [ ] Seil, Sollbruchstelle, Verbindung zum Gurtzeug geprüft
- [ ] Fernbedienung in der Hand / befestigt, OLED zeigt Verbindung
- [ ] Potentiometer noch an der Aus-Endstellung

## 5. Während des (Stufen-)Schlepps und zum Ausklinken

- [ ] Fernbedienung während des ganzen Schlepps an und in Reichweite
- [ ] Zugstufen schrittweise erhöhen (Default → Vorzug → Startzug → Vollzug → starker Zug), nicht überspringen
- [ ] Bei Problem: DOWN lang drücken → Softbremse; danach ggf. Doppelklick DOWN → Neutral
- [ ] Ausklinken wie geplant (Höhe/Position), danach Winde auf Softbremse oder wie vereinbart

## 6. Nach dem Schlepp / Seil einholen

- [ ] ⚠ Einholen nur mit **max. Stufe 2 (prePull)** – mit mehr Zug kann die Autostop-Bremse die Trommel nicht rechtzeitig stoppen (Karabiner wird ins Azimut-System gezogen, Seil reißt; bereits einmal passiert)
- [ ] Fernbedienung bleibt **an**, bis das Seil vollständig eingeholt ist und AutoStop die Trommel gestoppt hat
- [ ] Seillage auf der Trommel während des Einholens beobachten (Monitor)
