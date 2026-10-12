# Landrumower (Sunray-Fork)

Fork von `Ardumower/Sunray` (`upstream`) für den Eigenbau-Mäher „Landrumower". `origin` ist `EinEinfach/Sunray`.

## Aktuelles Ziel

**Pico-Firmware 4.x in C++, komplett from scratch**, unter `landrumower/firmware/4.x`.

- 1.x und 2.x (MicroPython) sowie 3.x (C++) bleiben als Referenz liegen und werden nicht weiterentwickelt.
- 2.x (Stand 2.5.1) hat alle Features, schaffte aber nur etwa 10 Loops/s und war damit für die Motorregelung unbrauchbar.
- 3.x wurde nie fertig. Sie lief testweise auf dem Mäher, aber nicht mit allen Features.

## Aufbau des Mähers (Branch `landrumower-linux`)

- Raspberry Pi 4 mit Sunray als Linux-Build (`alfred/build/sunray`, systemd-Service, siehe `landrumower/readme.md`).
- Raspberry Pi Pico als Motor- und Sensor-MCU auf eigener KiCad-Platine (`landrumower/schematics/`). Es ist ein Pico 1 (RP2040) ohne W, bestätigt über den Bootloader (`RPI-RP2`, UF2 Bootloader v3.0).
- Pi ↔ Pico: UART `/dev/ttyS0`, 115200 Baud, AT-Protokoll des `SerialRobotDriver` (`AT+M`, `AT+S`, `AT+V`, dazu eigene `AT+R` und `AT+Y3`). **Das Protokoll bleibt für 4.x unverändert.**
- Motortreiber: JYQD (bleiben). Strommessung über vier INA226 per I2C, dazu LCD, Buzzer, Bumper, Lift, Stop, Regensensor.
- Bekannte Platinenfehler und Workarounds: `landrumower/schematics/findings.txt`.

## Verfügbare Hardware (Stand 10/2026)

Der Mäher selbst steht nicht zur Verfügung. Auf dem Tisch liegt nur die Hauptplatine mit aufgestecktem Pico, Display und Buzzer. Der Pi ist nicht erreichbar.

- Vorhanden: Pico (Versorgung nur über USB), LCD am I2C, Buzzer.
- Der Pico ist vom Mac per USB erreichbar (Datenkabel vorhanden). Im BOOTSEL-Modus erscheint er als Laufwerk `RPI-RP2`.
- Fehlt: alle vier INA226, Motoren, JYQD-Treiber, damit auch Encoder-Impulse.
- 4.x wird also zunächst am Tisch entwickelt und getestet, ohne Mäher und ohne laufendes Sunray. Der Mac spielt Sunray und schickt die AT-Kommandos über USB.
- Fehlende Hardware (INA226, Motor mit Encoder) muss in der Firmware simulierbar sein, ohne dass der echte Codepfad dafür umgebaut wird.
- Sunray wird auf dem Pi selbst gebaut (cmake in `alfred/build`) mit der Landrumower-Config. Das ist vorerst kein Thema.

## Entscheidungen für 4.x

- **Toolchain:** PlatformIO mit earlephilhower-Arduino-Core (wegen funktionierendem Dual-Core über `setup1()`/`loop1()`). Pico-SDK-Funktionen direkt nutzen, wo Arduino zu langsam ist.
- **Aufteilung:** Core 0 für Encoder, PID, PWM und UART im festen Takt. Core 1 für alles Langsame und Blockierende (INA226, LCD, Buzzer, Regen, Batterie). Keine `String`-Objekte im zeitkritischen Pfad.
- **Regelung:** Der PID läuft auf dem Pico. Sunray schickt Soll-Geschwindigkeiten in mm/s (letzte zwei Felder von `AT+M`).
- **Bremsen:** PWM auf dem Brake-Eingang der JYQD-Treiber ist ein Versuch. Laut Datenblatt ist der Eingang binär, der nutzbare PWM-Bereich ist vermutlich klein. Deshalb: Frequenz und Duty-Grenzen konfigurierbar, ein Messmodus zum Ausmessen des Bereichs, Rückfall auf binäres Bremsen per Config.

## Funktionsumfang und Struktur von 4.x

- **Umfang:** alles, was 2.5.1 konnte, eventuell mehr. Referenz ist `landrumower/firmware/2.x`. Dazu gehören `AT+V`, `AT+M`, `AT+S`, `AT+R`, `AT+E`, `AT+Y`, `AT+Y3`, Überstromerkennung mit Treiber-Recovery, Batterie (Unterspannungsabschaltung, Ladeerkennung, Power-Switch), Bumper, Lift, Stop-Taste, Regensensor, LCD mit Sunray-Status, Buzzer-Muster, Watchdog und die USB-Konsole.
- **Zwei Regelmodi bleiben erhalten:** Der Pico regelt selbst (Soll-Geschwindigkeit aus `AT+M`), oder Sunray regelt und der Pico reicht nur PWM durch. Per Config umschaltbar.
- **Neu gegenüber 2.5.1:** vorerst nur das geregelte Bremsen.
- **Flache Feature-Architektur:** ein Ordner pro Feature auf einer Ebene, keine tiefen Schichten.
- **Tests parallel dazu:** eigener `test`-Ordner, dessen Aufbau die Feature-Ordner spiegelt. Jedes Feature bekommt seine Tests zusammen mit dem Code, nicht nachträglich.

Vereinbartes Layout (die Feature-Liste ist ein erster Schnitt und darf sich beim Bauen verschieben):

```
landrumower/firmware/4.x/
├── platformio.ini          Umgebungen: pico, pico_sim, native (Tests auf dem Mac)
├── include/config.h        Pins, Grenzwerte, Modi
├── src/main.cpp            nur Verdrahtung: setup/loop für Core 0 und Core 1
├── lib/<feature>/          protocol, pid, motor, encoder, current, battery,
│                           sensors, lcd, buzzer, console
└── test/test_<feature>/    ein Ordner je Feature, gleiche Namen
```

- Features liegen unter `lib/`, weil PlatformIO sie dort automatisch findet und einzeln für die nativen Tests baut.
- Testordner brauchen das Präfix `test_`, sonst erkennt PlatformIO sie nicht. Die Spiegelung ist also `lib/motor` zu `test/test_motor`.

## Stand von 4.x

- Fertig mit Tests: `protocol` (AT-Parser, CRC, Antworten, Zeilenleser) und `pid`.
- `src/main.cpp` beantwortet bisher `AT+V` echt und `AT+M`/`AT+S` mit leeren Berichten. Alle anderen Features fehlen noch.
- Auf dem Pico am Tisch geflasht und über USB geprüft (11.10.2026): Antworten und CRCs stimmen, 200 `AT+M` im 20-ms-Takt ohne Verlust. Der UART zum Pi ist ungetestet.
- Flashen mit `pio run -e pico -t upload` (Port `/dev/cu.usbmodem1101`). Läuft 4.x bereits, wechselt der Upload selbst in den Bootloader. Lief vorher MicroPython, muss der Pico erst über die REPL mit `machine.bootloader()` oder per BOOTSEL-Taste in den Bootloader.
- Strenger als 2.x/3.x: Ein vorhandener, aber falscher CRC wird auch über USB abgelehnt. `landrumower/sim/sim_sunray_comm.py` sendet einen falschen CRC und passt deshalb nicht zu 4.x.

## Werkzeuge und Konventionen

- PlatformIO liegt in der Projekt-venv: `.venv/bin/pio` (Core 6.1.18). Die earlephilhower-Toolchain für den Pico ist bereits installiert.
- Aus `landrumower/firmware/4.x`: Tests mit `../../../.venv/bin/pio test -e native`, Firmware bauen mit `../../../.venv/bin/pio run -e pico -e pico_sim`.
- Code, Kommentare und Commit-Nachrichten auf Englisch. Mit dem Nutzer wird Deutsch gesprochen.

## Testaufbau für 4.x

Drei Ebenen, damit trotz fehlender Hardware fast alles prüfbar ist:

1. **Native Tests auf dem Mac, ohne Pico.** PID, Protokoll-Parser mit CRC und Geschwindigkeitsberechnung sind reine C++-Module ohne Arduino-Abhängigkeit und laufen über die native PlatformIO-Umgebung.
2. **Simulierte Hardware auf dem Pico.** Motor und Stromsensoren haben je eine schmale Schnittstelle mit echter und simulierter Umsetzung (einfaches Motormodell: PWM rein, Drehzahl und Strom raus). Regler, Protokoll, LCD und Buzzer laufen unverändert.
3. **Encoder-Schleife per Drahtbrücke (optional).** Der Pico gibt die simulierten Encoder-Impulse auf einem freien Pin aus, der auf den Encoder-Eingang gebrückt wird. Das prüft den echten Interrupt-Pfad.

Der Mac schickt dabei die AT-Kommandos im 20-ms-Takt wie Sunray. Nicht prüfbar ohne Treiber und Motor: das PWM-Bremsen.

## Branches

| Branch | Inhalt |
|---|---|
| `landrumower-linux` | Pi + Pico. Arbeitsbranch für 4.x. |
| `landrumower` | Anderer Aufbau: Sunray direkt auf der Ardumower-Platine (`DRV_ARDUMOWER`, JYQD), alter Sunray-Stand von 10/2022. Enthält die aktuelle Hindernislogik. |
| `master` | Spiegel des Upstream-Stands von 04/2025. |

## Offene Punkte (Sunray-Seite, unabhängig von 4.x)

- **Hindernislogik von `landrumower` nach `landrumower-linux` übernehmen.** Quelle sind die Commits `168e01d`, `2d85661` und `79dd9e8`. Sie ersetzt die Sperre in `Map::clearObstacles()` aus `8872947`, die fälschlich bei allen Aufrufern greift. Zielverhalten:
  - Mähpunkt erreicht und weniger als `OBSTACLE_MAX_NUMBER` (20) Hindernisse: nach `obstaclesCpy` sichern, aktive Liste leeren.
  - Neues Hindernis: Sicherung zurückholen, dann das neue anhängen.
  - Grenze erreicht oder Ladestation angedockt: alles löschen, auch die Sicherung.
- `OBSTACLE_MAX_NUMBER` steht nur in `sunray/config.h`. Der CMake-Build überschreibt diese Datei mit der gewählten Config, in `landrumower/config.h` und `alfred/config.h` fehlt der Wert.
- `origin/tests` existiert noch auf GitHub (lokal gelöscht, letzter Commit `2d966bf`, inhaltlich durch `landrumower` überholt). Löschen nur nach ausdrücklicher Bestätigung.
- Provisorien in Sunray auf `landrumower-linux`, deren Absicht nicht geklärt ist: festes `return 25` in `SerialBatteryDriver::getBatteryVoltage()` und `battery.chargingCurrent` statt `motor.motorsSenseLP` in `cmdSummary()`. Nicht stillschweigend ändern, erst nachfragen.
- Ungeklärt: ob eine neue Platinenrevision geplant ist, und welches Tool die eigenen Kommandos `AT+CM` und `AT+E2` an Sunray schickt.
