# MobileRobotSim

Browser-Simulator für mobile Roboter (Differentialantrieb) – Lehrmaterial, DHBW Stuttgart.

**Online starten:** https://papillon220.github.io/MobileRobotSim/

## Inhalte

- **Manuelle Steuerung & Odometrie**
- **Bahnplanung & Pfadfolgeregelung** – Planungsalgorithmen auf Belegungskarten
- **Lidar SLAM**
- **Pfadfolgeregelung**
- **ROBKIT1 Control** – Steuerung des Roboters ROBKIT1 (STM32 + BlueNRG) über Web Bluetooth: Modi (Fernsteuerung, Autopilot, Follow me, Wegpunkte), Joystick, Tastatur und PS4-Gamepad, Kartierung mit dem ToF-Sensor, Wegpunktfahrt mit Pure Pursuit, Parameter, Aufzeichnung und Replay. Lokalisierung in drei zuschaltbaren Stufen: Gyro-Offset im Stillstand (auf dem Roboter), Abgleich der ToF-Bilder mit der Karte, Ausrichtung an rechtwinkligen Wänden. Ohne Hardware: Simulation des Roboters mit wahrer Umgebung und Karte des Roboters nebeneinander (u. a. Bürogebäude mit Atrium, ganz kartierbar), *Ideale Odometrie* schaltet alle Odometrie-Fehler mit einem Klick ab. **Co-Simulation** mit dem Simulink-Modell `robkit_SimMdl` (Repository [Robkit1-CTRL](https://github.com/Papillon220/Robkit1-CTRL), privat, Ordner `Simulink_Robkit`): Simulink rechnet Strecke und Controller, der Unterreiter *Co-Simulation* zeigt wahre Umgebung und Karte des Roboters und bedient ihn über dasselbe Protokoll wie per Bluetooth (WebSocket `ws://127.0.0.1:8767`). Code in `js/tab-robkit.js`, Element-IDs mit Präfix `rk`.

Oben rechts schaltet *◐ Auto / ☀ Hell / ☾ Dunkel* die Darstellung aller Reiter um (Auto folgt dem System, die Wahl wird im Browser gespeichert).

## Lokal starten

`index.html` im Browser öffnen (Chrome, Edge, Firefox). Keine Installation, keine Internetverbindung nötig.

Für die Bluetooth-Verbindung im Reiter *ROBKIT1 Control* braucht der Browser Web Bluetooth (Chrome oder Edge) und HTTPS bzw. `http://localhost` – also die Online-Version oder einen lokalen Webserver (z. B. `python -m http.server` im Projektordner). Die Simulation des Roboters funktioniert auch direkt aus der Datei. Für die Co-Simulation in MATLAB `robkit_sim_mode('cosim')` ausführen, `robkit_SimMdl` starten und im Unterreiter *Co-Simulation* „▶ Co-Simulation starten“ wählen.

Die MATLAB-Skripte `mobRob_planningAlgorithms.m` und `mobRob_Lidar.m` sind die zugehörigen Referenzimplementierungen.

---
Prof. Dr.-Ing. Frank Bender · DHBW Stuttgart
