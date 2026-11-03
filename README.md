# MobileRobotSim

Browser-Simulator für mobile Roboter (Differentialantrieb) – Lehrmaterial, DHBW Stuttgart.

**Online starten:** https://papillon220.github.io/MobileRobotSim/

## Inhalte

- **Bahnplanung & Pfadverfolgung** – Planungsalgorithmen auf Belegungskarten
- **Manuelle Steuerung & Odometrie**
- **Lidar SLAM**
- **Pfadfolgeregelung**
- **ROBKIT1 Control** – Steuerung des Roboters ROBKIT1 (STM32 + BlueNRG) über Web Bluetooth: Modi (Fernsteuerung, Autopilot, Follow me, Wegpunkte), Joystick, Tastatur und PS4-Gamepad, Kartierung mit dem ToF-Sensor, Wegpunktfahrt mit Pure Pursuit, Parameter, Aufzeichnung und Replay. Lokalisierung in drei zuschaltbaren Stufen: Gyro-Offset im Stillstand (auf dem Roboter), Abgleich der ToF-Bilder mit der Karte, Ausrichtung an rechtwinkligen Wänden. Ohne Hardware: Simulation des Roboters mit wahrer Umgebung und Karte des Roboters nebeneinander (u. a. Bürogebäude mit Atrium, ganz kartierbar). Code in `js/tab-robkit.js`, Element-IDs mit Präfix `rk`.

Oben rechts schaltet *◐ Auto / ☀ Hell / ☾ Dunkel* die Darstellung aller Reiter um (Auto folgt dem System, die Wahl wird im Browser gespeichert).

## Lokal starten

`index.html` im Browser öffnen (Chrome, Edge, Firefox). Keine Installation, keine Internetverbindung nötig.

Für die Bluetooth-Verbindung im Reiter *ROBKIT1 Control* braucht der Browser Web Bluetooth (Chrome oder Edge) und HTTPS bzw. `http://localhost` – also die Online-Version oder einen lokalen Webserver (z. B. `python -m http.server` im Projektordner). Die Simulation des Roboters funktioniert auch direkt aus der Datei.

Die MATLAB-Skripte `mobRob_planningAlgorithms.m` und `mobRob_Lidar.m` sind die zugehörigen Referenzimplementierungen.

---
Prof. Dr.-Ing. Frank Bender · DHBW Stuttgart
