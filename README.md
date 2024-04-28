Schießstandpieper
================

Hierbei handelt es sich um die Hard- und Software für ein Gerät, das zur Zeitmessung verschiedener sportlicher Aktivitäten verwendet werden kann und ursprünglich entwickelt wurde, um bei Schießwettbewerben laut genug zu sein. 

Hardware
---------
Die Hardware basiert auf einem Arduino Nano 33 IoT, der sowohl den WIFI-Zugangspunkt als auch den DAC zur Ansteuerung des Lautsprechers bereitstellt. 

Zusätzlich wurden eine Batterieladeschaltung und ein Verstärker hinzugefügt, damit der Ton laut genug ist. 

Der Ton wird von einem piezoelektrischen Hochtöner abgestrahlt, der ein hervorragendes Verhältnis zwischen Preis und Leistung bietet. 

Das Gehäuse ist 3d-gedruckt und optional mit einem schönen akustischen Nylontuch versehen.

Firmware
---------
Die Firmware hat drei Hauptfunktionen:
* Einrichten des WIFI-Zugangspunkts
* Bedienung der Steuerschnittstelle
* Aussenden von Tönen

Die Ansteuerung des DAC erfolgt direkt, da die Arduino-Bibliotheken Probleme verursachen. 

Steuerung
---------
Um das Projekt so einfach wie möglich zu halten, ist keine App für die Nutzung erforderlich. Das Gerät wird über WIFI gesteuert, wo es als Zugangspunkt ohne Zugangskontrolle erscheint. Sobald die Verbindung hergestellt ist, können Sie einen Browser auf 10.10.10.10 richten und die Schnittstelle wird angezeigt. Hier können Sie die Abläufe verwalten, auswählen und starten/stoppen.


