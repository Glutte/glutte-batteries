Description logiciel
====================

Au démarrage, avant de passer à la mesure régulière, le code doit:

- Initialiser les GPIO
- Mettre en place un timer 100Hz pour pouvoir faire dormir le CPU
- Lire l'EEPROM pour charger le compteur mAh
  - Attention au risque de problemes après 100000 écritures
  - Faire une vérification lecture après écriture et communiquer le problème
- Mettre en place un watchdog
- Initialiser UART (uniquement TX, on verra si on a besoin du RX plus tard)
- Configurer SPI pour le LTC2400
- Initialiser entrées analogiques et mesurer tension batterie
- Initialiser entrée numérique D4 pour mesurer contacts auxiliaires des dijoncteurs des trois phases de l'éolienne.

Protocole du port série
-----------------------

Chaque message commence par un identificateur, suivi d'une virgule, du temps en
secondes, une virgule, un champ de données, et termine par CR LF.

+------------------+---------------------------------------------+
| Identificateur   | Contenu du champ                            |
+------------------+---------------------------------------------+
| `TEXT`           | Un message informatif                       |
| `ERROR`          | Erreur ou avertissement                     |
| `CAPA`           | Une valeur en mAh                           |
| `RELAY`          | Etats des trois relais, `On` ou `Off`       |
| `VBAT+`          | Une valeur en mV                            |
| `VBAT-`          | Une valeur en mV                            |
| `DISJEOL`        | `On` ou `Off`, état du disjoncteur eolienne |
+--------------------+-------------------------------------------+

Par exemple: `TEXT,12,Startup\r\n`

TODO
----

- Definir le comportement par defaut au démarrage, pas de glitch!


Reglages eFuse
--------------

- Low: Horloge crystal 8MHz ou plus, Startup time PWRDWN/RESET 1K/14CK + 4.1ms, diviseur /8 actif
- High: tout par défaut
- Ext: Brownout detector: 4.3V

Commande AVRdude equivalente: `-U lfuse:w:0x7e:m -U hfuse:w:0xd9:m -U efuse:w:0xfc:m`

Remarques
---------

- Le DS18B20 ne sera pas assemblé, l'entrée D4 et son footprint réutilisés pour brancher les auxiliaires des
  disjoncteurs de l'éolienne


Acknowledgements
================

Le dossier `lib/` contient des fichiers des bibliotheques suivantes:

- OneWire https://github.com/PaulStoffregen/OneWire
- DallasTemperature https://github.com/milesburton/Arduino-Temperature-Control-Library.git
- avr-uart https://github.com/andygock/avr-uart
- Code exemple LTC2400 de l'application note correspondante
