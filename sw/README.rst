Description logiciel
====================

TODO
----

- Initialiser GPIO
  - Ecire des fonctions pour contrôler les relais
  - Definir le comportement par defaut au démarrage, pas de glitch!
- Mettre en place un timer 100Hz pour pouvoir faire dormir le CPU
- Configurer SPI pour LTC2400
- Initialiser entrées analogiques
- Initialiser DS18B20
- Initialiser UART (uniquement TX, on verra si on a besoin du RX plus tard)
  - Décider quel protocole utiliser sur le port série
- Mettre en place un watchdog
- Utiliser l'EEPROM pour stocker le compteur
  - Risque de problemes après 10000 écritures (Estimation)
  - Faire une vérification lecture après écriture et communiquer le problème


Reglages eFuse
--------------

- Low: Horloge externe, diviseur /8 actif
- High: tout par défaut
- Ext: Brownout detector: 4.3V

Commande AVRdude equivalente: `-U lfuse:w:0x50:m -U hfuse:w:0xd9:m -U efuse:w:0xfc:m`
