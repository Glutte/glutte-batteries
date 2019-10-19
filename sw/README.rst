Description logiciel
====================

Au démarrage, avant de passer à la mesure régulière, le code doit:

- Initialiser les GPIO
- Mettre en place un timer 100Hz pour pouvoir faire dormir le CPU
- Lire l'EEPROM pour charger le compteur mAh
  - Attention au risque de problemes après 100000 écritures
  - Faire une vérification lecture après écriture et communiquer le problème
- Mettre en place un watchdog

TODO
----

- Definir le comportement par defaut au démarrage, pas de glitch!
- Configurer SPI pour LTC2400
- Initialiser entrées analogiques
- Initialiser DS18B20
- Initialiser UART (uniquement TX, on verra si on a besoin du RX plus tard)
  - Décider quel protocole utiliser sur le port série


Reglages eFuse
--------------

- Low: Horloge externe, diviseur /8 actif
- High: tout par défaut
- Ext: Brownout detector: 4.3V

Commande AVRdude equivalente: `-U lfuse:w:0x50:m -U hfuse:w:0xd9:m -U efuse:w:0xfc:m`
