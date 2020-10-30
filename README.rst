Mesure de l'état de charge de la batterie
=========================================

Les nouvelles (juillet 2019) batteries au plomb nécessitent une nouvelle
approche pour mesurer leur état de charge.

Mesurer la tension ne suffit pas, d'abord parce que la différence de tension
entre batterie pleine et vide est faible, d'autre part parce que la tension
mesurée dépend fortement du courant de dé-/charge.

Proposition
-----------

Mesurer le courant à intervalle régulier (p.ex. à 10Hz) et intégrer le courant
pour calculer une charge accumulée ou extraite de la batterie (A * s = Coulomb).

Besoins
-------

Quand le niveau de charge des batteries est trop bas, il faut couper des
consommateurs. Il est possible de commuter les groupes de consommateurs suivants:

- Balise 70cm
- Mode relais QRP/QRO
- Alimentation Relais
- Alimentation Raspberry Pi, WiFi et autres consommateurs 5V restants

Afin que le relais puisse indiquer le niveau de charge, un port série est
utilisé entre le coulomb-mètre et la glutt-o-matique (unidirectionnel). Pour la
surveillance ainsi que pour d'autres applications, un deuxième port série transmet
les mêmes informations à un Raspberry Pi connecté à Internet.

Le colomb-mètre doit donc comporter trois commutations à relais, qui peuvent à
leur tour commuter des relais de puissance.

Quelques définitions
--------------------

Tension nominale: Tension d'un élément chargé au repos à 25°C : 2.1V/élément soit
12.6V pour la traditionnelle batterie dite de 12V. C'est ce que vous devez lire
(à la précision de la mesure près) sur une batterie que vous avez chargée et
ensuite débranchée pendant une nuit.

Tension de floating (tension de charge d'entretien):
Tension à laquelle on peut maintenir en permanence un accumulateur
pour être sûr qu'il soit chargé au moment où en a besoin : 2.25 à 2.28/élément à
25°C. Cette valeur devrait être corrigée de 0.005V en plus ou en moins par degré
centigrade selon que la température descend ou monte.
A -10°C c'est 2.36V et à +40°C 2.21V.

Tension de recharge:
Tension maximum à laquelle on peut charger la batterie (mais pas la laisser en
permanence). 2.3 à 2.4V/élément toujours à 25°C et avec le même coefficient de
température de 0.005V/°C. Soit 13.8 à 14.4V pour un bloc 12V à 25°C.
Attention à prendre aussi en considération les pics de courte durée si la
tension est bruitée.

Evitez de décharger la batterie en dessous de 11.9V, car vous commencez à
décharger profondément la batterie.

Risque de gel: Une batterie profondément déchargée risque le gel!

+---------+---------------+------------------------+--------------------+
|  Baumé  | Densité H2SO4 | Etat de charge         | Température de gel |
+=========+===============+========================+====================+
| 13 à 23 | 1.099 à 1.190 | Profondément déchargée | entre -7C et -27C  |
+---------+---------------+------------------------+--------------------+
| 24 à 29 | 1.200 à 1.252 | Moyenne                | entre -27C et -52C |
+---------+---------------+------------------------+--------------------+
| 30 à 32 | 1.262 à 1.285 | Pleine                 | plus bas que -52C  |
+---------+---------------+------------------------+--------------------+

Composants
----------

Boitier: Hammond RZ0214C

Shunt: muRata 3020-01096-0
