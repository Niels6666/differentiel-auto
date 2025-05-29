# differentiel-auto
Ce repository contient le code source de la simulation physique d'un véhicule avec [JBullet](http://jbullet.advel.cz/) pour un projet scolaire.
Le rendu est fait avec openGL et l'interface avec [ImGui](https://github.com/SpaiR/imgui-java).
![image](https://github.com/user-attachments/assets/cf6fd3eb-aa6b-4004-b645-9a1949de28e3)

## Comment l'essayer
- Télécharger le dossier `simulation` qui contient le .jar et les ressources nécessaires.
- Lancer le .jar avec Java 24

## Commandes
- Pour diriger le véhicule, utiliser les touches fléchées, et pour freiner la barre espace.
- Pour diriger la caméra, cocher "caméra libre" et utiliser les touches `Z` pour avancer, `Q` pour aller à gauche, `S` pour reculer, `D` pour aller à droite, `shift` pour monter `ctrl` pour descendre.

## Interface
La fenêtre debug contient 3 sections :
- Détails : quelques informations sur l'application
- Véhicule : des informations sur la position, la vitesse du véhicule, le type de transmission, etc...
- Enregistrement : il est possible d'enregistrer une manoeuvre dans un fichier et de la rejouer.

### Sections véhicule et enregistrement
- Le bouton `remettre à zero` réinitialise la position du véhicule et met les vitesses à zéro.
- L'angle de braquage correspond à l'angle d'une roue directionnelle fictive située au milieu de l'essieu avant.
- Cocher `Rediriger les roues` permet de réduire automatiquement l'angle de braquage à zéro.
- Pour afficher la trajectoire prédite des roues par la géométrie d'Ackermann, cocher `Trajectoires des roues`.
- Pour afficher les positions des roues, cocher `Traces des roues`.
- Pour visualiser la vitesse des roues, cocher `Vitesse des roues`. Cela affichera un graphique avec 4 courbes où `FL` correspond à la roue avant gauche, `FR` à la roue avant droite, `RL` à la roue arrière gauche et `RR` à la roue arrière droite.
- Pour simuler un différentiel ouvert, cocher `Différentiel ouvert`, sinon l'essieu moteur sera comme un essieu rigide.
- Pour enregistrer une manoeuvre, spécifiez un nom de fichier valide puis cliquer sur `Enregistrer`. Les fichiers sont enregistrés dans le dossier `res/save`.
- Pour rejouer une manoeuvre, sélectionner un fichier et cliquer sur `Lire le fichier`. Si vous voulez comparer le comportement du véhicule avec/sans différentiel, vous pouvez rejouer l'enregistrement avec `inverser USE_DIFFERENTIAL` coché ce qui cochera automatiquement `Différentiel ouvert` si ce n'était pas fait lors de l'enregistrement et inversement si c'était fait.
- Enfin vous pouvez sauvegarder les positions des roues (lorsque vous rejouez une manoeuvre) dans fichier .txt (au format csv) en cochant `Sauvegarder les résultats`. Le fichier sera écrit dans `res/results`. C'est utile pour une exploitation dans Regressi par exemple.

## Fichiers de manoeuvre
Le dossier `res/save` contient 3 fichiers de manoeuvre par défaut.

Une manoeuvre est fichier binaire qui contient les contrôles c'est-à-dire les commandes du véhicule à chaque frame. Chaque frame est représentée par un octet dont :
- le bit n°1 indique si la touche `↑` est pressée
- le bit n°2 indique si la touche `↓` est pressée
- le bit n°3 indique si la touche `←` est pressée
- le bit n°4 indique si la touche `→` est pressée
- le bit n°5 indique si la touche espace est pressée
- le bit n°6 indique si les roues doivent être redirigées
- le bit n°7 indique si on est en traction ou propulsion
- le bit n°8 indique si la transmission a un différentiel
