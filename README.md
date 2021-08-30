Dans un premier temps, on peut partir d’une zone à mapper rectangulaire définie avec X0,YO et X1,Y1 les coordonnées des coins de la zone.

![map_1](/img/map_1.png)

Génération des points en 2D
taille de la zone = X1-XO et Y1-Y0
GSD => il faut prendre un point tout les X mètres
=> (X1-X0)/X et (Y1-Y0)/X
![map_2](/img/map_2.jpg)

Prise en compte du MNS pour la génération des points 3D
![map_3](/img/map_3.jpg)

Génération du kml

les objets waypoints générés seront ajoutés à une liste
- [ ] **generates_kml**
  -  Input
      -  X0,YO et X1,Y1
      -  altitude absolue du point de décollage du drone (ex 124m)
      -  hauteur de survol du point (ex = 10m) ou GSD : Valeur en cm d’un pixel sur l’image finale (ex 0.82cm)
      -  recouvrement H et V (peut être qu’on peut partir sur le même recouvrement )
      -  vitesse du drone sur le parcour
      -  orientation de la caméra
      -  type de caméra
      -  angle du parcours par rapport au nord géographique
  -  Output 
      - tableau avec les coordonnées des waypoints et/ou fichier KML

- [ ] **absolute_height**
  -  Input
      -  coordonnées d’un point X,Y
      -  altitude absolue du point de décollage du drone (ex 124m)
      -  GSD : Valeur en cm d’un pixel sur l’image finale (ex 0.82cm) ou hauteur de survol du point (ex = 10m)
      -  référence du fichier MNS (paris.MNS)
  - Output
      - altitude absolue de survol du point

- [ ] **generates_2D_waypoints**
  -  Input
      -  X0,YO et X1,Y1
      -  recouvrement H et V (peut être qu’on peut partir sur le même recouvrement )
      -  altitude absolue du point de décollage du drone (ex 124m)
      -  hauteur de survol du point (ex = 10m)
  - Output
      -  tableau avec les coordonnées des waypoints 2D

- [ ] **generates_3D_waypoints**
  -  Input
      -  tableau avec les coordonnées des waypoints 2D
      -  référence du fichier MNS (paris.MNS)
      -  altitude absolue du point de décollage du drone (ex 124m)
      -  hauteur de survol du point (ex = 10m)
  - Output
      -  tableau avec les coordonnées des waypoints 3D

- [ ] **return_relative_height**
  -  Input
      -  coordonnées X,Y d’un point de l’espace (ex ​​48.84493567421954, 2.3561025820862844)
      -  altitude absolue du point de décollage du drone (ex 124m)
      -  hauteur de survol du point (ex = 10m)
      -  référence du fichier MNS (paris.MNS)
  - Output
      -  distance verticale entre l’altitude de décollage du drone et le point

- [ ] **emprise_camera**
  -  Input
      -  référence de caméra. peut être un indice dans un tableau avec les noms des caméras ? (ex = X7)
      -  hauteur de survol
  - Output
      -  largeur et longeur en mètres de la photo

- [x] **latitude_longitude_plus_distance_and_bearing**
  -  Input
      -  latitude 48.84482270388685
      -  longitude 48.84482270388685
      -  distance_x (10m)
      -  distance_y (10m)
  - Output
      -  latitude + distance_x
      -  longitude + distance_y

- [ ] **return_altitude_from_gsd**
  -  Input
      -  référence de caméra. peut être un indice dans un tableau avec les noms des caméras ? (ex = X7)
      -  GSD : Valeur en cm d’un pixel sur l’image finale (ex 0.82cm)
  - Output
      -  altitude de survol pour avoir le GSD avec la caméra sélectionnée
- [X] **Generate_mission_map** : Generate a map with UAV positions, pictures footprint, waypoints, path
-  Input
      -  array of waypoints  


Références
https://link.springer.com/article/10.1007/s10846-016-0348-x
https://www.mdpi.com/2504-446X/3/1/4/pdf-vor

https://www.jpl.nasa.gov/news/meet-the-open-source-software-powering-nasas-ingenuity-mars-helicopter
https://www.hindawi.com/journals/complexity/2018/6879419/
https://sci-hub.do/https://arc.aiaa.org/doi/pdf/10.2514/6.2004-6227
https://en.wikipedia.org/wiki/Mesh_generation
https://www.mdpi.com/2220-9964/10/5/285/pdf
https://www.mdpi.com/2076-3417/9/7/1470/pdf-vor
