Some DJI UAV like Matrice 210 RTK don't have terrain awareness functionality. This project aims to create such feature

# Python 3 Requirements
pip install -r requirements.txt

# Preparing the data
## Generate kml of the area
### Clearance
Détails de la mission => Outils (capture d'écran, dossier mission...=>  Exporter la zone de vol au format Google Earth (KML)
### Google Earth
* In [Google Earth](https://earth.google.com/web/) click the <img src="img/google_earth_project.png" width="30"/>
* Click "New project" and select "Create a KML file" or "Create in Google Drive"
* Click "New element" and "Draw a line or shape"
* Create the shape and double clik on the first point to close the path. 
* It is not possible to remove point from a shape. A new shape needs to be draw
* Exit the “New element" by clicking the top left arrow
* Click <img src="img/burger.png" width="30"/> to export the kml file

## MNT
### Get the MNT from IGN
### Convert MNT to ellipsoid
### Generate .tfw
## Ini file
Copy the project_template.ini. ex : project.ini
### Import the kml to the ini file
```./google_kml_to_coordinates.py -i google_earth.kml -o project.ini```
### Project parameters
Modify the parameters in the project.ini file
# Run pathplanning
 ```./pathplanning.py -c project.ini```
 
A project.html file will be created with waypoints and flight path. Use this file to review your project 
The project.kml file then could be import to your DJI 

Références
https://link.springer.com/article/10.1007/s10846-016-0348-x
https://www.mdpi.com/2504-446X/3/1/4/pdf-vor

https://www.jpl.nasa.gov/news/meet-the-open-source-software-powering-nasas-ingenuity-mars-helicopter
https://www.hindawi.com/journals/complexity/2018/6879419/
https://sci-hub.do/https://arc.aiaa.org/doi/pdf/10.2514/6.2004-6227
https://en.wikipedia.org/wiki/Mesh_generation
https://www.mdpi.com/2220-9964/10/5/285/pdf
https://www.mdpi.com/2076-3417/9/7/1470/pdf-vor
