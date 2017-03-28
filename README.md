# Detection

## lidar_driver

Dans ce dossier, on trouve : 
* le SDK fourni par le fabriquant : librplidar_sdk.a (précompilé en X86_64) & ses includes/
* mon driver : Lidar.cpp & Lidar.c 
* Un makefile : qui créée une lib statique libLidarDriver.a

## visu_qt

C'est une petit visu QT (no shit ?) que j'ai fait histoire de visualiser facilement les sorties du lidar. Ca se link avec les 2 libs statiques dans lidar_driver et se compile via le makefile.

## sdk

Le SDK fourni par le fabriquant pour regenerer librplidar_sdk.a sur votre PC ou pour la raspberry.