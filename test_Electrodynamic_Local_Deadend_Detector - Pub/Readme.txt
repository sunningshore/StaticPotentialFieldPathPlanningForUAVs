This program finds feasible paths for a given predifined mission. The system is devided into several separated subsystems. Please run Runme.m. 

1.  shp2mat.m * 

Transaltes QGIS shape files into Matlab readable mats. 

* Please be noted that all date used in this project is confidential. We are sorry we couldn't provide full set of experiment data for verification. However, we can provide an artificial set of data that yields similar results. For verification, please skip this step.  

2.  main_BinMap.m 

Generates city landscape map, safety/accessibility heatmaps, core swich points, search boundaries. 

3. main_Layer_Map.m 

Finds possible paths. 

4. main_Intersection_Adjusting.m 

De-intersections. 
