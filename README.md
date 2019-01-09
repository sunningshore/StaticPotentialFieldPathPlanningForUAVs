# StaticPotentialFieldPathPlanningForUAVs 

This program finds feasible paths for a given predifined mission. The system is devided into several separated subsystems. Please run Runme.m. 

1.  shp2mat.m 

Transaltes QGIS shape files into Matlab readable mats. 

2.  main_BinMap.m 

Generates city landscape map, safety/accessibility heatmaps, core swich points, search boundaries. 

3. main_Layer_Map.m 

Finds possible paths. 

4. main_Intersection_Adjusting.m 

De-intersections. 
