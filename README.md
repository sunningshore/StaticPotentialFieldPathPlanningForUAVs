# StaticPotentialFieldPathPlanningForUAVs 

This program finds feasible paths for a given predifined mission. The system is devided into several separated subsystems. Please run accordingly in the specified order. 

1.  shp2mat.m 

Transaltes QGIS shape files into Matlab readable mats. 

2.  main_BinMap.m 

Generates city landscape map, safety/accessibility heatmaps, core swich points, search boundaries. 

3. test_Layer_Map.m 

Finds possible paths. 

4. test_Intersection_Adjusting.m 

De-intersections. 
