--------------------------------------------------------------------------------
Author: Michael Wachl
Jahr: 2018
Kurs: Leistungskurs C++
Gruppe: 8
Universität: Technische Universität München

--------------------------------------------------------------------------------

# Puckerkennung mit LIDAR
## Beschreibung Package/Idee
Dieses Package stellt den Puckkennungs-Node mit LIDAR dar. Dieser Node verwendet die 
den LIDAR und clustert (k-d-Baum) und filtert den nächstgelegenen Puck und published diesen anschließend.

#

--------------------------------------------------------------------------------
## Parameter
Alle Parameter sind in einem yaml-File gespeichert und können daher ohne neues komilieren
geändert werden. Zusätzlich sind die Parameter in Runtime über rqt_reconifgure 
veränderbar und als neues yaml-File exportierbar. Verwende dazu

`rosrun rqt_reconfigure  rqt_reconfigure`

Allgemein wurden alle Filterparameter relativ großzügig gewählt.

#

--------------------------------------------------------------------------------
## Topics

Position des nächsten Pucks als geometry_msgs::PointStamped msg

`/closest_puck`

--------------------------------------------------------------------------------
# Notwendige Libaries:
- standard ROS kinetic packages
- pcl libaries 
- Eigen3 libaries 


#

--------------------------------------------------------------------------------
# Launch
Der Launch dieses Nodes inklusieve remapping des 360 Grad Lidar
und des rplidarNode kann mit folgendem Launch-File gestartet werden

`roslaunch wachl_puck_lidar  get_puck_detection.launch`



