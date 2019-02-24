--------------------------------------------------------------------------------
Author: Michael Wachl
Jahr: 2018
Kurs: Leistungskurs C++
Gruppe: 8
Universität: Technische Universität München

--------------------------------------------------------------------------------

# Puckerkennung
## Beschreibung Package/Idee
Dieses Package stellt den Puckkennungs-Node dar. Dieser Node verwendet die 
den Farbbildstream um den nächsten Puck unserer Farbe zu identifizieren 
(HSV-filtering, morphological operations, Höhe/Breitenverhätlnis) und ein
Feedback für die Motoransteuerung (Puck zu Kameramitte) zurückzugeben. Füllt
der Puck schließlich einen bestimmten Bereich aus, so wird ein Topic mit 
true gepublihed, dass der Puck aufgenommen wurde. 

#

--------------------------------------------------------------------------------
## Parameter
Alle Parameter sind in einem yaml-File gespeichert und können daher ohne neues komilieren
geändert werden. Zusätzlich sind die Parameter in Runtime über rqt_reconifgure 
veränderbar und als neues yaml-File exportierbar. Verwende dazu

`rosrun rqt_reconfigure  rqt_reconfigure`

Allgemein wurden alle Filterparameter relativ großzügig gewählt, damit die 
Objekterkennung immer noch bei sich ändernen Lichtverhältnissen stabil funktioniert.
Die Kameramitte, also 0 Pixel kann auch verändert werden, da diese auf dem Roboter 
nicht 0 ist. 
#

--------------------------------------------------------------------------------
## Topics
Topic mit dem Typ std_msgs::Bool ob ein Puck aufgenommen wurde

`/puck_pickedup`

Position des nächsten Pucks mit eigener Teamfarbe, als std_msgs::Float32 Nachricht

`/puck_pos_to_center`

Dies stellt das Puckzentrum relativ zur Kameraframemitte dar. Der Wert kann 
positiv (rechts) und negativ (minus) sein und ist 0 falls der Puck in Kameramitte
ist. 

--------------------------------------------------------------------------------
# Notwendige Libaries:
- standard ROS kinetic packages
- openCV2 libaries 


#




