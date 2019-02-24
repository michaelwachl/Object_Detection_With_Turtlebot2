--------------------------------------------------------------------------------
Author: Michael Wachl
Jahr: 2018
Kurs: Leistungskurs C++
Gruppe: 8
Universität: Technische Universität München

--------------------------------------------------------------------------------

# Pfadplanung
## Beschreibung Package/Idee
Dieses Package stellt den Service zur Pfadplannung dar. Verschiedene Ziele
können als Optionen bei Serviceauruf angegeben werden. Dieser Node verwendet
die Klasse des D* Lite und die Klasse des SetPath. 

#

--------------------------------------------------------------------------------
## Service

## Als Eingabe benötigt der Service folgende Werte:
int8 PUCK_RADIUS

int8 ROBOT_RADIUS

int8 POST_RADIUS

float32 a

float32 b

string goal

### Als Rückgabe werden folgende Variablen zurückgegeben:
geometry_msgs/PoseArray path

bool success

### Aufrufbar mit
`/calculate_path`

--------------------------------------------------------------------------------
# Notwendige Libaries:
- standard ROS kinetic packages

#

