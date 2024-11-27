# multi_nav_pkg/msg/NavigationStatus.msg

string robot_namespace
string status          # E.g., "reached", "error", "navigating"
string error_message   # Dettagli sull'errore, se presente
float64 time_taken     # Tempo impiegato per raggiungere il waypoint
string current_waypoint
