# response from /ready
ready_response = None

# connection to the API
thApi = None

# current battery level, updated by call back to the ros topic
battery = None

# has the /start endpoint been hit once? this lets us fail on multiple
# starts
started = False

# logger from main
logger = None

# bot controller
bot_cont = None

# level
level = None

# waypoints for done message
tasks_finished = []

# for testing without th
th_connected = False