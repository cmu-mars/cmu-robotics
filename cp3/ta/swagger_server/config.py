# the CP3 object
cp = None

# the battery level, which will get updated by subscription to a ros topic
battery = None

# the start waypoint name from /ready
start = None

# the target waypoint name from /ready
target = None

# the logger captured in __main__
logger = None

# whether or not /start has been hit at least once yet, so that it can
# return an error if it's hit multiple times
started = False

# connections to the TH
thApi = None

# number of adaptations
adaptations = 0

# are we in adapting mode or not?
use_adaptation = False

#what is our config
nodes = []

sensors = []

plan = None

collisions = []
