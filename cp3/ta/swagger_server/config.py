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


## these variables keep track of the final-sim-time we report as
## (sim-time@done - sim-time@start) -
## sum(sim-time@adapted - sim-time@adapting)

time_at_start = None
time_at_done = None

expected_next_adapt = "ADAPTING"
time_spent_adapting = 0
time_at_last_adapt  = None

# uuid for log sequestering
uuid = None

## whether we got the ready message from the TH or from a file
th_connected = False

## light that are currently OFF (modulo what gazebo may be in the process
## of modifying). starts with the default off-set in the map def.
lights_off = [ "light%s" % n for n in [3,63,62,2,10,88,11,18,72,16]]
