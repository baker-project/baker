# List of directions (angles in degrees)  the vacuum cleaner shall clean

float32[] directions

# Number of repeated steps for each angle (one step consists of one
# forward and one backward movement. This list must have the same size
# as directions or be empty. In the latter case each step is executed
# once.

int32[] repetitions

# retract==true -> vacuum cleaner is retracted after completion of the
# whole operation, otherwise not

bool retract
