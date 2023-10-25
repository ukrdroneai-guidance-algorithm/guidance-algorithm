
import jsbsim
import matplotlib.pyplot as plt
import math

# Global variables that must be modified to match your particular need
# The aircraft name
# Note - It should match the exact spelling of the model file
AIRCRAFT_NAME = "737"
# Path to JSBSim files, location of the folders "aircraft", "engines" and "systems"
PATH_TO_JSBSIM_FILES = "E:/Programs/JSBSim"

# Avoid flooding the console with log messages
jsbsim.FGJSBBase().debug_lvl = 0

fdm = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)

# Load the aircraft model
fdm.load_model(AIRCRAFT_NAME)

# Set engines running
fdm['propulsion/engine[0]/set-running'] = 1
fdm['propulsion/engine[1]/set-running'] = 1

# Set alpha range for trim solutions
fdm['aero/alpha-max-rad'] = math.radians(12)
fdm['aero/alpha-min-rad'] = math.radians(-4.0)

# Set delta time based on our particular aircraft model (probably)
dt = fdm.get_delta_t()

# Max control deflection
aileronMax = 1
rudderMax = 1
elevatorMax = 1

# Number of seconds for control surface to reach max deflection
risetime = 0.5

# Per timestep increment for control surfaces
diAileron = aileronMax / (risetime/dt)
diRudder = rudderMax / (risetime/dt)
diElevator = elevatorMax / (risetime/dt)
print(diElevator)

# Control functions
def moveAileron(state):
    aileron = fdm['fcs/aileron-cmd-norm']
    if abs(aileron - state) < diAileron:
        aileron = state
    else:
        if aileron < state:
            aileron += diAileron
        else:
            if aileron > state:
                aileron -= diAileron
    fdm['fcs/aileron-cmd-norm'] = aileron

def moveRudder(state):
    rudder = fdm['fcs/rudder-cmd-norm']
    if abs(rudder - state) < diRudder:
        rudder = state
    else:
        if rudder < state:
            rudder += diRudder
        else:
            if rudder > state:
                rudder -= diRudder
    fdm['fcs/rudder-cmd-norm'] = rudder

def moveElevator(state):
    elevator = fdm['fcs/elevator-cmd-norm']
    if abs(elevator - state) < diElevator:
        elevator = state
    else:
        if elevator < state:
            elevator += diElevator
        else:
            if elevator > state:
                elevator -= diElevator
    fdm['fcs/elevator-cmd-norm'] = elevator

# Recorded data
times = []

rollAngle = []
pitchAngle = []
altitude = []

ailerons = []
elevators = []

# Initial conditions
fdm['ic/h-sl-ft'] = 5000 # Высота в футах
fdm['ic/vc-kts'] = 200 # Скорость в узлах
fdm['ic/gamma-deg'] = 0
fdm['ic/beta-deg'] = 0

# Initialize the aircraft with initial conditions
fdm.run_ic()

# Trim
try:
    fdm['simulation/do_simple_trim'] = 1

except jsbsim.TrimFailureError:
    print("Trim failed, continuing rudder kick in an untrimmed state.")
    pass  # Ignore trim failure

# Time to run for in seconds
run_period = 15

for i in range(int(run_period/dt)):
    fdm.run()
    second = fdm.get_sim_time()
    times.append(fdm.get_sim_time())

    rollAngle.append(fdm['attitude/phi-deg'])
    pitchAngle.append(fdm['attitude/theta-deg'])
    altitude.append(fdm['position/h-agl-ft'])

    ailerons.append(fdm['fcs/aileron-cmd-norm'])
    elevators.append(fdm['fcs/elevator-cmd-norm'])

    aileronCmd = fdm['fcs/aileron-cmd-norm']
    rudderCmd = fdm['fcs/rudder-cmd-norm']
    elevatorCmd = fdm['fcs/elevator-cmd-norm']
    if second <= 12:
        moveAileron(aileronMax)
    if second > 12:
        moveAileron(0)

# Plot results
plt.figure(figsize=(10, 8))

plt.subplot(2, 2, 1)
plt.xlabel('Time (s)')
plt.ylabel('Угол в градусах')
plt.title('Угол крена')
plt.plot(times, rollAngle)

plt.subplot(2, 2, 2)
plt.xlabel('Time (s)')
plt.ylabel('Altitude (ft)')
plt.title('Высота в футах')
plt.plot(times, altitude)

plt.subplot(2, 2, 3)
plt.xlabel('Time (s)')
plt.ylabel('State')
plt.title('Элероны')
plt.plot(times, ailerons)

plt.subplot(2, 2, 4)
plt.xlabel('Time (s)')
plt.ylabel('Угол в градусах')
plt.title('Угол тангажа')
plt.plot(times, pitchAngle)

# Регулируем расположение графиков
plt.tight_layout()
# Отображаем графики
plt.show()