
def EstimateRates(position):
    # Naive Differentiation (amplifies noise)

    position_final = 0
    position_initial = 0
    time_final = 0
    time_initial = 0

    rates = (position_final - position_initial) / (time_final - time_initial) # positions are vectors and time is a scalar
    return rates

def CalculateControlActionLQR(setpoint, position, rates):

