
def EstimateRates(positions, times):
    # Naive Differentiation (amplifies noise)
    
    dt = times[-1] - times[-2]
    dx = positions[-1] - positions[-2]
    rates = dx / dt
    return rates


