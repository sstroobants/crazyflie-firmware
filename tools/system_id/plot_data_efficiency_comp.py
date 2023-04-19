import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
import argparse

def loadFile(filename):
    fileData = np.loadtxt(filename, delimiter=',', skiprows=1, ndmin=2)
    return fileData

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--compare', nargs = '*')
    args = parser.parse_args()
    file1 = args.compare[0]
    file2 = args.compare[1]
    data1 = loadFile(file1)
    data2 = loadFile(file2)

    thrust1 = data1[1:1600,0] / 4  # g, per motor
    pwm1 = data1[1:1600,1]         # PWM value
    vbat1 = data1[1:1600,2]        # V, battery voltage, 
    rpm1 = np.mean(data1[1:1600,3:7],axis=1) # average over all motors
    vSid1 =  data1[1:1600, 7]      # Volts at system id deck
    amp1 =  data1[1:1600, 8]       # Amps
    pwr1 = data1[1:1600, 9]        # Power in watts
    pwr1_2 = vbat1 * amp1       # Power minus cable losses

    thrust2 = data2[1:1600,0] / 4  # g, per motor
    pwm2 = data2[1:1600,1]         # PWM value
    vbat2 = data2[1:1600,2]        # V, battery voltage, 
    rpm2 = np.mean(data2[1:1600,3:7],axis=1) # average over all motors
    vSid2 =  data2[1:1600, 7]      # Volts at system id deck
    amp2 =  data2[1:1600, 8]       # Amps
    pwr2 = data2[1:1600, 9]        # Power in watts
    pwr2_2 = vbat2 * amp2       # Power minus cable losses

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    fig.subplots_adjust(right=0.75)
    # Set axis ranges; by default this will put major ticks every 25.
    ax.set_xlim(0, 1600)
    ax.set_ylim(0, 6)

    ax.plot(thrust1/(pwr1_2 / 4), label='{} [g/W]'.format(file1),  color='tab:red')
    ax.plot(thrust2/(pwr2_2 / 4), label='{} [g/W]'.format(file2),  color='tab:blue')
    ax.plot(pwm1/65535, label='PWM[0-1]',  color='tab:green')
    ax.set_xlabel('Sample')
    ax.set_ylabel('Efficency [g/W]')
    ax.legend()
    
#    ax.grid(True)

    # Change major ticks to show every 20.
    ax.xaxis.set_major_locator(MultipleLocator(100))
    ax.yaxis.set_major_locator(MultipleLocator(1))

    # Change minor ticks to show every 5. (20/4 = 5)
    ax.xaxis.set_minor_locator(AutoMinorLocator(10))
    ax.yaxis.set_minor_locator(AutoMinorLocator(10))

    # Turn grid on for both major and minor ticks and style minor slightly
    # differently.
    ax.grid(which='major', alpha=0.5)
    ax.grid(which='minor', alpha=0.2)

    ax1 = ax.twinx()
    ax1.set_xlim(0, 1600)
    ax1.set_ylim(0, 120)
    ax1.plot(thrust1*4, color='tab:red')
    ax1.plot(thrust2*4, color='tab:blue')
    ax1.set_ylabel('thrust all motors [g]', color='tab:red')
    ax1.legend()

    ax2 = ax.twinx()
    ax2.set_xlim(0, 1600)
    ax2.set_ylim(0, 30)
    ax2.plot(rpm1/1000, color='tab:red')
    ax2.plot(rpm2/1000, color='tab:blue')
    ax2.set_ylabel('Avg kRPM', color='tab:red')
    ax2.legend()
    
    ax2.spines.right.set_position(("axes", 1.1))
    
    plt.title('Effeciency test, ramp up/down in steps')
    plt.show()
