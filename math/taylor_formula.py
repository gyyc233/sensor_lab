import numpy
import math
import matplotlib
import matplotlib.pyplot
import matplotlib.patches
import mpl_toolkits
import mpl_toolkits.mplot3d

def taylor_cosx():
    matplotlib.pyplot.close()
    figure = matplotlib.pyplot.figure(figsize = (8, 4))

    x = numpy.linspace(0, 20, 1000)
    cosy = numpy.cos(x)

    try:
        for t in range(1, 30):
            # 
            taylory = numpy.zeros(1000)
            for n in range(t):
                taylory = taylory + (numpy.power(-1, n) * numpy.power(x, 2 * n))/(math.factorial(2 * n))
            matplotlib.pyplot.cla()
            matplotlib.pyplot.grid(True)
            matplotlib.pyplot.ion()
            matplotlib.pyplot.ylim(-2.0, 2.0)
            matplotlib.pyplot.plot(x,cosy,label='cos', color = 'red', linewidth = 2)
            matplotlib.pyplot.plot(x,taylory,'b--',label='taylor')
            matplotlib.pyplot.pause(0.2)
    except Exception as err:
        print(err)
    
    return True

if __name__ == '__main__':
    taylor_cosx()
