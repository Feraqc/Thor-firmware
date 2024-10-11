import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def create_3d_animation(num_points=100, interval=50):
    # Create figure and 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set the viewing angle
    ax.view_init(30, 30)

    # Create data for a 3D scatter plot
    t = np.linspace(0, 4 * np.pi, num_points)
    x = [1,2,3,4,5]
    y = [1,0,1,0,1]
    z = [1, 2, 3, 4, 5]

    scatter = ax.scatter([], [], [], c='r', marker='o')
    line, = ax.plot([], [], [], 'b-', lw=2)
    ax.plot(x,y,z)

    # Set axis limits
    ax.set_xlim((-100, 100))
    ax.set_ylim((-100, 100))
    ax.set_zlim((0,100))

    # State variable
    data = {'puntoRealizado': True, 'index': 0}

    # Initialization function
    def init():
        scatter._offsets3d = ([], [], [])
        ax.plot(x,y,z)
        return scatter, line

    # Animation function
    def animate(i):

        ##LEER SERIAL AQUI


        if data['puntoRealizado']:
            scatter._offsets3d = (x[:data['index']], y[:data['index']], z[:data['index']])
            data['index'] += 1
            data['num'] = False
        else:
            scatter._offsets3d = (x[:data['index']], y[:data['index']], z[:data['index']])
            data['num'] = True
        return scatter

    # Create the animation
    anim = FuncAnimation(
        fig,               # The figure object
        animate,           # The function to update the plot
        init_func=init,    # The initialization function
        frames=num_points, # Number of frames
        interval=interval, # Delay between frames in milliseconds
        blit=False         # Do not use blitting
    )

    # Display the animation
    plt.show()

create_3d_animation(num_points=100, interval=10)