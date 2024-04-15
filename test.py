import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

# Your update_plot function as defined before

# Set up the figure and subplot
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)  # Adjust bottom margin to accommodate the slider

# Define initial frame index
frame_index = 0

# Create slider
ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')  # Define slider position and size
slider = Slider(ax_slider, 'Frame', 0, len(data['utime']) - 1, valinit=frame_index, valstep=1)

# Update function for the slider
def update(val):
    frame_index = int(slider.val)
    update_plot(frame_index)
    fig.canvas.draw_idle()

slider.on_changed(update)

# Initial plot
update_plot(frame_index)

# Show the plot
plt.show()
