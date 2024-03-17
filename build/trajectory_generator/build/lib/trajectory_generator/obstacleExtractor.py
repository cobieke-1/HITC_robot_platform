from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt


img = Image.open('/home/chris/capstone/hitc_ws/src/trajectory_generator/trajectory_generator/Converted_Maze.png')
img = ImageOps.grayscale(img)

np_img = np.array(img)
np_img = ~np_img
np_img[np_img > 0] = 1
plt.set_cmap('binary')
plt.imshow(np_img)

#Save Image
np.save('/home/chris/capstone/hitc_ws/src/trajectory_generator/trajectory_generator/Converted_Maze.npy', np_img)
grid = np.load('Converted_Maze.npy')
plt.imshow(grid, cmap = "binary")
plt.tight_layout()
plt.show()
