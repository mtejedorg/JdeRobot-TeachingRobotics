import numpy as np
from matplotlib import pyplot as plt

img_array = np.load('coso.npy')
plt.imshow(img_array)
plt.show()