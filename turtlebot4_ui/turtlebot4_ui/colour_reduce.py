
# image processing software options:
# OpenCV
# Pillow
# scikit-image
# pytorch and tensorflow
import numpy as np
from PIL import Image, ImageColor

'''
This is a helper function for Color_Reduce

it determines what the nearest colour out of a preset list of colours is to a given pixel. 
'''


def find_nearest_color(pixel, custom_colors):
    min_distance = float('inf')
    nearest_color = None
    for color in custom_colors:
        distance = np.linalg.norm(np.array(pixel) - np.array(color))
        if distance < min_distance:
            min_distance = distance
            nearest_color = color
    return nearest_color


'''
This function takes in an image bitmap and reduces it to another image bitmap with fewer colors. 

image path is a file path to the image to color reduce
colors is a list 3 hexadecimal colors
'''


def color_reduce(image_path, save_path, colors):
    # load the image with Pillow
    image = Image.open(image_path)
    # convert the image to a NumPy array
    image_np = np.array(image)
    # feed in the colors that we have selected.

    # converting the Hexadecimal colors to RBG colours.
    custom_colors = []
    for i in range(len(colors)):
        rgb_color = ImageColor.getcolor(colors[i], 'RGB')
        custom_colors.append(rgb_color)

    # Reshape the image data to a 2D array of RGB values
    print("reshaping image")
    pixels = image_np.reshape(-1, 3)
    
    print("quantizing pixels")
    # Create a new image with the specified color centers
    quantized_pixels = [find_nearest_color(pixel, custom_colors) for pixel in pixels] # TODO fix the computational complexity of this line.
    print("quantizing image")
    quantized_image = np.array(quantized_pixels).reshape(image_np.shape).astype(np.uint8)
    print("constructing image from array")
    data = Image.fromarray(quantized_image)
    print("saving image data to path")
    data.save(save_path)

