def image_to_svg(vector_file_save_path, image_file_load_path, TraceOptions=TraceOptions(despeckle_level=5, despeckle_tightness=7.0)):

    # load an image

    PIL_image = Image.open(image_file_load_path).convert("RGB")

    #print the dimensions of the image
    width, height = PIL_image.size
    print("width: ", width, ", height: ", height)

    image = np.asarray(PIL_image)

    # create a bitmap  
    bitmap = Bitmap(image)

    # trace the bitmap.
    vector = bitmap.trace(options=TraceOptions)

    #save the vector as an SVG
    vector.save(vector_file_save_path)

    # get an 

    # bitmap = Bitmap(quantized_image)

    # # https://github.com/autotrace/autotrace
    # options = TraceOptions(despeckle_level=5, despeckle_tightness=7.0)
    # vector = bitmap.trace(options=options)
    # vector.save("../images/outputs/vector4.svg")


#
# # Load the image using Pillow
# image = Image.open('../images/sample_images/colourwheel.jpeg')
#
# # Convert the image to a NumPy array
# image_np = np.array(image)
#
# #feed in the manual colours:
# custom_colors = [
#     (255,255,0),
#     (255,0,0),
#     (0,0,255)
# ]
#
# # Reshape the image data to a 2D array of RGB values
# pixels = image_np.reshape(-1, 3)
#
# # Find the nearest custom color for each pixel
# def find_nearest_color(pixel):
#     min_distance = float('inf')
#     nearest_color = None
#     for color in custom_colors:
#         distance = np.linalg.norm(np.array(pixel) - np.array(color))
#         if distance < min_distance:
#             min_distance = distance
#             nearest_color = color
#     return nearest_color
#
# # Create a new image with the specified color centers
# quantized_pixels = [find_nearest_color(pixel) for pixel in pixels]
# quantized_image = np.array(quantized_pixels).reshape(image_np.shape).astype(np.uint8)
#
# # Save the quantized image
# quantized_image_non_vector = Image.fromarray(quantized_image)
# quantized_image_non_vector.save('../images/outputs/quantized_image_2.jpg')
#
# # # pypotrace version.
# # #create bitmap from the array
# # bmp = potrace.Bitmap(quantized_image)
# #
# # #trace the bitmap to a path
# # path = bmp.trace()
# #
# # # iterate over path curves
# # for curve in path:
# #     print("start_point = ", curve.start_point)
# #     for segment in curve:
# #             print(segment)
# #             end_point_x, end_point_y = segment.end_point
#
# #autotrace version
#
# bitmap = Bitmap(quantized_image)

# # https://github.com/autotrace/autotrace
# options = TraceOptions(despeckle_level=5, despeckle_tightness=7.0)
# vector = bitmap.trace(options=options)
# vector.save("../images/outputs/vector4.svg")