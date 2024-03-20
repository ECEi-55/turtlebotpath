import tkinter as tk
from PIL import Image, ImageTk
from tkinter import filedialog, colorchooser
import customtkinter
import rclpy
from irobot_create_msgs.action import DriveDistance, RotateAngle
from sensor_msgs.msg import BatteryState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from turtlebot4_node_interfaces.srv import Drive, Rotate
import math
from rclpy.action import ActionClient
import time
from enum import Enum
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSPresetProfiles, QoSReliabilityPolicy, QoSHistoryPolicy
from PIL import Image, ImageTk, ImageColor
from functools import partial
import numpy as np
from autotrace import Bitmap, TraceOptions
import svgwrite #depricated suff stops it from working
import drawsvg as draw
import svgpathtools as svgpt
from svgpathtools import svg2paths, CubicBezier, Line, Path
from svglib.svglib import svg2rlg
from reportlab.graphics import renderPM
import cairosvg
import io

class signal_t(Enum):
    START = 0
    RAISE = 1
    STOP = 2
    RESET = 3
    LOWER_LIMIT = 4
    UPPER_LIMIT = 5

class state_t(Enum):
    IDLE = 0
    ACTIVE = 1
    RETRACT = 2
    STOW = 3
    EMPTY = 4

class TurtleBot4Sub(Node):
    def __init__(self):
        super().__init__('TurtleBot4Node_UI_SUB')

        self.qosp = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        self.debug_sub_ = self.create_subscription(String, 'debug', self.current_state_callback, 10)
        self.battery_sub_ = self.create_subscription(BatteryState, 'battery_state', self.battery_state_callback, self.qosp)
        
        self.debug_info = String()
        self.battery_status = BatteryState()

    def current_state_callback(self, event):
        self.debug_info = event

    def battery_state_callback(self, event):
        self.battery_status = event

    def get_battery_status(self):
        return self.battery_status.percentage * 100
    
    def get_state_status(self):
        return self.debug_info

class TurtleBot4Node(Node):
    def __init__(self):
        super().__init__('TurtleBot4Node_UI')
        self.chalk_signal_pub_ = self.create_publisher(Int32, 'signal', 10)
        
        self.cmd_vel_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.drive_cli = ActionClient(self, DriveDistance, 'drive_distance')
        # self.drive_cli = self.create_client(DriveDistance, 'drive_distance')
        # while not self.drive_cli.wait_for_server(timeout_sec=1.0):
        #     self.get_logger().info('drive server not available, waiting again...')
        self.drive_req = DriveDistance.Goal()
        
        self.rotate_cli = ActionClient(self, RotateAngle, 'rotate_angle')
        # self.rotate_cli= self.create_client(RotateAngle, 'rotate_angle')
        # while not self.rotate_cli.wait_for_server(timeout_sec=1.0):
        #     self.get_logger().info('rotate server not available, waiting again...')
        self.rotate_req = RotateAngle.Goal()

    def send_drive_request(self, distance):
        self.drive_req.distance = distance
        self.future = self.drive_cli.send_goal_async(self.drive_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future
    
    def send_rotate_requst(self, angle):
        self.rotate_req.angle = angle
        self.future = self.rotate_cli.send_goal_async(self.rotate_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future

    def send_signal(self, signal):
        chalk_signal_req = Int32()
        chalk_signal_req.data = signal
        self.chalk_signal_pub_.publish(chalk_signal_req)


class App:
    '''
    working_image_directory is the file path to the directory where the temporary image processing results are held.
    '''

    def __init__(self, app, args, working_image_directory):
        self.app = app
        rclpy.init(args=args)
        self.node = TurtleBot4Node()
        self.subnode = TurtleBot4Sub()
        self.working_image_directory = working_image_directory

        self.pointso = [
            [0,0],[0,0.5], 

            [0,0.5],[0.5,0.5],

            [0.5,0.5],[0.5,0],

            [0.5,0],[0,0]
        ]

        self.points = [
[0.0, 0.40789473684210525],
[0.3968253968253968, 0.3157894736842105],
[0.3968253968253968, 0.3157894736842105],
[0.5, 0.0],
[0.5, 0.0],
[0.6507936507936508, 0.1118421052631579],
[0.6507936507936508, 0.1118421052631579],
[0.5396825396825397, 0.4407894736842105],
[0.5396825396825397, 0.4407894736842105],
[0.14285714285714285, 0.5328947368421053],
[0.14285714285714285, 0.5328947368421053],
[0.0, 0.40789473684210525],
[0.18253968253968253, 0.5592105263157895],
[0.5714285714285714, 0.46710526315789475],
[0.5714285714285714, 0.46710526315789475],
[0.7142857142857143, 0.5855263157894737],
[0.7142857142857143, 0.5855263157894737],
[0.3253968253968254, 0.6710526315789473],
[0.3253968253968254, 0.6710526315789473],
[0.21428571428571427, 1.0],
[0.21428571428571427, 1.0],
[0.07936507936507936, 0.881578947368421],
[0.07936507936507936, 0.881578947368421],
[0.18253968253968253, 0.5592105263157895],
[0.36507936507936506, 0.7039473684210527],
[0.753968253968254, 0.618421052631579],
[0.753968253968254, 0.618421052631579],
[0.8650793650793651, 0.2894736842105263],
[0.8650793650793651, 0.2894736842105263],
[1.0, 0.40789473684210525],
[1.0, 0.40789473684210525],
[0.8968253968253969, 0.7368421052631579],
[0.8968253968253969, 0.7368421052631579],
[0.5079365079365079, 0.8289473684210527],
[0.5079365079365079, 0.8289473684210527],
[0.36507936507936506, 0.7039473684210527]]

        self.battery_text = customtkinter.CTkLabel(app, text="%0")
        # self.battery_text.pack()
        # app.after(1000, self.battery_percent)

        self.debug_text = customtkinter.CTkLabel(app, text="Debug Info: ")
        # self.debug_text.pack()
        # app.after(1000, self.debug_show)

        self.offset = 0.23
        
        app.bind("w", self.forward_handler)
        app.bind("s", self.back_handler)
        app.bind("a", self.left_handler)
        app.bind("d", self.right_handler)
        app.bind("r", self.raise_handler)
        app.bind("f", self.lower_handler)
        app.bind("x", self.reset_handler)

        # set up the frames

        # control frame, for the left side, holds the start, stop, pause etc.

        self.root_frm = customtkinter.CTkFrame(app)
        self.root_frm.grid(row=0, column=0, sticky="nsew")
        app.rowconfigure(0, weight=1)
        app.columnconfigure(0, weight=1)

        self.control_frm = customtkinter.CTkFrame(self.root_frm)
        self.control_frm.grid(row=0, column=0, padx=20, pady=20, sticky="ew")

        # image processing frame, hold the image processing stuff.

        self.image_setup_frm = customtkinter.CTkFrame(self.root_frm)
        self.image_setup_frm.rowconfigure(0, weight=1)
        self.image_setup_frm.rowconfigure(1, weight=1)
        self.image_setup_frm.grid(row=0, column=1, padx=20, pady=20, sticky="ew")

        self.color_button_frm = customtkinter.CTkFrame(self.image_setup_frm)
        self.color_button_frm.grid(row=0, column=1, padx=20, pady=20, sticky="ew")
        self.images_frm = customtkinter.CTkFrame(self.image_setup_frm)
        self.images_frm.grid(row=0, column=2, padx=20, pady=20, sticky="ew")

        self.startButton = customtkinter.CTkButton(self.control_frm, text="Start Drawing", command=self.startButtonCallback)
        self.startButton.pack(pady=10)

        self.stopButton = customtkinter.CTkButton(self.control_frm, text="Stop Drawing", command=self.stopButtonCallback)
        self.stopButton.pack(pady=10)

        self.progressbar = customtkinter.CTkProgressBar(self.control_frm, orientation="horizontal")
        self.progressbar.pack(pady=10)
        self.progressbar.set(0)

        self.control_robot_label = customtkinter.CTkLabel(self.control_frm, text="w: forward\ns: backward\na: left\n d: right\n r: raise chalk\n f: lower chalk")
        self.control_robot_label.pack(pady=10)

        # colour selection buttons
        self.image_colours = [None, None, None, None]
        self.colour_buttons = [None, None, None, None]

        for i in range(0,len(self.colour_buttons)):
            self.colour_buttons[i] = customtkinter.CTkButton(self.color_button_frm, text=("Colour " + str(i+1) ), command = partial(self.pick_colour, i))
            self.colour_buttons[i].pack(pady=10)

        # image stuff
        # self.working_image_directory = '../turtlebot4_ui' #TODO change this to be a proper gitignored directory
        self.selected_image_path = None
        self.selected_image_label = customtkinter.CTkLabel(self.images_frm)
        self.colour_reduced_image_path = None
        self.colour_reduced_image_label = customtkinter.CTkLabel(self.images_frm)
        self.svg_image_path = None
        self.svg_image_label = customtkinter.CTkLabel(self.images_frm)
        self.generated_path_image_path = None
        self.generated_path_image_label = customtkinter.CTkLabel(self.images_frm)

        self.upload_image_button = customtkinter.CTkButton(self.color_button_frm, text="Upload Image", command=self.upload_image)
        self.upload_image_button.pack(pady=10)
    
    def pick_colour(self, button_index):
        color = colorchooser.askcolor()[1]
        print("Color: ", color)
        self.colour_buttons[button_index].configure(fg_color=color)
        self.image_colours[button_index] = color
    
    def upload_image(self):
        file_path = filedialog.askopenfilename(title="Select Image")#, filetypes=[("Image Files", "*.png;*.jpg;*.jpeg")])
        if file_path: 
            # display the original image
            image = Image.open(file_path)
            image = image.resize((300,300))
            photo = customtkinter.CTkImage(image, size=(200,200))
            self.selected_image_path = file_path
            self.selected_image_label.configure(image=photo, text="")
            self.selected_image_label.pack(pady=20)


            # colour reduce the image:
            self.colour_reduced_image_path = self.working_image_directory + "colour_reduced_image.png"

            colors = self.image_colours

            self.color_reduce(file_path, self.colour_reduced_image_path, colors)

            image = Image.open(self.colour_reduced_image_path)
            image = image.resize((300,300))
            photo = customtkinter.CTkImage(image, size=(200,200))
            self.colour_reduced_image_label.configure(image=photo, text="")
            self.colour_reduced_image_label.pack(pady=20)

            # svg autotrace
            options =  TraceOptions(
                corner_always_threshold=0, #30
                corner_surround=20, #40
                corner_threshold=165,
                # despeckle_level=20, #20 is max
                # despeckle_tightness=8.0, # is max, 2.0 is default. 
                error_threshold=1.0, #2.0 is default. 
                filter_iterations=100, #default is 4 smoothenings.
                line_reversion_threshold=50,
                line_threshold=100,
                remove_adjacent_corners=True,
                tangent_surround=4
            ) 

            # self.svg_image_path = "../images/sample_images/test_square.svg"
            self.svg_image_path = "/home/ros/generated_vector.svg"
            self.generated_path_image_path = "/home/ros/generated_vector_from_path.svg"
            path = self.generate_path(self.svg_image_path,1.0,1.0, 0.0127, 0.15)
            self.path_to_svg(path=path, svg_save_path=self.generated_path_image_path, chalk_width=0.0127, pixel_scale=500)
            image_data = cairosvg.svg2png(url=self.generated_path_image_path)
            image = Image.open(io.BytesIO(image_data))
            image = image.resize((300,300))
            photo = customtkinter.CTkImage(image, size=(200,200))
            self.generated_path_image_label.configure(image=photo, text="")
            self.generated_path_image_label.pack(pady=20)

    def find_nearest_color(self, pixel, custom_colors):
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


    def color_reduce(self, image_path, save_path, colors):
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
        quantized_pixels = [self.find_nearest_color(pixel, custom_colors) for pixel in pixels] # TODO fix the computational complexity of this line.
        print("quantizing image")
        quantized_image = np.array(quantized_pixels).reshape(image_np.shape).astype(np.uint8)
        print("constructing image from array")
        data = Image.fromarray(quantized_image)
        print("saving image data to path to " + save_path) 
        print(data.save("/home/ros/colour_reduced_image.png"))
    
    def image_to_svg(self, vector_file_save_path, image_file_load_path, TraceOptions=TraceOptions(despeckle_level=5, despeckle_tightness=7.0)):

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

    def parse_color_code(self, input_str):
        '''
        parses the colour code from the style field of the auto generated svg
        '''

        start_index = input_str.find("#")
        if start_index != -1:
            #extract the substring starting from "#" until the next ';'
            end_index = input_str.find(';', start_index)
            if end_index != -1:
                color_code = input_str[start_index:end_index]
                return color_code
        return None

    '''

    inputs:

    svg: the input SVG image that we want to generate a path for 
    draw_area_width : the width in meters of the drawing area
    draw_area_height : the height in meters of the drawing area
    chalk_width : the diameter of the chalk in meters

    output:

    A path list. 
    each item in the format (start position, end position, colour)
    '''
    def generate_path(self, svg,draw_area_width, draw_area_height, chalk_width, min_exclude_dimension):

        #load the SVG to get the paths and attributes
        paths, attributes = svg2paths(svg)

        # first_path = paths[0]

        # handle the scaling of the SVG to the desired size
        largest_x = 0
        largest_y = 0
        for path in paths:
            xmin, xmax, ymin, ymax = path.bbox()
            if xmax > largest_x:
                largest_x = xmax
            if ymax > largest_y:
                largest_y = ymax

        # TODO make this work over all paths. right now it only takes the first path into account.
        x_greater = (largest_x > largest_y)
        scale_factor = None
        if x_greater:
            scale_factor = draw_area_width / largest_x
        else:
            scale_factor = draw_area_height / largest_y

        # scale each path in place
        for i in range(0, len(paths)):
            paths[i] = paths[i].scaled(scale_factor, scale_factor)

        # print(first_path)
        # first_segment = first_path[0]


        #then scale them to the desired size of area

        # transform or scaled methods?

        # then for each shape, we do a PCA to find the direction of maximum variance (or at least a close approximation)

        # simplest method would be to just start with an offset from the first point with the chalk diameter, then go along that line with the same slope until we find an intersection with another segment
        # then from that intersection, we offset the same amount as the original, find another intersection, set the cursor there and start going the other direction.

        # TODO use the principal component analysis to find the guideline instead of using the first segment

        # TODO loop through all the paths:
        chalk_lines = []

        # for a single path
        for p in range(0, len(paths)):
            
            # negation for paths that are too small
            xmin, xmax, ymin, ymax = paths[p].bbox()
            xdiff = xmax-xmin
            ydiff = ymax-ymin
            # print(f"(xdiff:{xdiff}, ydiff:{ydiff})")
            if xdiff < min_exclude_dimension or ydiff < min_exclude_dimension:
                # print(f"excluded dimension hit (xdiff:{xdiff}, ydiff:{ydiff})")
                continue
            
            # Also need to do some background detection. 
            if xdiff > draw_area_width - draw_area_width*0.1 or ydiff > draw_area_height - draw_area_height*0.1: 
                print("found very large shape")
                fill = attributes[p]["style"]
                fill = self.parse_color_code(fill)
                print("shape has this colour: ", fill)
                # this is the background, so exclude it. 
                continue

            print("starting path #", p)

            path_chalk_lines = []
            # pick the first segment as the guide line:
            first_segment = paths[p][0]
            guide_line = first_segment
            #scale the guide line to make it much longer than the object

            # find the slope of the line
            rise = guide_line[1].imag - guide_line[0].imag
            run = guide_line[1].real - guide_line[0].real

            #TODO change this to make it proper, instead of faking it
            if run == 0:
                run = 0.0001

            if run != 0:
                slope = rise/run
                y_intercept = guide_line[0].imag - slope*guide_line[0].real

                # was this supposed to give a horizontal line? 
                # new_start = complex(-1000, slope*(100)+y_intercept)
                # new_end = complex(1000, slope*(100)+y_intercept)

                new_start = complex(-1000, ymin)
                new_end = complex(1000, ymin)

                guide_line = Line(start=new_start, end=new_end)
            else:
                # handle the case where there is no run, can't divide by zero.
                
                new_start = complex(0, -1000)
                new_end = complex(0, 1000)

            # print("got past guide line creation, guide line: ", guide_line)

            # repeatedly add an offset to the guideline, find the two intersections, draw a line between them and add that to the chalk lines
            # stop when no intersections can be found.
            
            print("length of this path in segments: ", len(paths[p]))
            print(f"(xdiff:{xdiff}, ydiff:{ydiff})")

            fill = attributes[p]["style"]
            fill = self.parse_color_code(fill)
            print("fill for this path sement: ", fill)

            first_iteration = True
            last_point = None
            while True:
                # add offset to the guide line to make the new guide line
                offset = complex(0, chalk_width)
                if first_iteration:
                    offset = complex(0, chalk_width/2)

                #TODO change this so that the offset is perpendicular to the line, not a vertical change. 
                # use the cross product with the z axis and the guide line. 
                # atan2 function to get angle. 

                guide_line = guide_line.translated(offset)
                print(guide_line)

                # find all the intersections of that guide line with the path
                intersections = []
                # for path in paths[p]:
                #     print(guide_line.intersect(path))
                #     T1 = guide_line.intersect(path)
                #     intersections.append(guide_line.point(T1))

                for (T1, seg1, t1), (T2, seg2, t2) in paths[p].intersect(guide_line):
                    intersections.append(paths[p].point(T1))
                
                if len(intersections) == 0:
                    break
                elif len(intersections) != 2: #TODO modify it to handle multiple intersections. because the logo needs it. 
                    print(f"got {len(intersections)} intersections")
                    print("is this the first iteration: ", first_iteration)
                    fill = attributes[p]["style"]
                    fill = self.parse_color_code(fill)
                    print("shape has this colour: ", fill)
                    # print("length of this path in segments: ", len(paths[p]))
                    raise Exception("Got more than 2 intersections. Our software at the moment is not designed to handle this")
                
                # for the auto generated version we need to parse the style field for the fill colour. 
                fill = attributes[p]["style"]
                fill = self.parse_color_code(fill)
                if fill == None:
                    raise Exception("wrong fill style format for svg")
                
                chalk_line = None
                if first_iteration:
                    # it doesn't really matter
                    # print(f"intersections[0]: ({intersections[0].real},{intersections[0].imag})")
                    # print(f"intersections[1]: ({intersections[1].real},{intersections[1].imag})")
                    chalk_line = (intersections[0].real, intersections[0].imag, intersections[1].real, intersections[1].imag, fill)
                    last_point = intersections[1]
                else:
                    # find which intersection is closest to the last value in path_chalk_lines
                    dist_0 = abs(last_point - intersections[0])
                    dist_1 = abs(last_point - intersections[1])
                    if dist_0 < dist_1:
                        chalk_line = (intersections[0].real, intersections[0].imag, intersections[1].real, intersections[1].imag, fill)
                        last_point = intersections[1]
                    else:
                        chalk_line = (intersections[1].real, intersections[1].imag, intersections[0].real, intersections[0].imag, fill)
                        last_point = intersections[0]

                # append these intersections to the chalk_lines for this path
                # print(chalk_line)
                path_chalk_lines.append(chalk_line)
                # print(path_chalk_lines)

                first_iteration = False

            # chalk_lines = path_chalk_lines #TODO replace this
            chalk_lines += path_chalk_lines
            print("new length of chalk lines: ", len(chalk_lines))


            # then we find the lines to generate the fill (linear hatching) along that direction.
            # then convert that to path information by joining the shapes that have closest end points

        return chalk_lines

    '''
    This function is to generate a new SVG from the path extracted from the image's SVG. It is used for debug and confirmation purposes. 

    inputs:

    path (set of path points as defined earlier) 
    svg_save_path : the path to save the resulting SVG
    chalk_width : the width of the chalk in meters (diameter)
    pixel_scale : how many pixels per meter for the output drawing svg. (shouldn't matter too much since the end application will resize anyways.)

    outputs:

    svg image saved to given path. 

    '''
    def path_to_svg(self, path,svg_save_path, chalk_width, pixel_scale):

        line_list = [None] * len(path) # creating an empty list which will be used to store the draw.Line element for each.

        #while we do this pass, keep track of the max x and y values
        max_x = 0
        max_y = 0

        for i in range(0,len(path)):
            line = path[i]
            line_list[i] = draw.Line(line[0], line[1], line[2], line[3], fill=None, stroke=line[4], stroke_width=chalk_width)

            if line[0] > max_x:
                max_x = line[0]
            if line[2] > max_x:
                max_x = line[2]
            if line[1] > max_y:
                max_y = line[1]
            if line[3] > max_y:
                max_y = line[3]

        max_x = max_x + chalk_width/2
        max_y = max_y + chalk_width/2

        d = draw.Drawing(max_x,max_y)

        for line in line_list:
            d.append(line)

        # in this new SVG, it's assumed that every user pixel is 1 meter
        d.set_pixel_scale(pixel_scale)  # Set number of pixels per geometry unit
        d.save_svg(svg_save_path)

    def battery_percent(self):
        # rclpy.spin_once(self.subnode)
        # self.battery_text.configure(text=("%" + str(int(self.subnode.get_battery_status()))))
        self.app.after(1000, self.battery_percent)
    
    def debug_show(self):
        # rclpy.spin_once(self.subnode)
        # self.debug_text.configure(text=("Debug Info: " + str(self.subnode.get_state_status())))
        self.app.after(1000, self.debug_show)

    def forward_handler(self, event):
        move = Twist()
        move.linear.x = 1.0
        self.node.cmd_vel_pub_.publish(move)

    def back_handler(self, event):
        move = Twist()
        move.linear.x = -0.25
        self.node.cmd_vel_pub_.publish(move)
    
    def left_handler(self, event):
        move = Twist()
        move.angular.z = 3.1415/4.0
        self.node.cmd_vel_pub_.publish(move)
    
    def right_handler(self, event):
        move = Twist()
        move.angular.z = -3.1415/4.0
        self.node.cmd_vel_pub_.publish(move)
    
    def raise_handler(self, event):
        self.node.send_signal(signal_t.RAISE.value)

    def lower_handler(self, event):
        self.node.send_signal(signal_t.START.value)
     
    def reset_handler(self, event):
        self.node.send_signal(signal_t.RESET.value)

    def startButtonCallback(self):
        print("Starting Print")
        self.path()
        
    def stopButtonCallback(self):
        self.node.send_signal(signal_t.STOP.value)
        self.node.destroy_node()
        self.subnode.destroy_node()
        rclpy.shutdown()
        exit()
    
    def path(self):
        previous_pos = [0,0]
        previous_pos[0] = self.points[0][0]
        previous_pos[1] = self.points[0][1]
        previous_dir = [0,1]

        self.node.send_signal(signal_t.START.value)
        time.sleep(15)

        for i in range(0, len(self.points), 2):
            stroke_vector = [0,0]
            stroke_vector[0] = self.points[i+1][0] - self.points[i][0]
            stroke_vector[1] = self.points[i+1][1] - self.points[i][1]
            print("stroke_vector: ", stroke_vector)

            stroke_vector_mag = math.sqrt(math.pow(stroke_vector[0],2) + math.pow(stroke_vector[1],2))
            print("stroke_vector_mag: ", stroke_vector_mag)
            
            stroke_vector_norm = [0,0]
            stroke_vector_norm[0] = stroke_vector[0]/stroke_vector_mag
            stroke_vector_norm[1] = stroke_vector[1]/stroke_vector_mag

            offset_vector = [0,0]
            offset_vector[0] = self.offset * stroke_vector_norm[0]
            offset_vector[1] = self.offset * stroke_vector_norm[1]
            print("Offset vector: ", offset_vector)

            p1 = [0,0]
            p1[0] = self.points[i][0] + offset_vector[0]
            p1[1] = self.points[i][1] + offset_vector[1]
            print("P1: ", p1)

            p2 = [0,0]
            p2[0] = self.points[i+1][0] + offset_vector[0]
            p2[1] = self.points[i+1][1] + offset_vector[1]
            print("P2: ", p2)

            prev_to_p1_vector = [0,0]
            prev_to_p1_vector[0] = p1[0] - previous_pos[0]
            prev_to_p1_vector[1] = p1[1] - previous_pos[1]
            print("prev_to_p1_vector: ", prev_to_p1_vector)

            prev_to_p1_mag = math.sqrt(math.pow(prev_to_p1_vector[0], 2) + math.pow(prev_to_p1_vector[1],2))
            prev_to_p1_vector_norm = [0,0]
            prev_to_p1_vector_norm[0] = prev_to_p1_vector[0] / prev_to_p1_mag
            prev_to_p1_vector_norm[1] = prev_to_p1_vector[1] / prev_to_p1_mag
            print("prev_to_p1_vector_norm: ", prev_to_p1_vector_norm)

            prev_to_p1_cross_mag = (previous_dir[0] * prev_to_p1_vector_norm[1]) - (prev_to_p1_vector_norm[0] * previous_dir[1])

            theta = math.atan2(prev_to_p1_cross_mag, self.dot(prev_to_p1_vector_norm, previous_dir))
            print("Theta(1): ", theta)
            
            self.node.send_signal(signal_t.RAISE.value)
            time.sleep(4)
            self.node.send_rotate_requst(theta)
            time.sleep(4)
            self.node.send_drive_request(prev_to_p1_mag)
            time.sleep(4)

            p1_to_stroke_cross_mag = (prev_to_p1_vector_norm[0] * stroke_vector_norm[1]) - (stroke_vector_norm[0] * prev_to_p1_vector_norm[1])
            print("P1_to_stroke_cross_mag: ", p1_to_stroke_cross_mag)

            theta = math.atan2(p1_to_stroke_cross_mag, self.dot(stroke_vector_norm, prev_to_p1_vector_norm))
            print("Theta(2): ", theta)
            
            self.node.send_rotate_requst(theta)
            time.sleep(4)
            self.node.send_signal(signal_t.START.value)
            time.sleep(4)
            self.node.send_drive_request(stroke_vector_mag)
            time.sleep(4)

            previous_dir[0] = stroke_vector_norm[0]
            previous_dir[1] = stroke_vector_norm[1]
            previous_pos[0] = p2[0]
            previous_pos[1] = p2[1]

            self.progressbar.set(i+1/len(self.points))
        self.node.send_signal(signal_t.STOP.value)

 
    def dot(self, a, b):
        return a[0]*b[0] + a[1]*b[1]


def main(args=None):
    customtkinter.set_appearance_mode("dark")
    root = customtkinter.CTk()
    # root.geometry("1024x800")
    app = App(root, args, working_image_directory="/home/ros/")
    root.mainloop()

if __name__ == "__main__":
    main()

