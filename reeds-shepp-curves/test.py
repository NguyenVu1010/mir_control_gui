import turtle
import math
import random as rd

# drawing n units (eg turtle.forward(n)) will draw n * SCALE pixels
SCALE = 1

def scale(x):
    """
    Scale the input coordinate(s).
    """
    if isinstance(x, (tuple, list)):
        return [p * SCALE for p in x]
    return x * SCALE

def unscale(x):
    """
    Unscale the input coordinate(s).
    """
    if isinstance(x, (tuple, list)):
        return [p / SCALE for p in x]
    return x / SCALE

def vec(bob):
    """
    Draw an arrow.
    """
    bob.down()
    bob.pensize(3)
    bob.forward(scale(1.2))
    bob.right(25)
    bob.backward(scale(.4))
    bob.forward(scale(.4))
    bob.left(50)
    bob.backward(scale(.4))
    bob.forward(scale(.4))
    bob.right(25)
    bob.pensize(1)
    bob.up()

def goto(bob, pos, scale_pos=True):
    """
    Go to a position without drawing.
    """
    bob.up()
    if scale_pos:
        bob.setpos(scale(pos[:2]))
    else:
        bob.setpos(pos[:2])
    bob.setheading(pos[2])
    bob.down()

def draw_path(bob, path):
    """
    Draw the path (list of rs.PathElements).
    """
    current_position = [0, 0]  # Initial position (x, y)
    current_orientation = 0  # Initial orientation in radians
    path_coordinates = []  # List to store the coordinates of points

    for subpath in path:
        for segment in subpath:  # Iterate over each segment in the subpath
            gear = 1 if segment['Gear'] == 'FORWARD' else -1  # Access 'Gear' from dict
            if segment['Steering'] == 'LEFT':
                # Draw arc for left turn
                radius = scale(1)
                angle = gear * segment['distance'] # Convert degrees to radians
                bob.circle(radius, angle)
                # Update current position and orientation
                arc_length = abs(angle) * radius
                current_position[0] += arc_length * math.cos(current_orientation + angle / 2)
                current_position[1] += arc_length * math.sin(current_orientation + angle / 2)
                current_orientation += angle
            elif segment['Steering'] == 'RIGHT':
                # Draw arc for right turn
                radius = -scale(1)
                angle = gear * segment['distance']
                bob.circle(radius, angle)
                # Update current position and orientation
                arc_length = abs(angle) * radius
                current_position[0] += arc_length * math.cos(current_orientation + angle / 2)
                current_position[1] += arc_length * math.sin(current_orientation + angle / 2)
                current_orientation += angle
            elif segment['Steering'] == 'STRAIGHT':
                # Draw straight line
                bob.forward(gear * scale(segment['distance']))
                current_position[0] += gear * scale(segment['distance']) * math.cos(current_orientation)
                current_position[1] += gear * scale(segment['distance']) * math.sin(current_orientation)

            # Add current position to the coordinates list
            path_coordinates.append(tuple(current_position))  # Store the current position
    
    return path_coordinates  # Return the list of coordinates

def set_random_pencolor(bob):
    """
    Draws noodles.
    """
    r, g, b = 1, 1, 1
    while r + g + b > 2.5:
        r, g, b = rd.uniform(0, 1), rd.uniform(0, 1), rd.uniform(0, 1)
    bob.pencolor(r, g, b)

# Example path (replace with your actual path)
path = [
    [{'Steering': 'RIGHT', 'Gear': 'FORWARD', 'distance': 0.01}, {'Steering': 'STRAIGHT', 'Gear': 'FORWARD', 'distance': 13.63}, {'Steering': 'LEFT', 'Gear': 'FORWARD', 'distance': 0.63}],
    [{'Steering': 'RIGHT', 'Gear': 'FORWARD', 'distance': 0.01}, {'Steering': 'LEFT', 'Gear': 'FORWARD', 'distance': 0.24}, {'Steering': 'RIGHT', 'Gear': 'BACKWARD', 'distance': 0.09}],
    [{'Steering': 'RIGHT', 'Gear': 'FORWARD', 'distance': 0.0}, {'Steering': 'STRAIGHT', 'Gear': 'FORWARD', 'distance': 1.48}, {'Steering': 'LEFT', 'Gear': 'FORWARD', 'distance': 0.11}],
    [{'Steering': 'RIGHT', 'Gear': 'FORWARD', 'distance': 0.08}, {'Steering': 'LEFT', 'Gear': 'FORWARD', 'distance': 0.56}, {'Steering': 'RIGHT', 'Gear': 'BACKWARD', 'distance': 0.2}],
    [{'Steering': 'RIGHT', 'Gear': 'FORWARD', 'distance': 0.19}, {'Steering': 'LEFT', 'Gear': 'BACKWARD', 'distance': 0.23}, {'Steering': 'RIGHT', 'Gear': 'FORWARD', 'distance': 0.11}],
    [{'Steering': 'RIGHT', 'Gear': 'FORWARD', 'distance': 0.03}, {'Steering': 'STRAIGHT', 'Gear': 'FORWARD', 'distance': 4.58}, {'Steering': 'LEFT', 'Gear': 'FORWARD', 'distance': 0.57}],
    [{'Steering': 'RIGHT', 'Gear': 'BACKWARD', 'distance': 0.11}, {'Steering': 'LEFT', 'Gear': 'FORWARD', 'distance': 0.46}, {'Steering': 'RIGHT', 'Gear': 'BACKWARD', 'distance': 0.23}],
    [{'Steering': 'RIGHT', 'Gear': 'FORWARD', 'distance': 0.04}, {'Steering': 'STRAIGHT', 'Gear': 'FORWARD', 'distance': 2.18}, {'Steering': 'LEFT', 'Gear': 'FORWARD', 'distance': 0.45}],
    [{'Steering': 'LEFT', 'Gear': 'FORWARD', 'distance': 0.09}, {'Steering': 'STRAIGHT', 'Gear': 'FORWARD', 'distance': 10.3}, {'Steering': 'RIGHT', 'Gear': 'FORWARD', 'distance': 1.56}]
]

# Setup the turtle screen
screen = turtle.Screen()
screen.setup(width=600, height=600)

bob = turtle.Turtle()

# Draw the path and get the coordinates
coordinates = draw_path(bob, path)

# Print the coordinates of the path
print("Coordinates of the path:")
for coord in coordinates:
    print(coord)

# Hide the turtle and display the result
bob.hideturtle()
screen.mainloop()
