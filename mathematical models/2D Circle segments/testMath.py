import math
import matplotlib.pyplot as plt

# Given values from the sensors
sensor1X = -30
sensor2X = -20
sensor3X = -10
sensor4X = 10
sensor5X = 30
 
#tape setup measurements
arc_length = 10

def calculate_offset(previousX, previousY, sensorA, sensorB, arc_length):
    
    sensorA = math.radians(sensorA)
    sensorB = math.radians(sensorB)
    print(f"alpha: {sensorA:.3f}, beta: {sensorB:.3f}")
    
    theta = abs(sensorA - sensorB)
    print(f"theta: {theta:.3f}")    
    
    # Calculate the radius
    radius = arc_length / theta
    
    # Calculate the coordinates of the centre of the circle
    x_s = radius * math.cos(sensorA)
    y_s = radius * math.sin(sensorA)
    
    # Calculate the coordinates of the point S
    x_p = x_s - radius * math.cos(sensorB)
    y_p = y_s - radius * math.sin(sensorB)
    
    x_point = x_p + previousX
    y_point = previousY + abs(y_p)
   
    print(f"radius: {radius:.3f}")
    print(f"Coordinates middle point of circle: ({x_s:.3f}, {y_s:.3f})") 
    print(f"Coordinates opposed to previous point: ({x_p:.3f}, {y_p:.3f})") 
    print(f"Coordinates of point S: ({x_point:.3f}, {y_point:.3f})") 
    print("---------------------------------------------------------------")
    
    return x_point, y_point, radius

x1, y1, radius1 = calculate_offset(0, 0, sensor1X, sensor2X, arc_length)
x2, y2, radius2 = calculate_offset(x1, y1, sensor2X, sensor3X, arc_length)
x3, y3, radius3 = calculate_offset(x2, y2, sensor3X, sensor4X, arc_length)
x4, y4, radius4 = calculate_offset(x3, y3, sensor4X, sensor5X, arc_length)


# Create the scatter plot
plt.figure(figsize=(8, 8))

# Plot the center of the circle
#plt.scatter(x_s, y_s, color='blue', label='Circle Center')

# Plot point S
plt.scatter(0, 0, color='red', label='Point 1')
plt.scatter(x1, y1, color='blue', label='Point 2')
plt.scatter(x2, y2, color='orange', label='Point 3')
plt.scatter(x3, y3, color='green', label='Point 4')
plt.scatter(x4, y4, color='black', label='Point 5')
plt.xlim(-25, 25)
plt.ylim(-10, 40)

# Add labels and title
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()

