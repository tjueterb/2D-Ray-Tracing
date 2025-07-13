from pygame.locals import *
from operator import add, sub
import pygame
import sys
import math
import random

pygame.init()

# -----Options-----
WINDOW_SIZE = (1920, 1080) # Width x Height in pixels
NUM_RAYS = 250 # Must be between 1 and 360
SOLID_RAYS = False # Can be somewhat glitchy. For best results, set NUM_RAYS to 360
NUM_WALLS = 5 # The amount of randomly generated walls
ENABLE_REFLECTIONS = True # Enable first-order reflections
MAX_REFLECTIONS = 5 # Maximum number of reflections per ray
DEMO_MODE = True # Enable demo mode with controllable walls
#------------------

screen = pygame.display.set_mode(WINDOW_SIZE)
display = pygame.Surface(WINDOW_SIZE)

mx, my = pygame.mouse.get_pos()
lastClosestPoint = (0, 0)
running = True
rays = []
walls = []
particles = []

# Demo mode variables
emitter_x = 100
emitter_y = WINDOW_SIZE[1] - 100
controllable_wall_angle = 45  # degrees
controllable_wall_x = 400  # X position of controllable wall
controllable_wall_y = 400  # Y position of controllable wall

class Ray:
    def __init__(self, x, y, angle, reflection_depth=0):
        self.x = x
        self.y = y
        self.dir = (math.cos(angle), math.sin(angle))
        self.reflection_depth = reflection_depth

    def update(self, mx, my):
        self.x = mx
        self.y = my

    def checkCollision(self, wall):
        x1 = wall.start_pos[0]
        y1 = wall.start_pos[1]
        x2 = wall.end_pos[0]
        y2 = wall.end_pos[1]

        x3 = self.x
        y3 = self.y
        x4 = self.x + self.dir[0]
        y4 = self.y + self.dir[1]
    
        # Using line-line intersection formula to get intersection point of ray and wall
        # Where (x1, y1), (x2, y2) are the ray pos and (x3, y3), (x4, y4) are the wall pos
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        numerator = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
        if denominator == 0:
            return None
        
        t = numerator / denominator
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator

        if 1 > t > 0 and u > 0:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            collidePos = [x, y]
            return collidePos, wall
        return None

class Wall:
    def __init__(self, start_pos, end_pos, color = 'white', reflectance=0.7):
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.color = color
        self.reflectance = reflectance  # Reflectance value between 0 and 1 (0 = absorbs all light, 1 = reflects all light)
        self.slope_x = end_pos[0] - start_pos[0]
        self.slope_y = end_pos[1] - start_pos[1]
        if self.slope_x == 0:
            self.slope = 0
        else:
            self.slope = self.slope_y / self.slope_x
        self.length = math.sqrt(self.slope_x**2 + self.slope_y**2)

    def get_normal(self):
        """Get the normal vector of the wall (perpendicular to the wall)"""
        # Normalize the wall vector
        if self.length == 0:
            return (0, 1)  # Default normal for zero-length walls
        
        wall_unit_x = self.slope_x / self.length
        wall_unit_y = self.slope_y / self.length
        
        # Normal is perpendicular to the wall (rotate 90 degrees)
        normal_x = -wall_unit_y
        normal_y = wall_unit_x
        
        return (normal_x, normal_y)

    def reflect_ray(self, ray_dir):
        """Calculate the reflected ray direction given an incoming ray direction"""
        normal = self.get_normal()
        
        # Make sure normal points towards the incoming ray (not away from it)
        # If the dot product is positive, the normal is pointing the wrong way
        dot_product = ray_dir[0] * normal[0] + ray_dir[1] * normal[1]
        if dot_product > 0:
            normal = (-normal[0], -normal[1])
            dot_product = -dot_product
        
        # Reflection formula: r = d - 2(d·n)n
        # where d is the incident direction, n is the normal, r is the reflection
        reflected_x = ray_dir[0] - 2 * dot_product * normal[0]
        reflected_y = ray_dir[1] - 2 * dot_product * normal[1]
        
        return (reflected_x, reflected_y)

    def draw(self):
        pygame.draw.line(display, self.color, self.start_pos, self.end_pos, 3)

for i in range(0, 360, int(360/NUM_RAYS)):
    rays.append(Ray(mx, my, math.radians(i)))

def calculate_screen_intersection(ray):
    """Calculate where a ray intersects with the screen boundaries"""
    # Ray direction components
    dx = ray.dir[0]
    dy = ray.dir[1]
    
    # Ray starting position
    x0 = ray.x
    y0 = ray.y
    
    # Calculate intersection with each screen edge
    intersections = []
    
    # Left edge (x = 0)
    if dx != 0:
        t = (0 - x0) / dx
        if t > 0:  # Ray goes forward
            y = y0 + t * dy
            if 0 <= y <= WINDOW_SIZE[1]:
                intersections.append((0, y, t))
    
    # Right edge (x = WINDOW_SIZE[0])
    if dx != 0:
        t = (WINDOW_SIZE[0] - x0) / dx
        if t > 0:  # Ray goes forward
            y = y0 + t * dy
            if 0 <= y <= WINDOW_SIZE[1]:
                intersections.append((WINDOW_SIZE[0], y, t))
    
    # Top edge (y = 0)
    if dy != 0:
        t = (0 - y0) / dy
        if t > 0:  # Ray goes forward
            x = x0 + t * dx
            if 0 <= x <= WINDOW_SIZE[0]:
                intersections.append((x, 0, t))
    
    # Bottom edge (y = WINDOW_SIZE[1])
    if dy != 0:
        t = (WINDOW_SIZE[1] - y0) / dy
        if t > 0:  # Ray goes forward
            x = x0 + t * dx
            if 0 <= x <= WINDOW_SIZE[0]:
                intersections.append((x, WINDOW_SIZE[1], t))
    
    # Return the closest intersection (smallest t value)
    if intersections:
        closest = min(intersections, key=lambda intersection: intersection[2])
        return (closest[0], closest[1])
    else:
        # Fallback: extend ray a fixed distance
        return (x0 + dx * 2000, y0 + dy * 2000)

def drawRays(rays, walls, color = 'white'):
    global lastClosestPoint
    
    # Store all ray segments to draw them in the correct order
    ray_segments = []
    
    for ray in rays:
        current_ray = ray
        current_color = color
        
        # Track reflections for this ray
        for reflection_count in range(MAX_REFLECTIONS + 1):
            closest = 100000
            closestPoint = None
            closestWall = None
            
            for wall in walls:
                intersectResult = current_ray.checkCollision(wall)
                if intersectResult is not None:
                    intersectPoint, hitWall = intersectResult
                    # Get distance between ray source and intersect point
                    ray_dx = current_ray.x - intersectPoint[0]
                    ray_dy = current_ray.y - intersectPoint[1]
                    distance = math.sqrt(ray_dx**2 + ray_dy**2)
                    
                    # Only consider collisions that are a minimum distance away to avoid self-intersection
                    if distance > 0.01 and distance < closest:
                        closest = distance
                        closestPoint = intersectPoint
                        closestWall = hitWall

            if closestPoint is not None:
                # Store the ray segment for later drawing
                ray_segments.append({
                    'start': (current_ray.x, current_ray.y),
                    'end': closestPoint,
                    'color': current_color,
                    'reflection_order': reflection_count
                })
                
                if SOLID_RAYS:
                    pygame.draw.polygon(display, current_color, [(mx, my), closestPoint, lastClosestPoint])
                    lastClosestPoint = closestPoint
                
                # Create reflected ray if reflections are enabled and we haven't hit max reflections
                if ENABLE_REFLECTIONS and reflection_count < MAX_REFLECTIONS and closestWall is not None and closestWall.reflectance > 0:
                    # Calculate reflected direction
                    reflected_dir = closestWall.reflect_ray(current_ray.dir)
                    
                    # Add small offset to prevent hitting the same wall immediately
                    offset = 0.1
                    offset_x = closestPoint[0] + reflected_dir[0] * offset
                    offset_y = closestPoint[1] + reflected_dir[1] * offset
                    
                    # Create new ray from slightly offset position
                    reflected_angle = math.atan2(reflected_dir[1], reflected_dir[0])
                    current_ray = Ray(offset_x, offset_y, reflected_angle, reflection_count + 1)
                    
                    # Apply dimming based on wall reflectance
                    if isinstance(current_color, str):
                        # Convert white to RGB and apply reflectance-based dimming
                        current_color = (int(255 * closestWall.reflectance), int(255 * closestWall.reflectance), int(255 * closestWall.reflectance))
                    else:
                        # Apply reflectance-based dimming (multiply by reflectance)
                        current_color = tuple(max(0, int(c * closestWall.reflectance)) for c in current_color)
                else:
                    break  # No more reflections
            else:
                # No collision found - extend ray to screen boundary
                ray_end = calculate_screen_intersection(current_ray)
                ray_segments.append({
                    'start': (current_ray.x, current_ray.y),
                    'end': ray_end,
                    'color': current_color,
                    'reflection_order': reflection_count
                })
                
                if SOLID_RAYS:
                    pygame.draw.polygon(display, current_color, [(mx, my), ray_end, lastClosestPoint])
                    lastClosestPoint = ray_end
                break  # No collision found
    
    # Draw ray segments in reverse order (highest reflection order first, original rays last)
    ray_segments.sort(key=lambda segment: segment['reflection_order'], reverse=True)
    for segment in ray_segments:
        pygame.draw.line(display, segment['color'], segment['start'], segment['end'])

def generateWalls():
    walls.clear()

    if DEMO_MODE:
        # Demo mode: Two walls at 45 degrees
        # Fixed wall (farther from emitter)
        fixed_wall_center_x = WINDOW_SIZE[0] * 0.6
        fixed_wall_center_y = WINDOW_SIZE[1] * 0.2
        fixed_wall_length = 350
        
        # Calculate wall endpoints for 45-degree angle
        angle_rad = math.radians(45)
        half_length = fixed_wall_length / 2
        dx = math.cos(angle_rad) * half_length
        dy = math.sin(angle_rad) * half_length
        
        fixed_wall_start = (fixed_wall_center_x - dx, fixed_wall_center_y - dy)
        fixed_wall_end = (fixed_wall_center_x + dx, fixed_wall_center_y + dy)
        walls.append(Wall(fixed_wall_start, fixed_wall_end, 'red'))
        
        # Controllable wall (closer to emitter)
        # Position directly controlled by mouse
        controllable_center_x = controllable_wall_x
        controllable_center_y = controllable_wall_y
        
        # Wall orientation controlled by scroll wheel
        wall_orientation_angle = math.radians(controllable_wall_angle)
        wall_length = 250
        wall_half_length = wall_length / 2
        wall_dx = math.cos(wall_orientation_angle) * wall_half_length
        wall_dy = math.sin(wall_orientation_angle) * wall_half_length
        
        controllable_wall_start = (controllable_center_x - wall_dx, controllable_center_y - wall_dy)
        controllable_wall_end = (controllable_center_x + wall_dx, controllable_center_y + wall_dy)
        walls.append(Wall(controllable_wall_start, controllable_wall_end, 'yellow'))
        
        # Non-reflecting wall to the right of the emitter
        non_reflecting_wall_x = emitter_x + 150  # 150 pixels to the right
        non_reflecting_wall_start = (non_reflecting_wall_x, emitter_y - 550)
        non_reflecting_wall_end = (non_reflecting_wall_x, emitter_y + 350)
        walls.append(Wall(non_reflecting_wall_start, non_reflecting_wall_end, 'blue', reflectance=0.0))
    else:
        # Original random walls mode
        for i in range(NUM_WALLS):
            start_x = random.randint(0, WINDOW_SIZE[0])
            start_y = random.randint(0, WINDOW_SIZE[1])
            end_x = random.randint(0, WINDOW_SIZE[0])
            end_y = random.randint(0, WINDOW_SIZE[1])
            walls.append(Wall((start_x, start_y), (end_x, end_y)))

def draw():
    display.fill((0, 0, 0))

    for wall in walls:
        wall.draw()

    for particle in particles:
        particle.draw()

    drawRays([ray for ray in rays], [wall for wall in walls])
    
    # Draw emitter position in demo mode
    if DEMO_MODE:
        pygame.draw.circle(display, (0, 255, 0), (int(emitter_x), int(emitter_y)), 8)
        # Draw a small circle at the controllable wall center for visual reference
        pygame.draw.circle(display, (255, 255, 0), (int(controllable_wall_x), int(controllable_wall_y)), 4)

    screen.blit(display, (0, 0))

    pygame.display.update()

generateWalls()
while running:
    if DEMO_MODE:
        # In demo mode, use fixed emitter position
        mx, my = emitter_x, emitter_y
    else:
        # In normal mode, follow mouse
        mx, my = pygame.mouse.get_pos()
        
    for event in pygame.event.get():
        if event.type == QUIT:
            sys.exit()
            pygame.quit()

        if event.type == KEYDOWN:
            # Toggle demo mode with 'D' key
            if event.key == pygame.K_d:
                DEMO_MODE = not DEMO_MODE
                generateWalls()
            # Re-randomize walls on Space
            elif event.key == pygame.K_SPACE:
                generateWalls()
        
        if DEMO_MODE:
            # Control the controllable wall position with mouse movement
            if event.type == MOUSEMOTION:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                controllable_wall_x = mouse_x
                controllable_wall_y = mouse_y
                generateWalls()  # Regenerate walls with new position
            
            # Control rotation with scroll wheel
            if event.type == MOUSEWHEEL:
                # Scroll up (event.y > 0) rotates counter-clockwise, scroll down rotates clockwise
                controllable_wall_angle += event.y * 0.1  # 0.5 degrees per scroll step
                generateWalls()
            
            # Control distance with arrow keys (for backward compatibility)
            if event.type == KEYDOWN:
                if event.key == pygame.K_LEFT:
                    controllable_wall_angle -= 5  # 5 degrees per key press
                    generateWalls()
                elif event.key == pygame.K_RIGHT:
                    controllable_wall_angle += 5  # 5 degrees per key press
                    generateWalls()

    for ray in rays:
        ray.update(mx, my)

    draw()



