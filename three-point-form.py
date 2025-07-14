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
NUM_SAMPLES = 6 # Number of sample points per wall for three-point-form method
SOLID_RAYS = False # Can be somewhat glitchy. For best results, set NUM_RAYS to 360
ENABLE_REFLECTIONS = True # Enable first-order reflections
MAX_REFLECTIONS = 3 # Maximum number of reflections per ray
PHONG_EXPONENT = 200 # Phong exponent for specular reflections (higher = more specular)
DEMO_MODE = True # Enable demo mode with controllable walls (default mode)
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
emitter_x = WINDOW_SIZE[0] * 0.05
emitter_y = WINDOW_SIZE[1]*0.9
controllable_wall_angle = 135  # degrees
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

for i in range(NUM_RAYS):
    angle = (i * 360 / NUM_RAYS)  # Distribute rays evenly across 360 degrees
    rays.append(Ray(mx, my, math.radians(angle)))

def regenerate_rays():
    """Regenerate the rays array when NUM_RAYS changes"""
    global rays, mx, my
    rays.clear()
    print(f"Regenerating {NUM_RAYS} rays...")  # Debug output
    for i in range(NUM_RAYS):
        angle = (i * 360 / NUM_RAYS)  # Distribute rays evenly across 360 degrees
        rays.append(Ray(mx, my, math.radians(angle)))
    print(f"Generated {len(rays)} rays")  # Debug output

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
                    
                    # Apply simple reflectance dimming (no BSDF in traditional ray tracing)
                    if isinstance(current_color, str):
                        # Convert white to RGB and apply reflectance
                        base_intensity = 255
                        intensity = int(base_intensity * closestWall.reflectance)
                        current_color = (intensity, intensity, intensity)
                    else:
                        # Apply reflectance (multiply by wall reflectance)
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
    
    # Draw ray segments sorted by brightness (darkest first, brightest last)
    def get_brightness(segment):
        color = segment['color']
        if isinstance(color, str):
            return 255  # White is brightest
        elif isinstance(color, tuple):
            # Calculate brightness as average of RGB values
            return sum(color) / len(color)
        return 0
    
    ray_segments.sort(key=get_brightness)
    for segment in ray_segments:
        pygame.draw.line(display, segment['color'], segment['start'], segment['end'])

def generateWalls():
    walls.clear()

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
    
    # Second static wall (below the first one, also at 45 degrees)
    second_wall_center_x = WINDOW_SIZE[0] * 0.6
    second_wall_center_y = WINDOW_SIZE[1] * 0.8  # Lower than the first wall
    second_wall_length = 350
    
    # Calculate wall endpoints for 45-degree angle (same as first wall)
    second_angle_rad = math.radians(45)
    second_half_length = second_wall_length / 2
    second_dx = math.cos(second_angle_rad) * second_half_length
    second_dy = math.sin(second_angle_rad) * second_half_length
    
    second_wall_start = (second_wall_center_x - second_dx, second_wall_center_y - second_dy)
    second_wall_end = (second_wall_center_x + second_dx, second_wall_center_y + second_dy)
    walls.append(Wall(second_wall_start, second_wall_end, 'orange'))

    
    # Non-reflecting wall to the right of the emitter
    non_reflecting_wall_x = WINDOW_SIZE[0]*0.1
    non_reflecting_wall_start = (non_reflecting_wall_x, WINDOW_SIZE[1])
    non_reflecting_wall_end = (non_reflecting_wall_x, WINDOW_SIZE[1]*0.4)
    walls.append(Wall(non_reflecting_wall_start, non_reflecting_wall_end, 'blue', reflectance=0.0))
    
    # Non-reflecting wall to the left of the second wall
    non_reflecting_wall_x = WINDOW_SIZE[0]*0.5
    non_reflecting_wall_start = (second_wall_center_x - second_dx, second_wall_center_y - second_dy)
    non_reflecting_wall_end =  (second_wall_center_x - second_dx, WINDOW_SIZE[1])
    walls.append(Wall(non_reflecting_wall_start, non_reflecting_wall_end, 'blue', reflectance=0.0))
    
    # if not DEMO_MODE:
    right_wall_start = (WINDOW_SIZE[0]*0.99, second_wall_center_y - second_dy)
    right_wall_end = (WINDOW_SIZE[0]*0.99, second_wall_center_y + second_dy)
    walls.append(Wall(right_wall_start, right_wall_end, 'orange'))
    

def sample_points_on_wall(wall, num_samples):
    """Sample equally spaced points along a wall in local coordinates"""
    points = []
    
    # Add small offset to avoid sampling exactly at endpoints
    endpoint_offset = 0.05  # 5% offset from each end
    
    for i in range(num_samples):
        if num_samples == 1:
            # Single sample at the center
            t = 0.5
        else:
            # Map i from [0, num_samples-1] to [endpoint_offset, 1-endpoint_offset]
            t = endpoint_offset + (i / (num_samples - 1)) * (1 - 2 * endpoint_offset)
        
        # Interpolate between start and end points
        x = wall.start_pos[0] + t * (wall.end_pos[0] - wall.start_pos[0])
        y = wall.start_pos[1] + t * (wall.end_pos[1] - wall.start_pos[1])
        
        points.append((x, y, t))  # Include parameter t for local coordinate
    return points

def is_point_occluded(emitter_pos, target_point, walls, target_wall):
    """Check if a direct line from emitter to target point is occluded by other walls"""
    emitter_x, emitter_y = emitter_pos
    target_x, target_y = target_point
    
    # Create a temporary ray from emitter to target point
    direction_x = target_x - emitter_x
    direction_y = target_y - emitter_y
    distance_to_target = math.sqrt(direction_x**2 + direction_y**2)
    
    if distance_to_target == 0:
        return True  # Emitter is at target point
    
    # Normalize direction
    direction_x /= distance_to_target
    direction_y /= distance_to_target
    
    # Check intersection with all walls except the target wall
    for wall in walls:
        if wall == target_wall:
            continue
            
        # Line-line intersection between emitter->target and wall
        x1, y1 = emitter_x, emitter_y
        x2, y2 = target_x, target_y
        x3, y3 = wall.start_pos
        x4, y4 = wall.end_pos
        
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denominator) < 1e-10:  # Lines are parallel
            continue
            
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator
        
        # Check if intersection occurs on both line segments
        if 0 < t < 1 and 0 <= u <= 1:
            # Calculate intersection point
            intersect_x = x1 + t * (x2 - x1)
            intersect_y = y1 + t * (y2 - y1)
            
            # Check if intersection is between emitter and target (not beyond target)
            dist_to_intersection = math.sqrt((intersect_x - emitter_x)**2 + (intersect_y - emitter_y)**2)
            if dist_to_intersection < distance_to_target - 1e-6:  # Small epsilon for floating point comparison
                return True  # Occluded
                
    return False  # Not occluded

def drawThreePointForm(emitter_pos, walls):
    """Draw rays using three-point-form method: sample points on walls and draw direct lines if not occluded"""
    if not walls:
        return
    
    # Collect all ray segments from recursive drawing
    ray_segments = []
    drawThreePointFormRecursive(emitter_pos, walls, 0, 'white', ray_segments)
    
    # Draw ray segments sorted by brightness (darkest first, brightest last)
    def get_brightness(segment):
        color = segment['color']
        if isinstance(color, str):
            return 255  # White is brightest
        elif isinstance(color, tuple):
            # Calculate brightness as average of RGB values
            return sum(color) / len(color)
        return 0
    
    ray_segments.sort(key=get_brightness)
    for segment in ray_segments:
        pygame.draw.line(display, segment['color'], segment['start'], segment['end'], 1)
        
        # Draw sample point dots only for first-order rays
        if segment['reflection_order'] == 0 and 'sample_point' in segment:
            pygame.draw.circle(display, segment['wall_color'], 
                             (int(segment['sample_point'][0]), int(segment['sample_point'][1])), 2)

def drawThreePointFormRecursive(emitter_pos, walls, reflection_depth, color, ray_segments, source_sample_index=None):
    """Recursively collect three-point-form ray segments with reflections"""
    if not walls or reflection_depth > MAX_REFLECTIONS:
        return
    
    # Handle color input - it might be reflection data from previous iteration
    reflection_source = None
    if isinstance(color, dict) and 'wall' in color:
        # This is reflection data from previous iteration
        reflection_source = color
        base_color = color['color']
    else:
        # This is a simple color
        base_color = color
    
    # Use the input color directly (no global dimming factor)
    if isinstance(base_color, str):
        if base_color == 'white':
            base_intensity = 255
        else:
            base_intensity = 255  # Default for other color names
        current_color = (base_intensity, base_intensity, base_intensity)
    else:
        current_color = base_color
    
    # Store reflection points for next iteration
    reflection_points = []
    
    # For first-order rays (reflection_depth == 0), sample all points on all walls
    # For higher-order reflections, only sample corresponding indexed points
    if reflection_depth == 0:
        # First-order: sample all points on all walls
        for wall in walls:
            # Skip walls that don't reflect light (reflectance = 0)
            if wall.reflectance <= 0:
                continue
                
            # Sample points on this wall (same number of samples for all reflection depths)
            sample_points = sample_points_on_wall(wall, NUM_SAMPLES)
            
            for sample_index, (point_x, point_y, local_t) in enumerate(sample_points):
                # Check if direct line from emitter to this point is occluded
                if not is_point_occluded(emitter_pos, (point_x, point_y), walls, wall):
                    # For first-order rays, use full color
                    final_color = current_color
                    
                    # Store ray segment instead of drawing immediately
                    segment = {
                        'start': emitter_pos,
                        'end': (point_x, point_y),
                        'color': final_color,
                        'reflection_order': reflection_depth
                    }
                    
                    # Add sample point info for first-order rays
                    segment['sample_point'] = (point_x, point_y)
                    segment['wall_color'] = wall.color
                    
                    ray_segments.append(segment)
                    
                    # If we haven't reached max reflections and wall can reflect, prepare for next reflection
                    if reflection_depth < MAX_REFLECTIONS and wall.reflectance > 0:
                        # Calculate incident ray direction
                        incident_dx = point_x - emitter_pos[0]
                        incident_dy = point_y - emitter_pos[1]
                        incident_length = math.sqrt(incident_dx**2 + incident_dy**2)
                        
                        if incident_length > 0:
                            incident_dir = (incident_dx / incident_length, incident_dy / incident_length)
                            
                            # Calculate reflected direction using wall's reflection method
                            reflected_dir = wall.reflect_ray(incident_dir)
                            
                            # Get wall normal for later BSDF calculation
                            wall_normal = wall.get_normal()
                            
                            # Calculate new emitter position (slightly offset from reflection point)
                            offset = 0.1
                            new_emitter_x = point_x + reflected_dir[0] * offset
                            new_emitter_y = point_y + reflected_dir[1] * offset
                            
                            # Check if the reflected ray direction is not immediately blocked
                            # Test a short distance along the reflected ray to see if it's clear
                            test_distance = 10.0  # Test 10 pixels along the reflected direction
                            test_end_x = new_emitter_x + reflected_dir[0] * test_distance
                            test_end_y = new_emitter_y + reflected_dir[1] * test_distance
                            
                            # Only continue reflection if the reflected ray has a clear path
                            if not is_point_occluded((new_emitter_x, new_emitter_y), (test_end_x, test_end_y), walls, None):
                                # For energy conservation, use the actual reflected energy (final_color) 
                                # rather than the incoming energy (current_color)
                                # This ensures that next reflection gets energy based on what was actually reflected
                                if isinstance(final_color, tuple):
                                    # Use the actual reflected energy (final_color) for next reflection
                                    reduced_color = final_color
                                else:
                                    # Convert to tuple if needed
                                    reduced_color = (final_color, final_color, final_color) if isinstance(final_color, int) else final_color
                                
                                # Store reflection data for recursive call
                                reflection_points.append({
                                    'emitter': (new_emitter_x, new_emitter_y),
                                    'color': reduced_color,  # Use actual reflected energy for next reflection level
                                    'original_point': (point_x, point_y),
                                    'wall': wall,
                                    'incident_dir': incident_dir,
                                    'wall_normal': wall_normal,
                                    'sample_index': sample_index  # Pass the sample index for structured sampling
                                })
    else:
        # Higher-order reflections: only sample the corresponding indexed point on each wall
        if source_sample_index is not None:
            for wall in walls:
                # Skip walls that don't reflect light (reflectance = 0)
                if wall.reflectance <= 0:
                    continue
                    
                # Sample points on this wall (same number of samples for all reflection depths)
                sample_points = sample_points_on_wall(wall, NUM_SAMPLES)
                
                # Only sample the point with the same index as the source
                if source_sample_index < len(sample_points):
                    point_x, point_y, local_t = sample_points[source_sample_index]
                    
                    # Check if direct line from emitter to this point is occluded
                    if not is_point_occluded(emitter_pos, (point_x, point_y), walls, wall):
                        # Calculate final color for this ray segment
                        final_color = current_color
                        
                        # If this is a reflection from a previous surface, apply BSDF based on actual direction
                        if reflection_source is not None:
                            # Calculate actual direction from reflection point to this sample point
                            actual_direction_x = point_x - reflection_source['original_point'][0]
                            actual_direction_y = point_y - reflection_source['original_point'][1]
                            actual_length = math.sqrt(actual_direction_x**2 + actual_direction_y**2)
                            
                            if actual_length > 0:
                                actual_viewing_dir = (actual_direction_x / actual_length, actual_direction_y / actual_length)
                                
                                # Calculate BSDF using the wall reflectance (not cumulative reflectance)
                                # The current_color already contains the cumulative energy reduction
                                reflecting_wall = reflection_source['wall']
                                phong_brightness = calculate_phong_brightness(
                                    reflection_source['incident_dir'], 
                                    reflection_source['wall_normal'], 
                                    reflecting_wall.reflectance,  # Use the actual wall's reflectance for BSDF
                                    PHONG_EXPONENT, 
                                    actual_viewing_dir
                                )
                                
                                # Apply BSDF brightness to color - this naturally includes energy loss
                                # The BSDF caps at wall reflectance, ensuring energy conservation
                                if isinstance(current_color, tuple):
                                    final_color = tuple(max(0, int(c * phong_brightness)) for c in current_color)
                                else:
                                    # Convert string color to RGB and apply brightness
                                    base_intensity = 255
                                    intensity = int(base_intensity * phong_brightness)
                                    final_color = (intensity, intensity, intensity)
                        
                        # Store ray segment instead of drawing immediately
                        segment = {
                            'start': emitter_pos,
                            'end': (point_x, point_y),
                            'color': final_color,
                            'reflection_order': reflection_depth
                        }
                        
                        ray_segments.append(segment)
                        
                        # If we haven't reached max reflections and wall can reflect, prepare for next reflection
                        if reflection_depth < MAX_REFLECTIONS and wall.reflectance > 0:
                            # Calculate incident ray direction
                            incident_dx = point_x - emitter_pos[0]
                            incident_dy = point_y - emitter_pos[1]
                            incident_length = math.sqrt(incident_dx**2 + incident_dy**2)
                            
                            if incident_length > 0:
                                incident_dir = (incident_dx / incident_length, incident_dy / incident_length)
                                
                                # Calculate reflected direction using wall's reflection method
                                reflected_dir = wall.reflect_ray(incident_dir)
                                
                                # Get wall normal for later BSDF calculation
                                wall_normal = wall.get_normal()
                                
                                # Calculate new emitter position (slightly offset from reflection point)
                                offset = 0.1
                                new_emitter_x = point_x + reflected_dir[0] * offset
                                new_emitter_y = point_y + reflected_dir[1] * offset
                                
                                # Check if the reflected ray direction is not immediately blocked
                                # Test a short distance along the reflected ray to see if it's clear
                                test_distance = 10.0  # Test 10 pixels along the reflected direction
                                test_end_x = new_emitter_x + reflected_dir[0] * test_distance
                                test_end_y = new_emitter_y + reflected_dir[1] * test_distance
                                
                                # Only continue reflection if the reflected ray has a clear path
                                if not is_point_occluded((new_emitter_x, new_emitter_y), (test_end_x, test_end_y), walls, None):
                                    # For energy conservation, use the actual reflected energy (final_color) 
                                    # rather than the incoming energy (current_color)
                                    # This ensures that next reflection gets energy based on what was actually reflected
                                    if isinstance(final_color, tuple):
                                        # Use the actual reflected energy (final_color) for next reflection
                                        reduced_color = final_color
                                    else:
                                        # Convert to tuple if needed
                                        reduced_color = (final_color, final_color, final_color) if isinstance(final_color, int) else final_color
                                    
                                    # Store reflection data for recursive call
                                    reflection_points.append({
                                        'emitter': (new_emitter_x, new_emitter_y),
                                        'color': reduced_color,  # Use actual reflected energy for next reflection level
                                        'original_point': (point_x, point_y),
                                        'wall': wall,
                                        'incident_dir': incident_dir,
                                        'wall_normal': wall_normal,
                                        'sample_index': source_sample_index  # Continue with same sample index
                                    })
    
    # Process reflections recursively
    for reflection_data in reflection_points:
        # Calculate BSDF-modified color for this reflection using the actual directions to sample points
        # This will be done in the next recursive call where we know the actual sample directions
        drawThreePointFormRecursive(
            reflection_data['emitter'], 
            walls, 
            reflection_depth + 1, 
            reflection_data,  # Pass the full reflection data instead of just color
            ray_segments,
            reflection_data.get('sample_index')  # Pass the sample index for structured sampling
        )

def calculate_phong_brightness(incident_dir, wall_normal, wall_reflectance, phong_exponent=48, viewing_dir=None):
    """
    Calculate brightness using Phong BSDF model
    
    Args:
        incident_dir: Normalized incident ray direction
        wall_normal: Normalized wall normal
        wall_reflectance: Wall reflectance coefficient (0-1)
        phong_exponent: Phong exponent controlling specularity (higher = more specular)
        viewing_dir: Normalized outgoing/viewing direction (direction of reflected ray)
    
    Returns:
        Brightness factor (0-1) combining reflectance and specularity
    """
    # Make sure normal points toward the incident ray
    dot_product = incident_dir[0] * wall_normal[0] + incident_dir[1] * wall_normal[1]
    if dot_product > 0:
        wall_normal = (-wall_normal[0], -wall_normal[1])
        dot_product = -dot_product
    
    # Calculate perfect specular reflection direction
    # R = I - 2(I·N)N where I is incident, N is normal, R is perfect reflection
    perfect_reflection_x = incident_dir[0] - 2 * dot_product * wall_normal[0]
    perfect_reflection_y = incident_dir[1] - 2 * dot_product * wall_normal[1]
    
    # Use incident angle for basic calculation
    incident_factor = abs(dot_product)  # cos(incident_angle)
    
    if viewing_dir is not None:
        # If we have a viewing direction, use proper Phong BSDF with perfect reflection
        # Normalize viewing direction (just in case)
        viewing_length = math.sqrt(viewing_dir[0]**2 + viewing_dir[1]**2)
        if viewing_length > 0:
            viewing_dir = (viewing_dir[0] / viewing_length, viewing_dir[1] / viewing_length)
        
        # Calculate dot product between viewing direction and perfect reflection
        # This measures how close the actual outgoing direction is to perfect specular reflection
        specular_dot = (viewing_dir[0] * perfect_reflection_x + 
                       viewing_dir[1] * perfect_reflection_y)
        specular_dot = max(0, min(1, specular_dot))
        
        # Phong specular term: cos^n(angle between viewing direction and perfect reflection)
        specular_factor = specular_dot ** (phong_exponent)
    else:
        # Fallback: use incident angle-based calculation
        # Rays that hit more perpendicularly will reflect more strongly
        specular_factor = incident_factor ** (phong_exponent)
    
    # Pure specular reflection - no diffuse component
    # The specular factor already accounts for how well the viewing direction aligns with perfect reflection
    # The wall's reflectance is already < 1, so no additional energy loss is needed
    
    brightness = wall_reflectance * specular_factor
    
    # Cap at 100% of wall's reflectance - the BSDF can only redistribute energy, not create it
    brightness = max(0.0, min(wall_reflectance, brightness))
    
    return brightness

def draw():
    display.fill((0, 0, 0))

    # Draw particles first
    for particle in particles:
        particle.draw()

    # Draw rays before walls so walls appear on top
    if DEMO_MODE:
        # Use traditional ray tracing method
        drawRays([ray for ray in rays], [wall for wall in walls])
    else:
        # Use three-point-form method with same emitter position as demo mode
        drawThreePointForm((emitter_x, emitter_y), walls)
    
    # Draw walls after rays so they appear on top
    for wall in walls:
        wall.draw()
    
    # Draw emitter position and controllable wall indicator in both modes
    pygame.draw.circle(display, (0, 255, 0), (int(emitter_x), int(emitter_y)), 8)
    # Draw a small circle at the controllable wall center for visual reference
    pygame.draw.circle(display, (255, 255, 0), (int(controllable_wall_x), int(controllable_wall_y)), 4)

    screen.blit(display, (0, 0))

    pygame.display.update()

generateWalls()
while running:
    # Use fixed emitter position in both modes
    mx, my = emitter_x, emitter_y
        
    for event in pygame.event.get():
        if event.type == QUIT:
            sys.exit()
            pygame.quit()

        if event.type == KEYDOWN:
            # Control number of rays/samples with Up/Down arrows (works in both modes)
            if event.key == pygame.K_UP:
                if DEMO_MODE:
                    NUM_RAYS = min(360, NUM_RAYS + 10)  # Increase by 10, max 360
                    print(f"Rays increased to: {NUM_RAYS}")  # Debug output
                    regenerate_rays()
                else:
                    NUM_SAMPLES = min(100, NUM_SAMPLES + 1)  # Increase by 5, max 100
                    print(f"Samples increased to: {NUM_SAMPLES}")  # Debug output
            elif event.key == pygame.K_DOWN:
                if DEMO_MODE:
                    NUM_RAYS = max(10, NUM_RAYS - 10)  # Decrease by 10, min 10
                    print(f"Rays decreased to: {NUM_RAYS}")  # Debug output
                    regenerate_rays()
                else:
                    NUM_SAMPLES = max(1, NUM_SAMPLES - 1)  # Decrease by 5, min 1
                    print(f"Samples decreased to: {NUM_SAMPLES}")  # Debug output
            elif event.key == pygame.K_t and DEMO_MODE:
                # Toggle between 250 and 20 rays in demo mode
                NUM_RAYS = 20 if NUM_RAYS == 250 else 250
                print(f"Toggled rays to: {NUM_RAYS}")
                regenerate_rays()
            # Toggle between demo mode and alternative mode with 'D' key
            elif event.key == pygame.K_d:
                DEMO_MODE = not DEMO_MODE
                if not DEMO_MODE:
                    print("Switched to three-point-form")
                    # MAX_REFLECTIONS = 1
                else:
                    print("Switched to default path tracing.")
                    # MAX_REFLECTIONS = 3
                
                generateWalls()
            # Re-randomize walls on Space (only works in demo mode)
            elif event.key == pygame.K_SPACE and DEMO_MODE:
                generateWalls()
            # Wall controls (work in both modes)
            elif event.key == pygame.K_LEFT:
                MAX_REFLECTIONS = max(0, MAX_REFLECTIONS - 1)  # Decrease reflections, min 0
                print(f"Max reflections decreased to: {MAX_REFLECTIONS}")
            elif event.key == pygame.K_RIGHT:
                MAX_REFLECTIONS = min(10, MAX_REFLECTIONS + 1)  # Increase reflections, max 10
                print(f"Max reflections increased to: {MAX_REFLECTIONS}")
            # Controllable wall rotation controls
            elif event.key == pygame.K_a:
                controllable_wall_angle -= 5  # 5 degrees per key press
                generateWalls()
            elif event.key == pygame.K_d:
                controllable_wall_angle += 5  # 5 degrees per key press
                generateWalls()
            # Phong exponent controls (work in both modes)
            elif event.key == pygame.K_q:
                PHONG_EXPONENT = max(1, PHONG_EXPONENT - 2)  # Decrease specularity
                print(f"Phong exponent decreased to: {PHONG_EXPONENT}")
            elif event.key == pygame.K_e:
                PHONG_EXPONENT = min(64, PHONG_EXPONENT + 2)  # Increase specularity
                print(f"Phong exponent increased to: {PHONG_EXPONENT}")
            # Reset controllable wall angle
            elif event.key == pygame.K_r:
                controllable_wall_angle = 135  # Reset to default angle
                print(f"Wall angle reset to: {controllable_wall_angle} degrees")
                generateWalls()
        
        # Wall controls work in both modes
        # Control the controllable wall position with mouse movement
        if event.type == MOUSEMOTION:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            controllable_wall_x = mouse_x
            controllable_wall_y = mouse_y
            generateWalls()  # Regenerate walls with new position
        
        # Control rotation with scroll wheel
        if event.type == MOUSEWHEEL:
            # Scroll up (event.y > 0) rotates counter-clockwise, scroll down rotates clockwise
            controllable_wall_angle += event.y * 0.1  # 0.1 degrees per scroll step
            generateWalls()

    for ray in rays:
        ray.update(mx, my)

    draw()



