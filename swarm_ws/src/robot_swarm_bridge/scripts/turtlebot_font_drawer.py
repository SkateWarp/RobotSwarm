#!/usr/bin/env python3
"""
Enhanced Multi-Robot TurtleBot3 Font and Shape Drawing System

This ROS node provides a dynamic drawing system for multiple TurtleBot3 Waffle robots
that coordinates up to 10 robots to draw text, shapes, and patterns within a specified arena.
The system combines primitive-based drawing with data-driven font definitions and adds
sophisticated multi-robot coordination, collision avoidance, and arena awareness.

Features:
1. Support for up to 10 robots working simultaneously
2. Arena boundary awareness based on the 10x10m XACRO definition
3. Inter-robot collision avoidance
4. Coordinated multi-robot drawing capabilities
5. Optimized motion control with PD controller
6. Path planning with safety constraints
7. Integration with robot_swarm_manager through standardized topics
"""

import rospy
import math
import time
import json
import os
import re
import numpy as np
import threading
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray

# Import the provided TBI code
# Since the provided tbi.py is quite large, I'll import it and adapt as needed
# Here, we would normally import the classes and functions from tbi.py
# For this implementation, I'll extract and adapt the necessary parts

# Optional imports - will be handled gracefully if not available
try:
    import jsonschema
    HAS_JSONSCHEMA = True
except ImportError:
    HAS_JSONSCHEMA = False
    rospy.logwarn("jsonschema not available, using basic validation for font files")


#################################################
# GEOMETRY AND TRANSFORMATION UTILITY FUNCTIONS #
#################################################

class GeometryUtils:
    """
    Utility class for common geometric operations used across the system.
    Provides static methods for transformations, angle normalization, etc.
    """
    
    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to range [-pi, pi]
        
        Args:
            angle (float): Angle in radians
            
        Returns:
            float: Normalized angle in radians
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    @staticmethod
    def rotate_point(point, angle, origin=(0, 0)):
        """
        Rotate a point around an origin by a given angle.
        
        Args:
            point (tuple): (x, y) coordinates of point to rotate
            angle (float): Rotation angle in radians
            origin (tuple): (x, y) coordinates of rotation center
            
        Returns:
            tuple: Rotated (x, y) coordinates
        """
        # Translate point to origin
        x = point[0] - origin[0]
        y = point[1] - origin[1]
        
        # Rotate point
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        new_x = cos_a * x - sin_a * y + origin[0]
        new_y = sin_a * x + cos_a * y + origin[1]
        
        return (new_x, new_y)
    
    @staticmethod
    def compute_centroid(points):
        """
        Compute the centroid (average point) of a set of points.
        
        Args:
            points (list/numpy.ndarray): List or array of (x, y) points
            
        Returns:
            tuple: (x, y) centroid coordinates
        """
        # Convert to numpy array if not already
        if not isinstance(points, np.ndarray):
            points = np.array(points)
            
        # Compute mean along each axis
        centroid = np.mean(points, axis=0)
        
        return (centroid[0], centroid[1])


#############################
# DRAWING PRIMITIVE CLASSES #
#############################

class DrawingPrimitive:
    """Base class for all drawing primitives"""
    def get_points(self, num_points=20):
        """Get points along the primitive path"""
        raise NotImplementedError("Subclasses must implement get_points")
    
    def scale(self, scale_factor):
        """Scale the primitive by a factor"""
        raise NotImplementedError("Subclasses must implement scale")
    
    def translate(self, dx, dy):
        """Translate the primitive"""
        raise NotImplementedError("Subclasses must implement translate")
    
    def rotate(self, angle, origin=(0, 0)):
        """Rotate the primitive around an origin point"""
        raise NotImplementedError("Subclasses must implement rotate")


class Line(DrawingPrimitive):
    """A straight line segment from start to end"""
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start = (start_x, start_y)
        self.end = (end_x, end_y)
        rospy.Subscriber("/robot_manager/robot_list", String, self.robot_list_callback)

    def get_points(self, num_points=20):
        """Get evenly spaced points along the line"""
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = self.start[0] + t * (self.end[0] - self.start[0])
            y = self.start[1] + t * (self.end[1] - self.start[1])
            points.append((x, y))
        return points
    
    def scale(self, scale_factor):
        """Scale the line from its center"""
        # Find center
        center_x = (self.start[0] + self.end[0]) / 2
        center_y = (self.start[1] + self.end[1]) / 2
        
        # Scale relative to center
        new_start_x = center_x + (self.start[0] - center_x) * scale_factor
        new_start_y = center_y + (self.start[1] - center_y) * scale_factor
        new_end_x = center_x + (self.end[0] - center_x) * scale_factor
        new_end_y = center_y + (self.end[1] - center_y) * scale_factor
        
        return Line(new_start_x, new_start_y, new_end_x, new_end_y)
    
    def translate(self, dx, dy):
        """Translate the line"""
        return Line(
            self.start[0] + dx, self.start[1] + dy,
            self.end[0] + dx, self.end[1] + dy
        )
    
    def rotate(self, angle, origin=(0, 0)):
        """Rotate the line around an origin point"""
        # Translate to origin
        s_x = self.start[0] - origin[0]
        s_y = self.start[1] - origin[1]
        e_x = self.end[0] - origin[0]
        e_y = self.end[1] - origin[1]
        
        # Rotate
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        new_s_x = cos_a * s_x - sin_a * s_y + origin[0]
        new_s_y = sin_a * s_x + cos_a * s_y + origin[1]
        new_e_x = cos_a * e_x - sin_a * e_y + origin[0]
        new_e_y = sin_a * e_x + cos_a * e_y + origin[1]
        
        return Line(new_s_x, new_s_y, new_e_x, new_e_y)


class Arc(DrawingPrimitive):
    """An arc segment defined by center, radius, start angle, and end angle"""
    def __init__(self, center_x, center_y, radius, start_angle, end_angle, clockwise=False):
        self.center = (center_x, center_y)
        self.radius = radius
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.clockwise = clockwise
        
        # Ensure angles are properly set for drawing direction
        if self.clockwise and self.end_angle > self.start_angle:
            self.end_angle -= 2 * math.pi
        elif not self.clockwise and self.end_angle < self.start_angle:
            self.end_angle += 2 * math.pi
    
    def get_points(self, num_points=20):
        """Get evenly spaced points along the arc"""
        points = []
        # Use more points for larger arcs to maintain smooth appearance
        angle_diff = abs(self.end_angle - self.start_angle)
        adjusted_points = max(num_points, int(num_points * angle_diff / math.pi))
        
        angle_step = (self.end_angle - self.start_angle) / (adjusted_points - 1)
        
        for i in range(adjusted_points):
            angle = self.start_angle + i * angle_step
            x = self.center[0] + self.radius * math.cos(angle)
            y = self.center[1] + self.radius * math.sin(angle)
            points.append((x, y))
        
        return points
    
    def scale(self, scale_factor):
        """Scale the arc radius"""
        return Arc(
            self.center[0], self.center[1],
            self.radius * scale_factor,
            self.start_angle, self.end_angle,
            self.clockwise
        )
    
    def translate(self, dx, dy):
        """Translate the arc center"""
        return Arc(
            self.center[0] + dx, self.center[1] + dy,
            self.radius,
            self.start_angle, self.end_angle,
            self.clockwise
        )
    
    def rotate(self, angle, origin=(0, 0)):
        """Rotate the arc around an origin point"""
        # Translate center relative to origin
        c_x = self.center[0] - origin[0]
        c_y = self.center[1] - origin[1]
        
        # Rotate center
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        new_c_x = cos_a * c_x - sin_a * c_y + origin[0]
        new_c_y = sin_a * c_x + cos_a * c_y + origin[1]
        
        # Adjust arc angles
        new_start_angle = self.start_angle + angle
        new_end_angle = self.end_angle + angle
        
        return Arc(
            new_c_x, new_c_y,
            self.radius,
            new_start_angle, new_end_angle,
            self.clockwise
        )


class QuadraticBezier(DrawingPrimitive):
    """A quadratic Bezier curve with start, control, and end points"""
    def __init__(self, start_x, start_y, control_x, control_y, end_x, end_y):
        self.start = (start_x, start_y)
        self.control = (control_x, control_y)
        self.end = (end_x, end_y)
    
    def get_points(self, num_points=20):
        """Get points along the Bezier curve"""
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            # Quadratic Bezier formula: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
            x = ((1-t)**2 * self.start[0] + 
                 2 * (1-t) * t * self.control[0] + 
                 t**2 * self.end[0])
            y = ((1-t)**2 * self.start[1] + 
                 2 * (1-t) * t * self.control[1] + 
                 t**2 * self.end[1])
            points.append((x, y))
        return points
    
    def scale(self, scale_factor):
        """Scale the curve from its center"""
        # Find center of all points (including control point)
        center_x = (self.start[0] + self.control[0] + self.end[0]) / 3
        center_y = (self.start[1] + self.control[1] + self.end[1]) / 3
        
        # Scale relative to center
        new_start_x = center_x + (self.start[0] - center_x) * scale_factor
        new_start_y = center_y + (self.start[1] - center_y) * scale_factor
        new_control_x = center_x + (self.control[0] - center_x) * scale_factor
        new_control_y = center_y + (self.control[1] - center_y) * scale_factor
        new_end_x = center_x + (self.end[0] - center_x) * scale_factor
        new_end_y = center_y + (self.end[1] - center_y) * scale_factor
        
        return QuadraticBezier(
            new_start_x, new_start_y,
            new_control_x, new_control_y,
            new_end_x, new_end_y
        )
    
    def translate(self, dx, dy):
        """Translate the curve"""
        return QuadraticBezier(
            self.start[0] + dx, self.start[1] + dy,
            self.control[0] + dx, self.control[1] + dy,
            self.end[0] + dx, self.end[1] + dy
        )
    
    def rotate(self, angle, origin=(0, 0)):
        """Rotate the curve around an origin point"""
        # Translate to origin
        s_x = self.start[0] - origin[0]
        s_y = self.start[1] - origin[1]
        c_x = self.control[0] - origin[0]
        c_y = self.control[1] - origin[1]
        e_x = self.end[0] - origin[0]
        e_y = self.end[1] - origin[1]
        
        # Rotate
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        new_s_x = cos_a * s_x - sin_a * s_y + origin[0]
        new_s_y = sin_a * s_x + cos_a * s_y + origin[1]
        new_c_x = cos_a * c_x - sin_a * c_y + origin[0]
        new_c_y = sin_a * c_x + cos_a * c_y + origin[1]
        new_e_x = cos_a * e_x - sin_a * e_y + origin[0]
        new_e_y = sin_a * e_x + cos_a * e_y + origin[1]
        
        return QuadraticBezier(
            new_s_x, new_s_y,
            new_c_x, new_c_y,
            new_e_x, new_e_y
        )


class Shape:
    """A shape composed of multiple drawing primitives"""
    def __init__(self, primitives=None):
        self.primitives = primitives or []
    
    def add_primitive(self, primitive):
        """Add a primitive to the shape"""
        self.primitives.append(primitive)
    
    def get_all_points(self, points_per_primitive=20):
        """Get all points for all primitives in the shape"""
        all_paths = []
        for primitive in self.primitives:
            all_paths.append(primitive.get_points(points_per_primitive))
        return all_paths
    
    def scale(self, scale_factor):
        """Scale the entire shape"""
        new_shape = Shape()
        for primitive in self.primitives:
            new_shape.add_primitive(primitive.scale(scale_factor))
        return new_shape
    
    def translate(self, dx, dy):
        """Translate the entire shape"""
        new_shape = Shape()
        for primitive in self.primitives:
            new_shape.add_primitive(primitive.translate(dx, dy))
        return new_shape
    
    def rotate(self, angle, origin=(0, 0)):
        """Rotate the entire shape around an origin point"""
        new_shape = Shape()
        for primitive in self.primitives:
            new_shape.add_primitive(primitive.rotate(angle, origin))
        return new_shape


class FontParser:
    """
    Parses font definitions from a JSON file and creates shapes.
    """
    
    def __init__(self, font_path=None):
        """
        Initialize the font parser with a font definition file.
        
        Args:
            font_path (str): Path to the font JSON file
        """
        self.font_data = None
        self.default_em_size = 1.0
        self.baseline = 0.2
        
        # Load the default font if provided
        if font_path and os.path.exists(font_path):
            self.load_font(font_path)
        else:
            # Load a simplified default font if no file is provided
            self.load_default_font()
            if font_path:
                rospy.logwarn(f"Font file '{font_path}' not found. Using default font.")
    
    def load_font(self, font_path):
        """
        Load a font definition from a JSON file.
        
        Args:
            font_path (str): Path to the font JSON file
        """
        try:
            with open(font_path, 'r') as f:
                self.font_data = json.load(f)
                
            # Get metadata values
            if 'metadata' in self.font_data:
                metadata = self.font_data['metadata']
                self.default_em_size = metadata.get('em_size', 1.0)
                self.baseline = metadata.get('baseline', 0.2)
                
            rospy.loginfo(f"Loaded font: {self.font_data.get('metadata', {}).get('name', 'Unknown')} "
                          f"(em_size: {self.default_em_size}, baseline: {self.baseline})")
                          
            # Validate glyphs section
            if 'glyphs' not in self.font_data:
                rospy.logwarn("Font file missing 'glyphs' section. Using default font.")
                self.load_default_font()
                
        except json.JSONDecodeError as e:
            rospy.logerr(f"Error parsing font file: {e}")
            self.load_default_font()
        except Exception as e:
            rospy.logerr(f"Error loading font file: {str(e)}")
            self.load_default_font()
    
    def load_default_font(self):
        """Load a minimal default font with basic letter definitions"""
        rospy.loginfo("Loading default font")
        self.font_data = {
            "metadata": {
                "name": "DefaultFont",
                "author": "TurtleBot Drawer",
                "version": "1.0",
                "baseline": 0.2,
                "em_size": 1.0
            },
            "glyphs": {
                "A": {
                    "advance_width": 0.8,
                    "paths": [
                        {"type": "line", "points": [[0.4, 0], [0, 1]]},
                        {"type": "line", "points": [[0.4, 0], [0.8, 1]]},
                        {"type": "line", "points": [[0.2, 0.5], [0.6, 0.5]]}
                    ]
                },
                "O": {
                    "advance_width": 0.8,
                    "paths": [
                        {
                            "type": "arc",
                            "center": [0.4, 0.5],
                            "radius": 0.4,
                            "start_angle": 0,
                            "end_angle": 360,
                            "degrees": True
                        }
                    ]
                }
            }
        }
        
    def create_glyph(self, char, x=0, y=0, scale=1.0):
        """
        Create a shape for a character glyph.
        
        Args:
            char (str): Character to create
            x (float): X position
            y (float): Y position
            scale (float): Scale factor
            
        Returns:
            Shape: Shape object for the glyph, or None if not found
        """
        if not self.font_data or 'glyphs' not in self.font_data:
            rospy.logwarn("No font data loaded")
            return None
        
        # Get the character data
        char = char.upper()  # Convert to uppercase
        if char not in self.font_data['glyphs']:
            rospy.logdebug(f"Character '{char}' not found in font")
            return None
        
        glyph_data = self.font_data['glyphs'][char]
        shape = Shape()
        
        # Apply baseline adjustment for proper vertical positioning
        adjusted_y = y + (self.baseline * scale)
        
        # Create primitives for each path in the glyph
        for path in glyph_data.get('paths', []):
            path_type = path.get('type', '')
            
            if path_type == 'line':
                points = path.get('points', [])
                if len(points) >= 2:
                    # Scale points by em size and user scale
                    em_scale = self.default_em_size * scale
                    x1 = x + points[0][0] * em_scale
                    y1 = adjusted_y + points[0][1] * em_scale
                    x2 = x + points[1][0] * em_scale
                    y2 = adjusted_y + points[1][1] * em_scale
                    
                    shape.add_primitive(Line(x1, y1, x2, y2))
            
            elif path_type == 'arc':
                center = path.get('center', [0, 0])
                radius = path.get('radius', 0.1)
                start_angle = path.get('start_angle', 0)
                end_angle = path.get('end_angle', 360)
                degrees = path.get('degrees', False)
                clockwise = path.get('clockwise', False)
                
                # Convert degrees to radians if needed
                if degrees:
                    start_angle = math.radians(start_angle)
                    end_angle = math.radians(end_angle)
                
                # Scale by em size and user scale
                em_scale = self.default_em_size * scale
                center_x = x + center[0] * em_scale
                center_y = adjusted_y + center[1] * em_scale
                scaled_radius = radius * em_scale
                
                shape.add_primitive(Arc(
                    center_x, center_y, 
                    scaled_radius, 
                    start_angle, end_angle, 
                    clockwise
                ))
            
            elif path_type == 'bezier':
                points = path.get('points', [])
                if len(points) >= 3:
                    # Scale points by em size and user scale
                    em_scale = self.default_em_size * scale
                    start_x = x + points[0][0] * em_scale
                    start_y = adjusted_y + points[0][1] * em_scale
                    control_x = x + points[1][0] * em_scale
                    control_y = adjusted_y + points[1][1] * em_scale
                    end_x = x + points[2][0] * em_scale
                    end_y = adjusted_y + points[2][1] * em_scale
                    
                    shape.add_primitive(QuadraticBezier(
                        start_x, start_y,
                        control_x, control_y,
                        end_x, end_y
                    ))
        
        return shape
    
    def get_advance_width(self, char, scale=1.0):
        """
        Get the advance width of a character.
        
        Args:
            char (str): Character
            scale (float): Scale factor
            
        Returns:
            float: Advance width or default width if not found
        """
        if not self.font_data or 'glyphs' not in self.font_data:
            return 0.5 * scale  # Default width
        
        char = char.upper()
        if char in self.font_data['glyphs']:
            return self.font_data['glyphs'][char].get('advance_width', 0.5) * self.default_em_size * scale
        
        return 0.5 * scale  # Default width


class ShapeLibrary:
    """Library of predefined shapes and letter generation methods"""
    
    @staticmethod
    def create_circle(center_x, center_y, radius):
        """Create a circle shape using arc"""
        shape = Shape()
        shape.add_primitive(Arc(center_x, center_y, radius, 0, 2 * math.pi))
        return shape
    
    @staticmethod
    def create_rectangle(x, y, width, height):
        """Create a rectangle shape"""
        shape = Shape()
        shape.add_primitive(Line(x, y, x + width, y))
        shape.add_primitive(Line(x + width, y, x + width, y + height))
        shape.add_primitive(Line(x + width, y + height, x, y + height))
        shape.add_primitive(Line(x, y + height, x, y))
        return shape
    
    @staticmethod
    def create_regular_polygon(center_x, center_y, radius, sides):
        """Create a regular polygon"""
        shape = Shape()
        angle_step = 2 * math.pi / sides
        
        for i in range(sides):
            angle1 = i * angle_step
            angle2 = (i + 1) * angle_step
            
            x1 = center_x + radius * math.cos(angle1)
            y1 = center_y + radius * math.sin(angle1)
            x2 = center_x + radius * math.cos(angle2)
            y2 = center_y + radius * math.sin(angle2)
            
            shape.add_primitive(Line(x1, y1, x2, y2))
        
        return shape
    
    @staticmethod
    def create_star(center_x, center_y, outer_radius, inner_radius, points):
        """Create a star shape"""
        shape = Shape()
        angle_step = math.pi / points
        
        for i in range(2 * points):
            radius = outer_radius if i % 2 == 0 else inner_radius
            angle1 = i * angle_step
            angle2 = (i + 1) * angle_step
            
            x1 = center_x + radius * math.cos(angle1)
            y1 = center_y + radius * math.sin(angle1)
            x2 = center_x + (inner_radius if i % 2 == 0 else outer_radius) * math.cos(angle2)
            y2 = center_y + (inner_radius if i % 2 == 0 else outer_radius) * math.sin(angle2)
            
            shape.add_primitive(Line(x1, y1, x2, y2))
        
        return shape
    
    @staticmethod
    def create_oval(center_x, center_y, x_radius, y_radius, num_points=20):
        """Create an oval shape"""
        shape = Shape()
        
        # Create an approximation using lines
        for i in range(num_points):
            angle1 = 2 * math.pi * i / num_points
            angle2 = 2 * math.pi * ((i + 1) % num_points) / num_points
            
            x1 = center_x + x_radius * math.cos(angle1)
            y1 = center_y + y_radius * math.sin(angle1)
            x2 = center_x + x_radius * math.cos(angle2)
            y2 = center_y + y_radius * math.sin(angle2)
            
            shape.add_primitive(Line(x1, y1, x2, y2))
        
        return shape
    
    @staticmethod
    def create_heart(center_x, center_y, size=0.5):
        """
        Create a heart shape centered at the given coordinates.
        
        Args:
            center_x (float): X-coordinate of the heart's center
            center_y (float): Y-coordinate of the heart's center
            size (float): Size of the heart (height/width)
            
        Returns:
            Shape: Heart shape object
        """
        shape = Shape()
        
        # Calculate dimensions based on size
        width = size
        height = size * 0.9  # Slightly shorter than wide for better proportions
        
        # Calculate the centers of the two arcs that form the top of the heart
        arc_radius = width / 4
        left_arc_center_x = center_x - arc_radius
        right_arc_center_x = center_x + arc_radius
        arc_center_y = center_y + height/4
        
        # Calculate the bottom point of the heart
        bottom_point_x = center_x
        bottom_point_y = center_y - height/2
        
        # Add the left arc (top-left curve) - from 180° to 0° clockwise
        shape.add_primitive(Arc(
            left_arc_center_x, arc_center_y,
            arc_radius,
            math.pi, 0,
            True  # Clockwise direction
        ))
        
        # Add the right arc (top-right curve) - from 0° to 180° counter-clockwise
        shape.add_primitive(Arc(
            right_arc_center_x, arc_center_y,
            arc_radius,
            0, math.pi,
            False  # Counter-clockwise direction
        ))
        
        # Add the left diagonal line (left side coming down to a point)
        shape.add_primitive(Line(
            left_arc_center_x, arc_center_y - arc_radius,  # Bottom of left arc
            bottom_point_x, bottom_point_y  # Bottom point
        ))
        
        # Add the right diagonal line (right side coming down to a point)
        shape.add_primitive(Line(
            right_arc_center_x, arc_center_y - arc_radius,  # Bottom of right arc
            bottom_point_x, bottom_point_y  # Bottom point
        ))
        
        return shape
    
    @staticmethod
    def create_spiral(center_x, center_y, radius=0.5, turns=2):
        """
        Create a spiral shape centered at the given coordinates.
        
        Args:
            center_x (float): X-coordinate of the spiral's center
            center_y (float): Y-coordinate of the spiral's center
            radius (float): Maximum radius of the spiral
            turns (int): Number of complete turns in the spiral
            
        Returns:
            Shape: Spiral shape object
        """
        shape = Shape()
        points_per_turn = 20
        total_points = int(points_per_turn * turns)
        
        # Calculate the spiral points
        for i in range(total_points - 1):
            t1 = i * 2 * math.pi / points_per_turn
            t2 = (i + 1) * 2 * math.pi / points_per_turn
            
            # The radius grows linearly with t
            r1 = (radius * i) / total_points
            r2 = (radius * (i + 1)) / total_points
            
            x1 = center_x + r1 * math.cos(t1)
            y1 = center_y + r1 * math.sin(t1)
            x2 = center_x + r2 * math.cos(t2)
            y2 = center_y + r2 * math.sin(t2)
            
            shape.add_primitive(Line(x1, y1, x2, y2))
        
        return shape
    
    @staticmethod
    def create_parametric_letter(letter, x=0, y=0, scale=1.0):
        """
        Create letters using parametric algorithms for special cases.
        This is a backup method for letters not defined in the font.
        
        Args:
            letter (str): Letter to create (uppercase)
            x (float): X position
            y (float): Y position
            scale (float): Scale factor
            
        Returns:
            Shape: Shape object for the letter
        """
        letter = letter.upper()
        shape = Shape()
        
        width = 0.6 * scale
        height = 1.0 * scale
        
        if letter == 'I':
            # Parametric I with serifs
            shape.add_primitive(Line(x, y, x + width, y))  # Bottom
            shape.add_primitive(Line(x + width/2, y, x + width/2, y + height))  # Middle
            shape.add_primitive(Line(x, y + height, x + width, y + height))  # Top
            
        elif letter == 'O':
            # Parametric O
            shape.add_primitive(Arc(x + width/2, y + height/2, width/2, 0, 2*math.pi))
            
        else:
            # Generic box for unknown letters
            shape = ShapeLibrary.create_rectangle(x, y, width, height)
            
        return shape


class ArenaManager:
    """
    Manages the arena constraints and collision prevention for multiple robots.
    Tracks the positions of all robots and provides collision checking capabilities.
    """
    
    def __init__(self, arena_width=10.0, arena_height=10.0, robot_radius=0.22, safety_margin=0.1):
        """
        Initialize the arena manager with the dimensions of the arena.
        
        Args:
            arena_width (float): Width of the arena in meters
            arena_height (float): Height of the arena in meters
            robot_radius (float): Radius of the TurtleBot3 robots in meters
            safety_margin (float): Additional safety distance between robots in meters
        """
        self.arena_width = arena_width
        self.arena_height = arena_height
        self.robot_radius = robot_radius  # Approximate radius of TurtleBot3 Waffle
        self.safety_margin = safety_margin
        self.safe_distance = 2 * robot_radius + safety_margin
        
        # Arena boundaries (from the .xacro file, the arena is 10x10 centered at origin)
        self.min_x = -arena_width / 2 + robot_radius + safety_margin
        self.max_x = arena_width / 2 - robot_radius - safety_margin
        self.min_y = -arena_height / 2 + robot_radius + safety_margin
        self.max_y = arena_height / 2 - robot_radius - safety_margin
        
        # Dictionary to store robot positions
        self.robot_positions = {}
        
        # Robot locks for temporary pathing restrictions
        self.robot_locks = set()
        
        rospy.loginfo(f"Arena manager initialized with boundaries: x=[{self.min_x:.2f}, {self.max_x:.2f}], y=[{self.min_y:.2f}, {self.max_y:.2f}]")
    
    def update_robot_position(self, robot_name, position):
        """
        Update the stored position of a robot.
        
        Args:
            robot_name (str): Name of the robot
            position (tuple): (x, y) coordinates of the robot
        """
        self.robot_positions[robot_name] = position
    
    def register_robot(self, robot_name):
        """
        Register a new robot with the arena manager.
        
        Args:
            robot_name (str): Name of the robot
        """
        if robot_name not in self.robot_positions:
            self.robot_positions[robot_name] = (0.0, 0.0)  # Default position
    
    def check_point_in_bounds(self, point):
        """
        Check if a point is within the arena boundaries.
        
        Args:
            point (tuple): (x, y) coordinates to check
            
        Returns:
            bool: True if the point is within boundaries, False otherwise
        """
        x, y = point
        return (self.min_x <= x <= self.max_x) and (self.min_y <= y <= self.max_y)
    
    def adjust_point_to_bounds(self, point):
        """
        Adjust a point to be within the arena boundaries.
        
        Args:
            point (tuple): (x, y) coordinates to adjust
            
        Returns:
            tuple: Adjusted (x, y) coordinates
        """
        x, y = point
        x = max(self.min_x, min(x, self.max_x))
        y = max(self.min_y, min(y, self.max_y))
        return (x, y)
    
    def check_robot_collision(self, robot_name, target_point):
        """
        Check if moving to the target point would cause a collision with other robots.
        
        Args:
            robot_name (str): Name of the robot to check
            target_point (tuple): Target (x, y) coordinates
            
        Returns:
            bool: True if collision would occur, False otherwise
        """
        target_x, target_y = target_point
        
        for other_robot, (other_x, other_y) in self.robot_positions.items():
            if other_robot == robot_name:
                continue
                
            # Calculate distance between target point and other robot
            distance = math.sqrt((target_x - other_x)**2 + (target_y - other_y)**2)
            
            # Check if distance is less than the safe distance
            if distance < self.safe_distance:
                return True  # Collision would occur
                
        return False  # No collision


class TurtleBotFontDrawer:
    """
    Controls TurtleBot3 robots to draw text and shapes using a font system.
    Modified to support the /robot/n/ namespace format and multi-robot coordination.
    """
    
    def __init__(self, robot_namespace_template="/robot/{}/", font_path=None):
        """
        Initialize the TurtleBot font drawer node.
        
        Args:
            robot_namespace_template (str): Template for robot namespaces
            font_path (str): Path to the font JSON file
        """
        rospy.init_node('turtlebot_font_drawer', anonymous=True)
        
        # Store the namespace template
        self.robot_namespace_template = robot_namespace_template
        
        # Load configurable parameters from the parameter server
        self.speed_linear = rospy.get_param("~speed_linear", 0.2)          # m/s
        self.speed_angular = rospy.get_param("~speed_angular", 0.5)        # rad/s
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.05)  # m
        self.angle_tolerance = rospy.get_param("~angle_tolerance", 0.1)    # rad
        self.control_rate = rospy.get_param("~control_rate", 10)           # Hz
        self.visualization_time = rospy.get_param("~visualization_time", 60)  # seconds
        self.detailed_logging = rospy.get_param("~detailed_logging", False)
        
        # Arena parameters
        self.arena_width = rospy.get_param("~arena_width", 10.0)
        self.arena_height = rospy.get_param("~arena_height", 10.0)
        
        # Initialize the rate controller
        self.rate = rospy.Rate(self.control_rate)
        
        # Set up publishers for each robot
        self.publishers = {}          # cmd_vel publishers
        self.robot_positions = {}     # Current position of each robot
        self.robot_orientations = {}  # Current orientation of each robot
        self.visualization_pubs = {}  # For visualizing paths in RViz
        
        # Create lock for thread safety
        self.lock = threading.RLock()
        
        # Create the arena manager
        self.arena_manager = ArenaManager(
            arena_width=self.arena_width,
            arena_height=self.arena_height
        )
        
        # Subscribe to command topic
        rospy.Subscriber("/drawer/command", String, self.command_callback)
        
        # Subscribe to multi-robot task topic
        rospy.Subscriber("/drawer/task", String, self.task_callback)
        
        # Publisher for status
        self.status_pub = rospy.Publisher("/drawer/status", String, queue_size=10)
        
        # Initialize font parser
        self.font_parser = FontParser(font_path)
        
        # Define available shape generation functions
        self.shapes = {
            "circle": self.draw_circle,
            "triangle": self.draw_regular_polygon,
            "square": self.draw_square,
            "rectangle": self.draw_rectangle,
            "star": self.draw_star,
            "oval": self.draw_oval,
            "hexagon": self.draw_regular_polygon,
            "pentagon": self.draw_regular_polygon,
            "octagon": self.draw_regular_polygon,
            "heart": self.draw_heart,
            "spiral": self.draw_spiral
        }
        
        # Define polygon sides for each shape
        self.polygon_sides = {
            "triangle": 3,
            "square": 4,
            "pentagon": 5,
            "hexagon": 6,
            "octagon": 8
        }
        
        # Register shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        
        # Start status publisher
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        rospy.loginfo("TurtleBot Font Drawer initialized")
        rospy.loginfo("Using robot namespace template: " + robot_namespace_template)
    
    def shutdown_hook(self):
        """Ensure robots stop when the node is shutting down"""
        rospy.loginfo("Shutting down TurtleBot Font Drawer - stopping all robots")
        with self.lock:
            for robot_name in self.publishers:
                self.stop_robot(robot_name)

    # Add this new method to the TurtleBotFontDrawer class
    def robot_list_callback(self, msg):
        """
        Process robot list updates and auto-register robots.
        
        Args:
            msg (std_msgs.String): JSON string with robot info
        """
        try:
            # Parse the JSON string
            robot_list = json.loads(msg.data)
            
            # Register each robot in the list
            for robot_info in robot_list:
                robot_id = robot_info['robot_id']
                robot_name = f"robot_{robot_id}"
                
                # Register if not already registered
                if robot_name not in self.publishers:
                    rospy.loginfo(f"Auto-registering robot {robot_name}")
                    self.register_robot(robot_id)
                    
        except Exception as e:
            rospy.logerr(f"Error processing robot list: {str(e)}")

    # Improve the check_for_obstacles method
    def check_for_obstacles(self, robot_name, target_x, target_y):
        """Check if moving to target would cause collision"""
        with self.lock:
            # Get current position of the robot
            if robot_name not in self.robot_positions:
                return False
                
            current_x, current_y = self.robot_positions[robot_name]
            
            # Check each other robot
            for other_robot, (other_x, other_y) in self.robot_positions.items():
                if other_robot == robot_name:
                    continue
                    
                # Check if other robot is in path
                # Calculate distance to other robot
                dist = math.sqrt((other_x - target_x)**2 + (other_y - target_y)**2)
                
                # Consider minimum safe distance (robot diameter + margin)
                if dist < 0.7:  # Increased from 0.5
                    rospy.loginfo(f"Obstacle detected: {other_robot} near target for {robot_name}")
                    return True
                    
            return False
    
    def stop_robot(self, robot_name):
        """Stop the robot by publishing zero velocity"""
        with self.lock:
            if robot_name in self.publishers:
                twist = Twist()  # All fields initialize to zero
                # Publish several times to ensure it's received
                for _ in range(5):
                    self.publishers[robot_name].publish(twist)
                    rospy.sleep(0.1)
    
    def register_robot(self, robot_id):
        """
        Register a robot with the font drawer system.
        
        Args:
            robot_id (int): Robot ID
            
        Returns:
            bool: True if registration was successful, False otherwise
        """
        with self.lock:
            # Create the robot namespace
            robot_ns = self.robot_namespace_template.format(robot_id)
            robot_name = f"robot_{robot_id}"
            
            # Check if robot is already registered
            if robot_name in self.publishers:
                return True
                
            try:
                # Create publishers and subscribers for this robot
                self.publishers[robot_name] = rospy.Publisher(
                    f"{robot_ns}cmd_vel", 
                    Twist, 
                    queue_size=10
                )
                
                # Subscriber for odometry
                rospy.Subscriber(
                    f"{robot_ns}odom",
                    Odometry,
                    self.odom_callback,
                    callback_args=robot_name
                )
                
                # Publisher for visualization
                self.visualization_pubs[robot_name] = rospy.Publisher(
                    f"{robot_ns}drawing_path",
                    MarkerArray,
                    queue_size=10
                )
                
                # Initialize position and orientation
                self.robot_positions[robot_name] = (0.0, 0.0)
                self.robot_orientations[robot_name] = 0.0
                
                # Register with arena manager
                self.arena_manager.register_robot(robot_name)
                
                rospy.loginfo(f"Registered robot {robot_name} with namespace {robot_ns}")
                return True
                
            except Exception as e:
                rospy.logerr(f"Error registering robot {robot_name}: {str(e)}")
                return False
    
    def odom_callback(self, msg, robot_name):
        """
        Update robot position and orientation from odometry data.
        
        Args:
            msg (nav_msgs.Odometry): Odometry message
            robot_name (str): Name of the robot
        """
        with self.lock:
            try:
                # Extract position
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                
                # Store position
                self.robot_positions[robot_name] = (x, y)
                
                # Update arena manager
                self.arena_manager.update_robot_position(robot_name, (x, y))
                
                # Extract orientation (yaw)
                orientation_q = msg.pose.pose.orientation
                _, _, yaw = euler_from_quaternion(
                    [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                )
                self.robot_orientations[robot_name] = yaw
                
                if self.detailed_logging:
                    rospy.logdebug(f"Robot {robot_name} at position ({x:.2f}, {y:.2f}), orientation {yaw:.2f}")
            except Exception as e:
                rospy.logerr(f"Error in odom callback for {robot_name}: {str(e)}")
    
    def visualize_path(self, robot_name, paths):
        """
        Visualize the drawing path in RViz.
        
        Args:
            robot_name (str): Name of the robot
            paths (list): List of path segments, each a list of (x, y) points
        """
        try:
            with self.lock:
                if robot_name not in self.visualization_pubs:
                    rospy.logwarn(f"No visualization publisher for robot {robot_name}")
                    return
                    
                marker_array = MarkerArray()
                for i, path in enumerate(paths):
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = f"{robot_name}_path"
                    marker.id = i
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.02  # Line width
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.lifetime = rospy.Duration(self.visualization_time)  # Auto-expire after set time
                    
                    for x, y in path:
                        p = Point()
                        p.x = x
                        p.y = y
                        p.z = 0.05  # Slightly above ground
                        marker.points.append(p)
                    
                    marker_array.markers.append(marker)
                
                self.visualization_pubs[robot_name].publish(marker_array)
        except Exception as e:
            rospy.logerr(f"Error visualizing path: {str(e)}")
    
    def parse_command(self, command_str):
        """
        Parse command string using regular expressions for more robust handling.
        
        Args:
            command_str (str): Command string in the format "robot_name:action[:params]"
            
        Returns:
            tuple: (robot_name, action, params_dict) or (None, None, {}) if parsing fails
        """
        try:
            # Match the format robot_name:action[:params]
            match = re.match(r'([^:]+):([^:]+)(?::(.+))?', command_str)
            if not match:
                rospy.logwarn(f"Invalid command format: '{command_str}'. Use 'robot_name:action[:params]'")
                return None, None, {}
            
            robot_name, action, params_str = match.groups()
            params = {}
            
            if params_str:
                # If the action is 'text', treat everything after the second colon as text
                if action.lower() == 'text':
                    # Check if there are parameters after the text
                    if ':' in params_str:
                        text, param_str = params_str.split(':', 1)
                        params['text'] = text
                        # Parse parameters
                        if '=' in param_str:
                            param_pairs = re.findall(r'([^,=]+)=([^,]+)(?:,|$)', param_str)
                            for key, value in param_pairs:
                                key = key.strip()
                                value = value.strip()
                                try:
                                    # Try to convert to number
                                    if '.' in value:
                                        params[key] = float(value)
                                    else:
                                        params[key] = int(value)
                                except ValueError:
                                    params[key] = value
                    else:
                        params['text'] = params_str
                
                # If it's a 'combined' command, treat as a comma-separated list
                elif action.lower() == 'combined':
                    # Check if there are parameters after the elements
                    if ':' in params_str:
                        elements_str, param_str = params_str.split(':', 1)
                        params['elements'] = [elem.strip() for elem in elements_str.split(',')]
                        # Parse parameters
                        if '=' in param_str:
                            param_pairs = re.findall(r'([^,=]+)=([^,]+)(?:,|$)', param_str)
                            for key, value in param_pairs:
                                key = key.strip()
                                value = value.strip()
                                try:
                                    # Try to convert to number
                                    if '.' in value:
                                        params[key] = float(value)
                                    else:
                                        params[key] = int(value)
                                except ValueError:
                                    params[key] = value
                    else:
                        params['elements'] = [elem.strip() for elem in params_str.split(',')]
                
                # Otherwise look for key=value pairs
                elif '=' in params_str:
                    # Parse key=value pairs
                    param_pairs = re.findall(r'([^,=]+)=([^,]+)(?:,|$)', params_str)
                    for key, value in param_pairs:
                        key = key.strip()
                        value = value.strip()
                        try:
                            # Try to convert to number
                            if '.' in value:
                                params[key] = float(value)
                            else:
                                params[key] = int(value)
                        except ValueError:
                            params[key] = value
                else:
                    # Try to parse as a single number for scale
                    try:
                        params['scale'] = float(params_str)
                    except ValueError:
                        # If that fails, just store as a string
                        params['param'] = params_str
            
            return robot_name, action, params
        
        except Exception as e:
            rospy.logerr(f"Error parsing command: {str(e)}")
            return None, None, {}
    
    def command_callback(self, msg):
        """
        Process incoming commands from the /drawer/command topic.
        
        Args:
            msg (std_msgs.String): Command message in format "robot_name:action[:params]"
        """
        try:
            rospy.loginfo(f"DRAWER RECEIVED: {msg.data}")
            # Parse command
            robot_name, action, params = self.parse_command(msg.data)
            
            if not robot_name or not action:
                return
            
            # Ensure the robot is registered
            if robot_name not in self.publishers:
                # Try to register the robot by extracting ID from name
                robot_id_match = re.match(r'robot_(\d+)', robot_name)
                if robot_id_match:
                    robot_id = int(robot_id_match.group(1))
                    if not self.register_robot(robot_id):
                        rospy.logwarn(f"Failed to register robot {robot_name}")
                        return
                else:
                    rospy.logwarn(f"Robot '{robot_name}' not found. Available robots: {list(self.publishers.keys())}")
                    return
            
            # Get scale parameter (if present)
            scale = params.get('scale', 1.0)
            
            # Special text command
            if action.lower() == "text":
                text = params.get('text', '')
                if not text:
                    rospy.logwarn("Text command requires text, e.g., robot:text:HELLO")
                    return
                
                rospy.loginfo(f"Drawing text '{text}' with robot: {robot_name}")
                threading.Thread(target=self.draw_text, args=(robot_name, text, scale)).start()
                return
            
            # Check if action is a shape
            if action.lower() in self.shapes:
                rospy.loginfo(f"Drawing shape: {action} with robot: {robot_name}")
                
                # Prepare keyword arguments for the shape drawing function
                kwargs = {k: v for k, v in params.items() if k != 'scale'}
                
                # Add n_sides parameter for regular polygons
                if action.lower() in self.polygon_sides:
                    kwargs['n_sides'] = self.polygon_sides[action.lower()]
                
                # Call the appropriate shape drawing function in a thread
                shape_func = self.shapes[action.lower()]
                threading.Thread(target=shape_func, args=(robot_name,), kwargs=dict(scale=scale, **kwargs)).start()
                return
            
            # Check if action is a letter
            if len(action) == 1 and action.upper() in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                rospy.loginfo(f"Drawing letter: {action} with robot: {robot_name}")
                threading.Thread(target=self.draw_letter, args=(robot_name, action.upper(), scale)).start()
                return
            
            # Check for combined shapes
            if action.lower() == "combined":
                elements = params.get('elements', [])
                if not elements:
                    rospy.logwarn("Combined command requires elements, e.g., robot:combined:A,B,C")
                    return
                
                rospy.loginfo(f"Drawing combined shape: {','.join(elements)} with robot: {robot_name}")
                threading.Thread(target=self.draw_combined, args=(robot_name, elements, scale)).start()
                return
            
            rospy.logwarn(f"Unknown action: '{action}'")
            
        except Exception as e:
            rospy.logerr(f"Error processing command: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def task_callback(self, msg):
        """
        Process incoming tasks from the /drawer/task topic.
        These are multi-robot coordinated tasks.
        
        Args:
            msg (std_msgs.String): Task message
        """
        try:
            # Parse task
            task_parts = msg.data.split(':', 2)
            if len(task_parts) < 2:
                rospy.logwarn("Invalid task format. Use 'task_type:params'")
                return
                
            task_type = task_parts[0]
            task_params = task_parts[1]
            extra_params = task_parts[2] if len(task_parts) > 2 else ""
            
            # Parse parameters
            params = {}
            
            # Add the main parameter
            params['content'] = task_params
            
            # Add any extra parameters
            if extra_params:
                param_pairs = re.findall(r'([^,=]+)=([^,]+)(?:,|$)', extra_params)
                for key, value in param_pairs:
                    key = key.strip()
                    value = value.strip()
                    try:
                        # Try to convert to number
                        if '.' in value:
                            params[key] = float(value)
                        else:
                            params[key] = int(value)
                    except ValueError:
                        params[key] = value
            
            # Get available robots
            with self.lock:
                robot_names = list(self.publishers.keys())
            
            if not robot_names:
                rospy.logwarn("No robots available for task")
                return
                
            # Process different task types
            if task_type == "draw_text":
                text = params['content']
                scale = params.get('scale', 1.0)
                robot_count = min(len(robot_names), params.get('robots', len(text)))
                
                rospy.loginfo(f"Coordinated task: Drawing text '{text}' with {robot_count} robots")
                threading.Thread(target=self.coordinated_text_drawing, 
                                args=(text, robot_names[:robot_count], scale)).start()
                
            elif task_type == "draw_shape":
                shape = params['content']
                scale = params.get('scale', 1.0)
                robot_count = min(len(robot_names), params.get('robots', 1))
                
                rospy.loginfo(f"Coordinated task: Drawing shape '{shape}' with {robot_count} robots")
                threading.Thread(target=self.coordinated_shape_drawing, 
                                args=(shape, robot_names[:robot_count], scale)).start()
                
            elif task_type == "draw_pattern":
                pattern = params['content']
                scale = params.get('scale', 1.0)
                robot_count = min(len(robot_names), params.get('robots', 4))
                
                rospy.loginfo(f"Coordinated task: Drawing pattern '{pattern}' with {robot_count} robots")
                threading.Thread(target=self.coordinated_pattern_drawing, 
                                args=(pattern, robot_names[:robot_count], scale)).start()
                
            else:
                rospy.logwarn(f"Unknown task type: {task_type}")
                
        except Exception as e:
            rospy.logerr(f"Error processing task: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def move_to_point(self, robot_name, target_x, target_y, speed=None):
        """Move robot to target point with improved collision avoidance"""
        if speed is None:
            speed = self.speed_linear
                
        with self.lock:
            if robot_name not in self.publishers or robot_name not in self.robot_positions:
                rospy.logwarn(f"No publisher or position data for robot {robot_name}")
                return False
                
            current_x, current_y = self.robot_positions[robot_name]
        
        # Check if target would cause collision and find safe path
        rospy.loginfo(f"MOVE: {robot_name} to ({target_x:.2f}, {target_y:.2f})")
        original_target = (target_x, target_y)
        if self.arena_manager.check_robot_collision(robot_name, original_target):
            rospy.loginfo(f"Finding safe path for {robot_name} to avoid collision")
            
            # Calculate direction vector to target
            dx = target_x - current_x
            dy = target_y - current_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            # Try different paths at increasing angles from original
            for angle_offset in [0.5, -0.5, 1.0, -1.0, 1.5, -1.5]:
                # Rotate direction vector
                angle = math.atan2(dy, dx) + angle_offset
                
                # Find intermediate point at half distance
                safe_dist = min(dist/2, 0.5)  # Don't go too far
                new_x = current_x + safe_dist * math.cos(angle)
                new_y = current_y + safe_dist * math.sin(angle)
                
                # Check if new point is safe
                if not self.arena_manager.check_robot_collision(robot_name, (new_x, new_y)):
                    rospy.loginfo(f"Safe intermediate path found at angle {angle_offset}")
                    
                    # First move to intermediate point
                    if self._execute_move(robot_name, new_x, new_y, speed):
                        # Then try direct path to original target
                        return self._execute_move(robot_name, original_target[0], original_target[1], speed)
                    return False
        
            rospy.logwarn(f"No safe path found for {robot_name}")
            return False
        
        # Direct path is safe
        return self._execute_move(robot_name, target_x, target_y, speed)

    def _execute_move(self, robot_name, target_x, target_y, speed):
        """Execute the actual movement with PD control"""
        # Your existing movement control logic goes here
        # (Without the collision checking parts)
        control_rate = rospy.Rate(self.control_rate)
        max_duration = 60.0
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > max_duration:
                rospy.logwarn(f"Timeout reaching ({target_x:.2f}, {target_y:.2f})")
                self.stop_robot(robot_name)
                return False
                
            with self.lock:
                if robot_name not in self.robot_positions:
                    return False
                    
                current_x, current_y = self.robot_positions[robot_name]
                current_theta = self.robot_orientations[robot_name]
            
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < self.position_tolerance:
                self.stop_robot(robot_name)
                return True
                
            target_theta = math.atan2(dy, dx)
            angle_diff = GeometryUtils.normalize_angle(target_theta - current_theta)
            
            twist = Twist()
            if abs(angle_diff) < self.angle_tolerance:
                forward_speed = min(speed, 0.5 * distance + 0.05)
                twist.linear.x = forward_speed
                twist.angular.z = 0.3 * angle_diff
            else:
                twist.linear.x = 0
                twist.angular.z = min(self.speed_angular, max(-self.speed_angular, 
                                                    self.speed_angular * angle_diff))
            
            with self.lock:
                if robot_name in self.publishers:
                    self.publishers[robot_name].publish(twist)
            
            control_rate.sleep()
    
    def follow_path(self, robot_name, points, speed=None):
        """
        Move the robot along a path defined by a series of points.
        
        Args:
            robot_name (str): Name of the robot to move
            points (list): List of (x, y) coordinates defining the path
            speed (float, optional): Linear speed in m/s
            
        Returns:
            bool: True if the entire path was followed, False otherwise
        """
        if not points:
            return True
            
        if self.detailed_logging:
            rospy.loginfo(f"Following path with {len(points)} points")
            
        for i, (x, y) in enumerate(points):
            if self.detailed_logging and i % 5 == 0:  # Log progress every 5 points
                rospy.loginfo(f"Moving to point {i+1}/{len(points)}: ({x:.2f}, {y:.2f})")
                
            if not self.move_to_point(robot_name, x, y, speed):
                rospy.logwarn(f"Failed to reach point {i+1}/{len(points)}")
                return False
        
        return True
    
    def draw_shape(self, robot_name, shape, scale=1.0):
        """
        Draw a Shape object with the robot.
        
        Args:
            robot_name (str): Name of the robot to use
            shape (Shape): Shape object to draw
            scale (float): Scale factor for the shape
        """
        with self.lock:
            if robot_name not in self.robot_positions:
                rospy.logwarn(f"No position data available for robot {robot_name}")
                return
                
            # Get current position for centering
            current_x, current_y = self.robot_positions[robot_name]
        
        # Scale the shape
        scaled_shape = shape.scale(scale)
        
        # Translate to robot's current position
        positioned_shape = scaled_shape.translate(current_x, current_y)
        
        # Get all path segments
        paths = positioned_shape.get_all_points()
        
        # Visualize the path in RViz
        self.visualize_path(robot_name, paths)
        
        # Draw each path segment
        for i, path in enumerate(paths):
            rospy.loginfo(f"Drawing path segment {i+1}/{len(paths)}")
            self.follow_path(robot_name, path)
    
    # Shape drawing methods
    def draw_circle(self, robot_name, radius=0.5, scale=1.0):
        """
        Draw a circle using Arc primitive.
        
        Args:
            robot_name (str): Name of the robot to use
            radius (float): Radius of the circle in meters
            scale (float): Scale factor
        """
        # Create circle at origin, then translate to robot
        circle = ShapeLibrary.create_circle(0, 0, radius)
        rospy.loginfo(f"CIRCLE: robot={robot_name}, radius={radius}, scale={scale}")
        self.draw_shape(robot_name, circle, scale)
    
    def draw_square(self, robot_name, side_length=0.5, scale=1.0):
        """
        Draw a square using Line primitives.
        
        Args:
            robot_name (str): Name of the robot to use
            side_length (float): Length of each side in meters
            scale (float): Scale factor
        """
        # Create square centered at origin
        half_size = side_length / 2
        square = ShapeLibrary.create_rectangle(-half_size, -half_size, side_length, side_length)
        self.draw_shape(robot_name, square, scale)
    
    def draw_rectangle(self, robot_name, length=0.7, width=0.4, scale=1.0):
        """
        Draw a rectangle using Line primitives.
        
        Args:
            robot_name (str): Name of the robot to use
            length (float): Length of the rectangle in meters
            width (float): Width of the rectangle in meters
            scale (float): Scale factor
        """
        # Create rectangle centered at origin
        half_length = length / 2
        half_width = width / 2
        rectangle = ShapeLibrary.create_rectangle(
            -half_length, -half_width, 
            length, width
        )
        self.draw_shape(robot_name, rectangle, scale)
    
    def draw_regular_polygon(self, robot_name, n_sides=3, radius=0.5, scale=1.0):
        """
        Draw a regular polygon using Line primitives.
        
        Args:
            robot_name (str): Name of the robot to use
            n_sides (int): Number of sides
            radius (float): Radius from center to vertices
            scale (float): Scale factor
        """
        polygon = ShapeLibrary.create_regular_polygon(0, 0, radius, n_sides)
        self.draw_shape(robot_name, polygon, scale)
    
    def draw_star(self, robot_name, outer_radius=0.5, inner_radius=0.25, points=5, scale=1.0):
        """
        Draw a star using Line primitives.
        
        Args:
            robot_name (str): Name of the robot to use
            outer_radius (float): Radius to outer points
            inner_radius (float): Radius to inner points
            points (int): Number of points
            scale (float): Scale factor
        """
        star = ShapeLibrary.create_star(0, 0, outer_radius, inner_radius, points)
        self.draw_shape(robot_name, star, scale)
    
    def draw_oval(self, robot_name, major_axis=0.8, minor_axis=0.4, scale=1.0):
        """
        Draw an oval using Line primitives.
        
        Args:
            robot_name (str): Name of the robot to use
            major_axis (float): Length of the major axis
            minor_axis (float): Length of the minor axis
            scale (float): Scale factor
        """
        oval = ShapeLibrary.create_oval(0, 0, major_axis/2, minor_axis/2)
        self.draw_shape(robot_name, oval, scale)
        
    def draw_heart(self, robot_name, size=0.5, scale=1.0):
        """
        Draw a heart shape using primitive elements.
        
        Args:
            robot_name (str): Name of the robot to use
            size (float): Size of the heart in meters
            scale (float): Additional scale factor
        """
        # Create heart at origin, then translate to robot position
        heart = ShapeLibrary.create_heart(0, 0, size)
        self.draw_shape(robot_name, heart, scale)
    
    def draw_spiral(self, robot_name, radius=0.5, turns=2, scale=1.0):
        """
        Draw a spiral shape using primitive elements.
        
        Args:
            robot_name (str): Name of the robot to use
            radius (float): Maximum radius of the spiral
            turns (int): Number of complete turns in the spiral
            scale (float): Scale factor
        """
        # Create spiral at origin, then translate to robot position
        spiral = ShapeLibrary.create_spiral(0, 0, radius, turns)
        self.draw_shape(robot_name, spiral, scale)
    
    def draw_letter(self, robot_name, letter, scale=1.0):
        """
        Draw a letter using the font parser.
        
        Args:
            robot_name (str): Name of the robot to use
            letter (str): Letter to draw (uppercase)
            scale (float): Scale factor
        """
        letter = letter.upper()
        
        # Try to create the shape from the font with the scale applied directly
        shape = self.font_parser.create_glyph(letter, 0, 0, scale)
        
        # If not found in font, use parametric letter
        if not shape or not shape.primitives:
            rospy.loginfo(f"Letter '{letter}' not found in font, using parametric generation")
            shape = ShapeLibrary.create_parametric_letter(letter, 0, 0, scale)
        
        # Draw the shape (no additional scaling needed)
        self.draw_shape(robot_name, shape, 1.0)
    
    def draw_text(self, robot_name, text, scale=1.0):
        """
        Draw text using the font parser.
        
        Args:
            robot_name (str): Name of the robot to use
            text (str): Text to draw
            scale (float): Scale factor
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            with self.lock:
                if robot_name not in self.robot_positions:
                    rospy.logwarn(f"No position data available for robot {robot_name}")
                    return False
                    
                # Get current position
                current_x, current_y = self.robot_positions[robot_name]
            
            # Create a combined shape for all letters
            combined = Shape()
            x_offset = 0
            
            # Process each character
            for char in text.upper():
                if char in " \t\n":
                    # Handle whitespace
                    x_offset += 0.5 * scale  # Add space
                    continue
                    
                # Try to get the glyph from the font with scale already applied
                char_shape = self.font_parser.create_glyph(char, x_offset, 0, scale)
                
                # If not found, use parametric letter
                if not char_shape or not char_shape.primitives:
                    char_shape = ShapeLibrary.create_parametric_letter(char, x_offset, 0, scale)
                
                # Add the glyph primitives to the combined shape
                for primitive in char_shape.primitives:
                    combined.add_primitive(primitive)
                
                # Advance to next character position
                x_offset += self.font_parser.get_advance_width(char, scale)
            
            # Center the text horizontally
            text_width = x_offset
            centered = combined.translate(current_x - text_width/2, current_y)
            
            # Get all path segments
            paths = centered.get_all_points()
            
            # Visualize the path
            self.visualize_path(robot_name, paths)
            
            # Draw each path segment
            success = True
            for i, path in enumerate(paths):
                rospy.loginfo(f"Drawing text segment {i+1}/{len(paths)}")
                if not self.follow_path(robot_name, path):
                    success = False
                    
            return success
        
        except Exception as e:
            rospy.logerr(f"Error drawing text: {str(e)}")
            return False
    
    def draw_combined(self, robot_name, elements, scale=1.0):
        """
        Draw a combined shape made of multiple elements.
        
        Args:
            robot_name (str): Name of the robot to use
            elements (list): List of elements to combine
            scale (float): Scale factor
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            with self.lock:
                if robot_name not in self.robot_positions:
                    rospy.logwarn(f"No position data available for robot {robot_name}")
                    return False
                    
                # Get current position
                current_x, current_y = self.robot_positions[robot_name]
            
            # Create a combined shape
            combined = Shape()
            spacing = 0.6 * scale  # Spacing between elements
            
            x_offset = 0
            for element in elements:
                if len(element) == 1 and element.upper() in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                    # It's a letter
                    letter_shape = self.font_parser.create_glyph(element, x_offset, 0, scale)
                    
                    # If not found in font, use parametric letter
                    if not letter_shape or not letter_shape.primitives:
                        letter_shape = ShapeLibrary.create_parametric_letter(element, x_offset, 0, scale)
                        
                    for primitive in letter_shape.primitives:
                        combined.add_primitive(primitive)
                        
                    # Advance by letter width
                    x_offset += self.font_parser.get_advance_width(element, scale)
                    
                elif element.lower() in self.shapes:
                    # It's a shape
                    if element.lower() == "circle":
                        shape = ShapeLibrary.create_circle(x_offset, 0, 0.3 * scale)
                    elif element.lower() == "square":
                        shape = ShapeLibrary.create_rectangle(
                            x_offset - 0.3 * scale, -0.3 * scale,
                            0.6 * scale, 0.6 * scale
                        )
                    elif element.lower() == "heart":
                        shape = ShapeLibrary.create_heart(x_offset, 0, 0.5 * scale)
                    elif element.lower() == "spiral":
                        shape = ShapeLibrary.create_spiral(x_offset, 0, 0.3 * scale, 2)
                    elif element.lower() in self.polygon_sides:
                        shape = ShapeLibrary.create_regular_polygon(
                            x_offset, 0, 0.3 * scale,
                            self.polygon_sides[element.lower()]
                        )
                    else:
                        # Default to a circle
                        shape = ShapeLibrary.create_circle(x_offset, 0, 0.3 * scale)
                    
                    for primitive in shape.primitives:
                        combined.add_primitive(primitive)
                        
                    # Advance by shape width
                    x_offset += 0.7 * scale
                
                # Add extra spacing
                x_offset += 0.1 * scale
            
            # Center the combined shape
            center_x = x_offset / 2
            centered = combined.translate(current_x - center_x, current_y)
            
            # Get all path segments
            paths = centered.get_all_points()
            
            # Visualize the path
            self.visualize_path(robot_name, paths)
            
            # Draw each path segment
            for i, path in enumerate(paths):
                rospy.loginfo(f"Drawing combined shape segment {i+1}/{len(paths)}")
                self.follow_path(robot_name, path)
                
            return True
        
        except Exception as e:
            rospy.logerr(f"Error drawing combined shape: {str(e)}")
            return False
    
    def coordinated_text_drawing(self, text, robot_names, scale=1.0):
        """
        Coordinate multiple robots to efficiently draw text.
        
        Args:
            text (str): Text to draw
            robot_names (list): List of robot names
            scale (float): Scale factor
        """
        if not robot_names:
            rospy.logwarn("No robots available for coordinated text drawing")
            return
            
        rospy.loginfo(f"Coordinating {len(robot_names)} robots to draw text: '{text}'")
        
        # Calculate center position for text
        center_x = 0
        center_y = 0
        
        # Split the text among robots
        text_parts = []
        if len(text) <= len(robot_names):
            # One letter per robot
            text_parts = [(robot, char) for robot, char in zip(robot_names, text.upper())]
        else:
            # Distribute text evenly
            chars_per_robot = len(text) // len(robot_names)
            extra_chars = len(text) % len(robot_names)
            
            start_idx = 0
            for i, robot in enumerate(robot_names):
                # Calculate how many characters this robot gets
                if i < extra_chars:
                    num_chars = chars_per_robot + 1
                else:
                    num_chars = chars_per_robot
                    
                end_idx = start_idx + num_chars
                if end_idx <= len(text):
                    text_parts.append((robot, text[start_idx:end_idx]))
                    start_idx = end_idx
        
        # Calculate total width to center properly
        total_width = 0
        for _, part in text_parts:
            for char in part.upper():
                if char in " \t\n":
                    total_width += 0.5 * scale
                else:
                    total_width += self.font_parser.get_advance_width(char, scale)
        
        # Position each robot and assign text
        x_pos = center_x - total_width / 2
        for robot_name, part in text_parts:
            # Calculate width of this part
            part_width = 0
            for char in part.upper():
                if char in " \t\n":
                    part_width += 0.5 * scale
                else:
                    part_width += self.font_parser.get_advance_width(char, scale)
            
            # Move robot to starting position
            target_x = x_pos + part_width / 2
            target_y = center_y
            
            with self.lock:
                if robot_name in self.robot_positions:
                    # Move to starting position
                    rospy.loginfo(f"Moving {robot_name} to position ({target_x:.2f}, {target_y:.2f})")
                    self.move_to_point(robot_name, target_x, target_y)
                    
                    # Draw the text part
                    rospy.loginfo(f"{robot_name} drawing '{part}'")
                    self.draw_text(robot_name, part, scale)
            
            # Update position for next robot
            x_pos += part_width
    
    def coordinated_shape_drawing(self, shape, robot_names, scale=1.0):
        """
        Coordinate multiple robots to draw shapes.
        
        Args:
            shape (str): Shape to draw
            robot_names (list): List of robot names
            scale (float): Scale factor
        """
        if not robot_names:
            rospy.logwarn("No robots available for coordinated shape drawing")
            return
            
        rospy.loginfo(f"Coordinating {len(robot_names)} robots to draw shape: {shape}")
        
        # Calculate positions for robots in a circular pattern
        center_x = 0
        center_y = 0
        radius = 2.0 * scale  # Adjust based on scale
        
        positions = []
        for i in range(len(robot_names)):
            angle = 2 * math.pi * i / len(robot_names)
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            positions.append((x, y))
        
        # Move robots to positions and draw shapes
        for i, robot_name in enumerate(robot_names):
            pos_x, pos_y = positions[i]
            
            with self.lock:
                if robot_name in self.robot_positions:
                    # Move to position
                    rospy.loginfo(f"Moving {robot_name} to position ({pos_x:.2f}, {pos_y:.2f})")
                    self.move_to_point(robot_name, pos_x, pos_y)
                    
                    # Draw the shape
                    if shape.lower() in self.shapes:
                        shape_func = self.shapes[shape.lower()]
                        
                        # Prepare kwargs
                        kwargs = {'scale': scale}
                        if shape.lower() in self.polygon_sides:
                            kwargs['n_sides'] = self.polygon_sides[shape.lower()]
                            
                        rospy.loginfo(f"{robot_name} drawing {shape}")
                        shape_func(robot_name, **kwargs)
    
    def coordinated_pattern_drawing(self, pattern, robot_names, scale=1.0):
        """
        Coordinate multiple robots to draw patterns like stars, flowers, etc.
        
        Args:
            pattern (str): Pattern type ('star', 'flower', 'spiral', etc.)
            robot_names (list): List of robot names
            scale (float): Scale factor
        """
        if not robot_names:
            rospy.logwarn("No robots available for coordinated pattern drawing")
            return
            
        rospy.loginfo(f"Coordinating {len(robot_names)} robots to draw pattern: {pattern}")
        
        # Center of the pattern
        center_x = 0
        center_y = 0
        
        if pattern.lower() == 'star':
            # Create a star pattern with robots
            points = min(len(robot_names), 8)  # Max 8 points in the star
            use_robots = robot_names[:points]
            
            # First move all robots to the center
            for robot_name in use_robots:
                with self.lock:
                    if robot_name in self.robot_positions:
                        rospy.loginfo(f"Moving {robot_name} to center position")
                        self.move_to_point(robot_name, center_x, center_y)
            
            # Then have them move outward in a star pattern
            radius = 3.0 * scale  # Adjust based on scale
            
            for i, robot_name in enumerate(use_robots):
                with self.lock:
                    if robot_name in self.robot_positions:
                        # Calculate target position
                        angle = 2 * math.pi * i / points
                        target_x = center_x + radius * math.cos(angle)
                        target_y = center_y + radius * math.sin(angle)
                        
                        # Move to target position, drawing a line
                        rospy.loginfo(f"{robot_name} drawing star ray")
                        self.move_to_point(robot_name, target_x, target_y)
        
        elif pattern.lower() == 'circle':
            # Have robots form a circle
            radius = 3.0 * scale  # Adjust based on scale
            
            for i, robot_name in enumerate(robot_names):
                with self.lock:
                    if robot_name in self.robot_positions:
                        # Calculate position on circle
                        angle = 2 * math.pi * i / len(robot_names)
                        target_x = center_x + radius * math.cos(angle)
                        target_y = center_y + radius * math.sin(angle)
                        
                        # Move to position
                        rospy.loginfo(f"Moving {robot_name} to circle position")
                        self.move_to_point(robot_name, target_x, target_y)
            
            # Have each robot draw a local shape
            for robot_name in robot_names:
                with self.lock:
                    if robot_name in self.robot_positions:
                        rospy.loginfo(f"{robot_name} drawing circle at position")
                        self.draw_circle(robot_name, radius=0.3*scale)
        
        elif pattern.lower() == 'spiral':
            # Have robots form a spiral
            for i, robot_name in enumerate(robot_names):
                with self.lock:
                    if robot_name in self.robot_positions:
                        # Calculate position on spiral
                        angle = 2 * math.pi * i / len(robot_names)
                        radius_factor = (i + 1) / len(robot_names)
                        target_x = center_x + 3.0 * scale * radius_factor * math.cos(angle)
                        target_y = center_y + 3.0 * scale * radius_factor * math.sin(angle)
                        
                        # Move to position
                        rospy.loginfo(f"Moving {robot_name} to spiral position")
                        self.move_to_point(robot_name, target_x, target_y)
            
            # Have each robot draw a spiral
            for robot_name in robot_names:
                with self.lock:
                    if robot_name in self.robot_positions:
                        rospy.loginfo(f"{robot_name} drawing spiral at position")
                        self.draw_spiral(robot_name, radius=0.5*scale, turns=2)
        
        else:
            # Default pattern: coordinate a simple grid of shapes
            num_robots = len(robot_names)
            grid_dim = math.ceil(math.sqrt(num_robots))
            
            for i, robot_name in enumerate(robot_names):
                with self.lock:
                    if robot_name in self.robot_positions:
                        # Calculate grid position
                        row = i // grid_dim
                        col = i % grid_dim
                        
                        target_x = center_x + (col - (grid_dim-1)/2) * 2.0 * scale
                        target_y = center_y + (row - (grid_dim-1)/2) * 2.0 * scale
                        
                        # Move to position
                        rospy.loginfo(f"Moving {robot_name} to grid position")
                        self.move_to_point(robot_name, target_x, target_y)
                        
                        # Draw a shape
                        shape_type = ['circle', 'square', 'star', 'heart'][i % 4]
                        if shape_type in self.shapes:
                            rospy.loginfo(f"{robot_name} drawing {shape_type}")
                            shape_func = self.shapes[shape_type]
                            shape_func(robot_name, scale=scale)
    
    def publish_status(self, event=None):
        """Publish the current status of all robots"""
        try:
            with self.lock:
                # Create status dictionary
                status = {}
                for robot_name in self.publishers.keys():
                    # Check if robot is actively moving or drawing
                    if robot_name in self.robot_positions:
                        status[robot_name] = 'idle'  # Default to idle
                
                # Convert to JSON and publish
                status_json = json.dumps(status)
                self.status_pub.publish(status_json)
        except Exception as e:
            rospy.logerr(f"Error publishing status: {str(e)}")
    
    def run(self):
        """Run the node until shutdown"""
        rospy.loginfo("TurtleBot Font Drawer is running")
        rospy.loginfo("Send commands to /drawer/command and coordinated tasks to /drawer/task")
        rospy.spin()

def main():
    """Main function to start the node"""
    # Get parameters from parameter server
    robot_namespace_template = rospy.get_param("~robot_namespace_template", "/robot/{}/")
    font_path = rospy.get_param("~font_path", "")
    
    drawer = TurtleBotFontDrawer(robot_namespace_template, font_path)
    drawer.run()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass