#!/usr/bin/env python

import sys
from PIL import Image
import numpy as np
import os

def read_bitmap(file_path):
    """
    Read a bitmap image using the PIL module and return detailed information.
    
    Args:
        file_path (str): Path to the bitmap image file
        
    Returns:
        tuple: (info_dict, image_object, pixel_array)
    """
    try:
        # Open the image file
        img = Image.open(file_path)
        
        # Get basic image properties
        width, height = img.size
        mode = img.mode
        format = img.format
        
        # Check if it's actually a bitmap
        if img.format != "BMP":
            print(f"Note: The image format is {img.format}, not BMP. Processing anyway.")
        
        # Convert image to RGB if it isn't already
        if mode != "RGB":
            img_rgb = img.convert("RGB")
        else:
            img_rgb = img
            
        # Convert image to numpy array for analysis
        pixel_array = np.array(img_rgb)
        
        # Extract palette information if applicable
        palette = None
        if hasattr(img, "palette") and img.palette:
            palette = img.palette
        
        # Determine bit depth based on mode
        if mode == "1":  # 1-bit pixels, black and white
            bit_depth = 1
            colors = 2
        elif mode == "L":  # 8-bit pixels, grayscale
            bit_depth = 8
            colors = 256
        elif mode == "P":  # 8-bit pixels, mapped to any other mode using a color palette
            bit_depth = 8
            colors = len(img.getcolors(maxcolors=256)) if img.getcolors(maxcolors=256) else "More than 256"
        elif mode == "RGB":  # 3x8-bit pixels, true color
            bit_depth = 24
            colors = "Up to 16.7 million"
        elif mode == "RGBA":  # 4x8-bit pixels, true color with transparency
            bit_depth = 32
            colors = "Up to 16.7 million with alpha"
        else:
            bit_depth = "Unknown"
            colors = "Unknown"
        
        # Get file information using low-level file operations
        with open(file_path, 'rb') as f:
            header = f.read(54)  # BMP header is typically 54 bytes
            if header[0:2] == b'BM':  # Check BMP signature
                file_size = int.from_bytes(header[2:6], byteorder='little')
                data_offset = int.from_bytes(header[10:14], byteorder='little')
                compression = int.from_bytes(header[30:34], byteorder='little')
            else:
                file_size = "Unknown"
                data_offset = "Unknown"
                compression = "Unknown"
        
        # Calculate image statistics
        if pixel_array.size > 0:
            min_val = pixel_array.min(axis=(0, 1))
            max_val = pixel_array.max(axis=(0, 1))
            mean_val = pixel_array.mean(axis=(0, 1))
        else:
            min_val = max_val = mean_val = None
        
        # Return information as dictionary
        info = {
            "width": width,
            "height": height,
            "mode": mode,
            "format": format,
            "bit_depth": bit_depth,
            "colors": colors,
            "file_size": file_size,
            "data_offset": data_offset,
            "compression": compression,
            "min_values": min_val,
            "max_values": max_val,
            "mean_values": mean_val,
            "palette": palette
        }
        
        return info, img, pixel_array
        
    except Exception as e:
        print(f"Error reading bitmap: {e}")
        return None, None, None

def print_image_info(info):
    """Print formatted image information"""
    print("\nImage Information:")
    print(f"Format: {info['format']}")
    print(f"Dimensions: {info['width']} x {info['height']} pixels")
    print(f"Mode: {info['mode']}")
    print(f"Bit Depth: {info['bit_depth']}")
    print(f"Colors: {info['colors']}")
    
    if info['file_size'] != "Unknown":
        print(f"File Size: {info['file_size']} bytes")
    
    if info['compression'] != "Unknown":
        comp_types = {
            0: "None (BI_RGB)",
            1: "RLE 8-bit (BI_RLE8)",
            2: "RLE 4-bit (BI_RLE4)",
            3: "Bitfields (BI_BITFIELDS)",
            4: "JPEG (BI_JPEG)",
            5: "PNG (BI_PNG)"
        }
        comp_name = comp_types.get(info['compression'], f"Unknown ({info['compression']})")
        print(f"Compression: {comp_name}")
    
    if info['min_values'] is not None:
        print(f"Min RGB Values: {info['min_values']}")
        print(f"Max RGB Values: {info['max_values']}")
        print(f"Mean RGB Values: {info['mean_values'].astype(int)}")
    
    if info['palette'] is not None:
        print("Image has a color palette")

def analyze_pixel_data(pixel_array, sample_size=10):
    """Analyze and print information about pixel data"""
    if pixel_array is None:
        return
    
    print(f"\nImage Shape: {pixel_array.shape}")
    
    # Print a sample of pixel values
    if len(pixel_array.shape) == 3 and pixel_array.shape[0] >= sample_size and pixel_array.shape[1] >= sample_size:
        print(f"\nPixel Sample ({sample_size}x{sample_size} from top-left corner):")
        for y in range(min(sample_size, pixel_array.shape[0])):
            for x in range(min(sample_size, pixel_array.shape[1])):
                if len(pixel_array.shape) == 3 and pixel_array.shape[2] == 3:
                    r, g, b = pixel_array[y, x]
                    if x == 0:
                        print(f"Row {y}: ", end="")
                    print(f"({r},{g},{b}) ", end="")
            print()  # New line after each row

def rgb_to_4bit(r, g, b):
    """
    Convert RGB values to a 4-bit color index (16 colors).
    Uses a basic 16-color palette commonly used in embedded systems.
    
    Returns:
        int: A value from 0-15 representing the closest color
    """
    # Standard 16-color VGA palette
    palette = [
        (0, 0, 0),       # 0: Black
        (128, 0, 0),     # 1: Maroon
        (0, 128, 0),     # 2: Green
        (128, 128, 0),   # 3: Olive
        (0, 0, 128),     # 4: Navy
        (128, 0, 128),   # 5: Purple
        (0, 128, 128),   # 6: Teal
        (192, 192, 192), # 7: Silver
        (128, 128, 128), # 8: Gray
        (255, 0, 0),     # 9: Red
        (0, 255, 0),     # 10: Lime
        (255, 255, 0),   # 11: Yellow
        (0, 0, 255),     # 12: Blue
        (255, 0, 255),   # 13: Fuchsia
        (0, 255, 255),   # 14: Aqua
        (255, 255, 255)  # 15: White
    ]
    
    # Find the closest color in the palette
    min_distance = float('inf')
    best_idx = 0
    
    for idx, (pr, pg, pb) in enumerate(palette):
        # Calculate Euclidean distance in RGB space
        distance = (r - pr) ** 2 + (g - pg) ** 2 + (b - pb) ** 2
        if distance < min_distance:
            min_distance = distance
            best_idx = idx
            
    return best_idx

def generate_c_array(img, file_path, variable_name=None):
    """
    Generate a C array representation of the image using 4 bits per pixel (16 colors).
    
    Args:
        img: PIL Image object
        file_path: Path to output the C array code
        variable_name: Name for the C array variable (defaults to filename)
        
    Returns:
        str: Path to the generated C file
    """
    if img is None:
        print("Error: No image provided")
        return None
        
    # Convert image to RGB if it isn't already
    if img.mode != "RGB":
        img = img.convert("RGB")
    
    # Get image dimensions
    width, height = img.size
    
    # Define variable name if not provided
    if variable_name is None:
        base_name = os.path.basename(file_path)
        variable_name = os.path.splitext(base_name)[0].lower()
        # Remove non-alphanumeric characters
        variable_name = ''.join(c for c in variable_name if c.isalnum() or c == '_')
        if not variable_name[0].isalpha() and variable_name[0] != '_':
            variable_name = 'img_' + variable_name
    
    # Calculate array size - each byte holds two 4-bit pixels
    bytes_per_row = (width + 1) // 2  # Round up for odd widths
    total_bytes = bytes_per_row * height
    
    # Prepare output file
    output_path = os.path.splitext(file_path)[0] + '.c'
    
    with open(output_path, 'w') as f:
        # Write header comment
        f.write(f"/**\n")
        f.write(f" * 4-bit per pixel (16 colors) bitmap converted from {os.path.basename(file_path)}\n")
        f.write(f" * Size: {width}x{height} pixels\n")
        f.write(f" * Generated on {os.path.basename(sys.argv[0])}\n")
        f.write(f" */\n\n")
        
        # Write array dimensions as defines
        f.write(f"#define {variable_name.upper()}_WIDTH {width}\n")
        f.write(f"#define {variable_name.upper()}_HEIGHT {height}\n")
        f.write(f"#define {variable_name.upper()}_BPP 4\n\n")
        
        # Write color palette description
        f.write("/* 4-bit color palette (16 colors):\n")
        f.write(" * 0: Black       (0,0,0)\n")
        f.write(" * 1: Maroon      (128,0,0)\n")
        f.write(" * 2: Green       (0,128,0)\n")
        f.write(" * 3: Olive       (128,128,0)\n")
        f.write(" * 4: Navy        (0,0,128)\n")
        f.write(" * 5: Purple      (128,0,128)\n")
        f.write(" * 6: Teal        (0,128,128)\n")
        f.write(" * 7: Silver      (192,192,192)\n")
        f.write(" * 8: Gray        (128,128,128)\n")
        f.write(" * 9: Red         (255,0,0)\n")
        f.write(" * 10: Lime       (0,255,0)\n")
        f.write(" * 11: Yellow     (255,255,0)\n")
        f.write(" * 12: Blue       (0,0,255)\n")
        f.write(" * 13: Fuchsia    (255,0,255)\n")
        f.write(" * 14: Aqua       (0,255,255)\n")
        f.write(" * 15: White      (255,255,255)\n")
        f.write(" */\n\n")
        
        # Start array declaration
        f.write(f"const unsigned char {variable_name}[] = {{\n    ")
        
        # Process image data
        bytes_written = 0
        
        for y in range(height):
            for x in range(0, width, 2):  # Process 2 pixels at a time (8 bits)
                # Get first pixel
                r1, g1, b1 = img.getpixel((x, y))
                color1 = rgb_to_4bit(r1, g1, b1)
                
                # Get second pixel (handle edge case for odd width)
                if x + 1 < width:
                    r2, g2, b2 = img.getpixel((x + 1, y))
                    color2 = rgb_to_4bit(r2, g2, b2)
                else:
                    color2 = 0  # Padding for odd width
                
                # Combine two 4-bit colors into one byte
                byte_value = (color1 << 4) | color2
                
                f.write(f"0x{byte_value:02X}")
                bytes_written += 1
                
                # Format the output for readability
                if bytes_written < total_bytes:
                    f.write(", ")
                    if bytes_written % 16 == 0:
                        f.write("\n    ")
        
        # Close the array
        f.write("\n};\n")
    
    print(f"C array generated successfully: {output_path}")
    
    # Return stats about the array
    stats = {
        "output_path": output_path,
        "variable_name": variable_name,
        "size_bytes": total_bytes,
        "width": width,
        "height": height
    }
    
    return stats

def main():
    if len(sys.argv) < 2:
        print("Usage: python bitmap_reader.py <path_to_bitmap_file>")
        return
    
    file_path = sys.argv[1]
    info, img, pixel_array = read_bitmap(file_path)
    
    if info is not None:
        print_image_info(info)
        
        # Ask if user wants to see pixel data analysis
        response = input("\nShow pixel data sample? (y/n): ")
        if response.lower() == 'y':
            analyze_pixel_data(pixel_array)
        
        # Ask if user wants to generate C array
        response = input("\nGenerate 4-bit per pixel C array? (y/n): ")
        if response.lower() == 'y':
            var_name = input("Enter variable name (or press Enter for default): ").strip()
            var_name = var_name if var_name else None
            stats = generate_c_array(img, file_path, var_name)
            
            if stats:
                print("\nC Array Statistics:")
                print(f"Variable name: {stats['variable_name']}")
                print(f"Image dimensions: {stats['width']}x{stats['height']}")
                print(f"Array size: {stats['size_bytes']} bytes")
                print(f"Memory savings: {100 - (stats['size_bytes'] / (stats['width'] * stats['height'] * 3)) * 100:.1f}% compared to 24-bit RGB")
        
        # Ask if user wants to display the image
        response = input("\nDisplay the image? (y/n): ")
        if response.lower() == 'y':
            img.show()

if __name__ == "__main__":
    main()