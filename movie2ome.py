import numpy as np
import csv
import os
import re
import struct
from tifffile import TiffWriter
from datetime import datetime, timedelta, timezone
from ome_types.model import OME, Image, Pixels, Channel, Plane
import sys


######################## Variables to edit

movie_folder_path="C:/Users/ahm50/Documents/temp/movies/27_8_25"


output_path='C:/Users/ahm50/Documents/temp/movies'

## Used this: illumination=[0x01,0x60,0x40,0x80] # change depending on which channel is being used

compression_type = "raw"  # or "tiff_lzw" for lossless compression or "raw" for no compresison

user_channel_colors = {
    0: "#FFFFFFFF", # white for greyscale
    1: "#FFFF00FF",  
    2: "#00FFFFFF", 
    3: "#FF0000FF"    
}


################################## variables and functions for movie2tiff conversions #################################


CAMERA_MOVIE_MAGIC = 0x496D6554  # 'TemI' little-endian
CAMERA_HEADER_LEN = 56 # character length of camera name

# defines the endians to check later
G_BIG_ENDIAN = 4321
G_LITTLE_ENDIAN = 1234





_STRUCT = struct.Struct("<7I2Q3I")  # Define Binary layout:
                                    # "<" : Little-endian byte order
                                    # 7I  : 7 unsigned 32-bit integers - all bits from FrameHeader magic -> endianness
                                    # 2Q  : 2 unsigned 64-bit integers - time_Sec and time_nsec
                                    # 3I  : 3 unsigned 32 bit integers - width, height, stride


def hdr_from_bytes(buf:bytes):
    if len(buf)<CAMERA_HEADER_LEN:
        raise ValueError("Incomplete camera_save_struct header")
    fields=_STRUCT.unpack_from(buf)


    # assign each of the unpacked binary information into their respective datasets
    hdr = {
        "magic": fields[0],
        "version": fields[1],
        "type": fields[2],
        "pixelformat": fields[3],
        "length_header": fields[4],
        "length_data": fields[5],
        "endianness": fields[6],
        "time_sec": fields[7],
        "time_nsec": fields[8],
        "width": fields[9],
        "height": fields[10],
        "stride": fields[11],
    }




    if hdr["magic"] != CAMERA_MOVIE_MAGIC: # makes sure that the buf starts with TemI
            raise ValueError("Bad TemI magic")
    

    return hdr



# Generator function to find the hdr: directory of fields and extra: data length of image frame

def iterate_frames(data):
    off=0
    total = len(data) # finds the total number of bytes of movie 

    magic=CAMERA_MOVIE_MAGIC.to_bytes(4,'little') # what TemI is in bytes

    while off<total: # looks through list upto the number of bytes in the movie
        idx=data.find(magic,off) # finds the index of the data which begins TemI starting from the off index

        if idx==-1:
            break

        extra_start=idx + CAMERA_HEADER_LEN # data index after the TemI file and camera name length are taken into account
        
        hdr=hdr_from_bytes(data[idx : extra_start]) # creates a dictionary from the function with relavent parts of the image
        
        
        extra_end= idx + hdr["length_header"] # where the bytes end for this frame
        extra=data[extra_start:extra_end] # looks at the movie file from indexes from extra_start to extra_end
        frame_start=extra_end # frame starts after all the heathers are finished
        frame_end= frame_start + hdr["length_data"] # all the data fromt the movie file for that specific frame
        yield hdr, extra, memoryview(data)[frame_start:frame_end] # memory view can access the bytes of data without copying them
        off=frame_end # Start of the next iteration



################################## Functions for ome.tiff conversion ####################


def write_ome_tiff(stack, timestamp_list, aq_time, output_path, filename, user_channel_colors):
    """
    Write a 5D OME-TIFF (T,C,Z,Y,X) with OME-XML channel colors.

    stack: numpy array (T,C,Z,Y,X)
    timestamp_list: list of length T with Unix timestamps
    output_path: folder path
    filename: output filename (without extension)
    user_channel_colors: dict {channel_index: color_name_or_hex}
        color_name_or_hex can be a predefined name (e.g., "red") or hex string (e.g., "#FF8800")
    """
    T, C, Z, Y, X = stack.shape

    # Predefined ARGB colors (0xAARRGGBB)
    PREDEFINED_COLORS = {
        "grey":    '#808080FF',
        "red":     '#FF0000FF',
        "green":   '#00FF00FF',
        "blue":    '#0000FFFF',
        "yellow":  '#FFFF00FF',
        "cyan":    '#00FFFFFF',
        "magenta": '#FF00FFFF',
        "white":   '#FFFFFFFF',
        "black":   '#000000FF',
    }

    # Convert hex string to ARGB
    def hex_to_argb(hex_str):
        hex_str = hex_str.lstrip("#")
        if len(hex_str) == 6:  # RGB
            argb = 0xFF000000 | int(hex_str, 16)
        elif len(hex_str) == 8:  # ARGB
            argb = int(hex_str, 16)
        else:
            raise ValueError(f"Invalid hex color: {hex_str}")
        
        # Convert to signed 32-bit
        if argb > 0x7FFFFFFF:
            argb -= 0x100000000
        
        return argb

    # Build OME Channels
    channels = []
    for i in range(C):
        color_input = user_channel_colors.get(i)
        if isinstance(color_input, str):
            argb_value = hex_to_argb(color_input) if color_input.startswith("#") else hex_to_argb(PREDEFINED_COLORS.get(color_input))
        else:
            argb_value = 0xFFFFFFFF  # default white

        channels.append(
            Channel(
                id=f"Channel:{i}",
                name=f"C{i}",
                samples_per_pixel=1,
                color=argb_value,
                #excitation_wavelength=ex, # do properly when you have the correct data
            )
        )

    # Build OME Planes
    planes = [
    Plane(
            the_c=cm,
            the_z=zm,
            the_t=tm,
            delta_t=float(timestamp_list[tm]),
            #acquisition_time=datetime.fromtimestamp(aq_time[tm,cm,zm], tz=timezone.utc).isoformat()
        )
        for tm in range(T)   # outermost (slowest)
        for cm in range(C)   # middle
        for zm in range(Z)   # innermost (fastest)
    ]

    # Create Pixels and OME object
    pixels = Pixels(
        dimension_order="XYZCT",
        type=str(stack.dtype.name),
        size_x=X,
        size_y=Y,
        size_z=Z,
        size_c=C,
        size_t=T,
        channels=channels,
        planes=planes
    )

    ome = OME(images=[Image(id="Image:0", name="Sample Image", pixels=pixels)]) # need to change 'id' and 'name'
    ome_xml = ome.to_xml()

    # Write TIFF (no colormap needed for 5D stacks)
    with TiffWriter(f"{output_path}/{filename}.ome.tif", bigtiff=True) as tif:
        tif.write(
            stack,
            description=ome_xml,
            compression="lzw",
            photometric='minisblack'
        )
#################################### code '########################################






# List all files in the folder
files = [f for f in os.listdir(movie_folder_path) if os.path.isfile(os.path.join(movie_folder_path, f)) and f.lower().endswith(".movie")]



############## Obtain the different samples from this.
# Dictionary to store files grouped by identifier
grouped_files = {}

for file in files:
    if file.startswith("sample_") and "_enclosure" in file:
        # Extract the identifier between 'sample_' and '_enclosure'
        identifier = file.split("sample_")[1].split("_enclosure")[0]
        
        if identifier not in grouped_files:
            grouped_files[identifier] = []
        
        grouped_files[identifier].append(file)


############ Sort each group in increasing timestamp output is grouped_files
for identifier, files_list in grouped_files.items():
    # Extract 'n' and sort
    files_list.sort(key=lambda f: int(re.search(r'_timestamp_(\d+)_', f).group(1)))


sample_names=list(grouped_files.keys()) # obtain sample names



########### open all frame mapping which contains the data:
                            #"frame": frame_seq_id,
                            #"sample": sample_names,
                            #"number": num,
                            #"z": zi,
                            #"illum_wavelength": illumination,
                            #"illum_exposure_time": illum_expose,
                            #"illum_pwr": laser_pwr


########### Have them in all_data, split between different samples
all_data = []  # This will hold multiple instances of data

for sample_idx in range(len(sample_names)):
     
     # Open the CSV file
    with open(f"{movie_folder_path}/frame_mapping_{sample_idx}.csv", mode='r', newline='') as file:
        reader = csv.DictReader(file)
        data = [row for row in reader]  # Each row is a dictionary
        all_data.append(data)  # Store this instance of data


########## CAN CREATE SEQUENCE ARRAY USING FRAME NUMBERS AND CROSS REFERENCE WITH TIMESTAMPS. 
########## THEN ITERATE TIMESTAMPS AS PER WHAT IS GIVEN. - MAKE SURE TO CHANGE BETWEEN FILES. CAN BE DONE BY ITERATING TIMESTAMP WITH FILE ORDER, AS IT IS ORDERED

#################### iterate through frames 

for sample_idx in range(len(sample_names)):

    num_pos = sorted(set(d.get('number') for d in all_data[sample_idx] if 'number' in d)) # find number of positions per sample

    for num in num_pos:  # for number of samples in the movie files

        num_dicts = [d for d in all_data[sample_idx] if d.get('number')==num] # takes only a specific number from the sample
        #print(num_dicts)
        c_per_num=sorted(set(d.get('illum_wavelength') for d in num_dicts if 'illum_wavelength' in d)) # find number of channels per sample

        total_t=len(grouped_files[next(iter(grouped_files))])
        total_c=len(set(d.get('illum_wavelength') for d in num_dicts if 'illum_wavelength' in d))
        total_z=len(set(d.get('z') for d in num_dicts if 'z' in d))
        stack = np.zeros((total_t, total_c, total_z, 2200, 3208), dtype=np.uint16 )


        timestamp_list = []

        aq_time = np.zeros((total_t,total_c,total_z), dtype=np.int32)

        
        
        

        for ti, file in enumerate(grouped_files[sample_names[sample_idx]]): # takes the ordered timelapse for each sample and reads them in order of timelapse T
            movie_name=f"{movie_folder_path}/{file}"
            with open(movie_name, "rb") as f:
                data = f.read() # reads the movie file as bytes

            timestamp_list.append(int(re.search(r'_timestamp_(\d+)_', file).group(1))) 
            
            frames=list(iterate_frames(data)) # creates a list of all frames from data
            n_frames=len(frames) # finds the number of frames

            if n_frames == 0:
                    raise ValueError("No TemI frames found") # incase no frames are in image

            #print(n_frames)
            #print(file)
            
    
        
            for ci,c in enumerate(c_per_num): # for number of channels in sample
                c_dicts=[d for d in num_dicts if d.get('illum_wavelength')==c] # takes only a specific channel from the sample
                
                z_per_c=sorted({int(d['z']) for d in c_dicts if 'z' in d})
                
                for zi,z in enumerate(z_per_c): # for number of z_stacks in channel
                    z_dicts=[d for d in c_dicts if d.get('z')==str(z)]
                    
                    frame=set(d.get('frame') for d in z_dicts if 'frame' in d)
                    frame = int(list(frame)[0])

                    if len(all_data[sample_idx]) + 1 == n_frames: # if statement as sometimes 1st frame is incorrect for use
                        frame_idx=frame+1
                    elif len(all_data[sample_idx]) == n_frames:
                        frame_idx=frame
                    else:
                        raise ValueError("Frames do not match")
                    
                    ############# Read and store frames in OME-Tiff
                    
                    hdr, extra, mv = frames[frame_idx - 1] # indexing for individual frame

                    h , w = hdr["height"] , hdr["width"] # finds the height and width of the frame
                    stride=hdr["stride"] # difference in bytes between start of 1 row and the next row of pixels

                    arr=np.empty((h, w), dtype=np.uint16) # output image is of the 16 bit type. This initialises the matrix
                    for r in range(h):
                        off = r * stride # number of bytes in each row, i.e. total number of bytes in that row
                        row=np.frombuffer(mv[off : off + w * 2], dtype=np.uint16, count=w) # creates array from buffer of binary data using memory view between off: off + w and reads w number of elements 
                        if hdr["endianness"] == G_BIG_ENDIAN: # check endianess and make sure it is little endian, i.e. byte order is 12 34 and not 43 21
                            row = row.byteswap()
                        arr[r] = row
                    
                    frame_time=hdr["time_sec"] + hdr["time_nsec"]/1e9



                    aq_time[ti,ci,zi]=frame_time
                    stack[ti, ci, zi,:,:] = arr
                    
        #stack2 = np.transpose(stack, (4, 3, 2, 1, 0))

        write_ome_tiff(
            stack=stack,
            timestamp_list=timestamp_list,
            aq_time=aq_time,
            output_path=output_path,
            filename=f"sample_{sample_names[sample_idx]}_position_{num}",
            user_channel_colors=user_channel_colors
            
        )