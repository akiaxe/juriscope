import socket
from datetime import timedelta
import numpy as np
import time
import csv
import pickle
import os

################################## CHANGE EVERYTHING IN THIS SECTION ######################

main_folder='/home/jb2547/data/20250902' # Give the file directory you are saving to

objective=20 # Set either 20 or 40 for which objective is being used

samples=2 #How many wells/capillaries do you have

sample_names=['no enzyme','w enzyme']

z_stack=1# Say if a z stack is being used; 1=yes, 0= not
timelapse=1 # say if you want time lapse on; 1=yes, 0= not



z_c_order='zc'  # zc -  Does all channels in each z first before moving to the next z value -- should be more precise
                # cz -  Does all z values per channel before moving to the next channel --- should be faster

##################################### time range

interval=np.array([3,6]) # interval time beteen imaging cycles in minutes


interval_time=np.array([9,18]) #  total time for each interval in min, same order as interval

#interval_time=interval_time*60 ## Remove this if you are running in minutes

## Further intervals can be added

################################### z step range
z_range=1 # +- this number, e.g. if it is 20, it is -20Âµm to +20 Âµm
z_step=1 # step size between z range, i.e. if z_range is 20 and z_step is 1, you would have 41 slices

## Temperature and heating time

enclosure_heating=1 # Say if you want enclosure heating on; 1=yes 2=no

temperature_enclosure = 30
heating_time = "0:0:1" # h:m:s

#### Peltier temperature variables

peltier=1 # say if you want peltier on; 1=yes, 0=not

pelt_start_temp=30
pelt_end_temp=20
pelt_step=-1

pelt_temp_list=list(range(pelt_start_temp, pelt_end_temp - 1, pelt_step))

pelt_time_temp_threshold=[29,28,27] # Cutoff temperature where the pelt_wait_time moves to the next wait time sequence
pelt_wait_time=[4200,30,10,5] # wait time for peltier. Should be threshold +1


################################## Illumination 


illumination=[0x20,0x80] # change depending on which channel is being used
illum_expose=[10000,100000] # change depending on exposure time for each channel
laser_pwr=[0.4,1] # change depending on the laser power wanted, between 0 and 1

##Specify illumination setting
#first: define the illumination channel that you want to use:
#ch postion 1: BF 475 nm: 0 bit value , Hex value: 0x01
#ch postion 2: BF 528 nm: 2 bit value, Hex value: 0x02
#ch postion 3: BF 625 nm: 4 bit value, Hex value: 0x04
#ch postion 4: BF 655 nm: 8 bit value, Hex value: 0x08
#ch postion 5: Fluo 385 nm: 16 bit value, Hex value: 0x10
#ch postion 6: Fluo 475 nm: 32 bit value, Hex value: 0x20  --- error in microscope so turns on both 475 and 528 on the 40x
#ch postion 7: Fluo 528 nm: 64 bit value, Hex value: 0x40
#ch postion 8: Fluo 625 nm: 128 bit value, Hex value: 0x80


###################################### END OF USER SET PARAMETERS #################################################



#################################### Initialisation

os.makedirs(main_folder, exist_ok=True)  # creates dircetory if missing

#assign cama name

if objective==20:
    camera_name = "Genicam FLIR Blackfly S BFS-U3-70S7M 0159F410" # 20x
elif objective==40:
    camera_name = "Genicam FLIR Blackfly S BFS-U3-70S7M 0159F411" #40x


########### Set peltier array
assert len(pelt_wait_time) == len(pelt_time_temp_threshold) + 1 # check if pelt wait time matches threshold

pelt_wait_schedule = []
for temp in pelt_temp_list:
    for i, threshold in enumerate(pelt_time_temp_threshold):
        if temp > threshold:
            pelt_wait_schedule.append(pelt_wait_time[i])
            break
    else:
        # If no threshold matched (i.e., temp <= all thresholds)
        pelt_wait_schedule.append(pelt_wait_time[-1])

############# initialise timestamp list
timestamp=[]


##### Open and configure a socket for allow the comunication between the py script and the temika software 
#the Host and Port are the same for both microscopes
HOST = "127.0.0.1"
PORT = 60000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # AF_INET: set address family as IPv4, SOCK_STREAM sockeet type TCP
sock.connect((HOST, PORT)) # connect to the host and the port given
sock.settimeout(120.) #timeout for connection is set at 1 second
reply = sock.recv(1024)





####################################### Funtions ####################################




def script_send_command(command, reply_wait, terminator=b'\n\r', max_wait=120):
    """
    Sends a command to the socket and optionally waits for a reply ending in `terminator`.

    Args:
        command (str or bytes): The command to send.
        reply_wait (bool): Whether to wait for a reply.
        terminator (bytes): Sequence marking the end of a message.
        max_wait (int): Maximum time to wait for reply in seconds.

    Returns:
        bytes or None: Reply without terminator, or None if no reply expected.
    """

    if isinstance(command, str):
        command = command.encode('utf-8')

    sock.sendall(command)

    if reply_wait:
        reply = b""
        sock_start_time = time.time()

        while True:
            if time.time() - sock_start_time > max_wait:
                raise TimeoutError(f"No full reply received within {max_wait} seconds.")

            try:
                chunk = sock.recv(1024)
                if not chunk:
                    raise ConnectionError("Socket connection closed by the remote host.")

                reply += chunk

                if reply.endswith(terminator):
                    return reply[:-len(terminator)]

            except socket.timeout:
                continue  # just keep waiting

    else:
        time.sleep(0.1)


# Function to get gets the current x, y, z, and a stepper motor positions from the microscope
def script_get_position():
    script_send_command(b'<microscopeone>', False) # sends command to open <microscopeone>

    # sends command to find get the stepper axis and 
    script_send_command(b'<stepper axis="x">', True) 
    reply = script_send_command(b'<status></status>', True)# gives the returned value
    script_send_command(b'</stepper>', False)
    x = float(reply.split()[3]) # receives the data anbd splits it where there is a whitespace and and takes the 4th word. In this case it is the x value
    
    # process repeated for y,z and apprature
    script_send_command(b'<stepper axis="y">', True)
    reply = script_send_command(b'<status></status>', True)
    script_send_command(b'</stepper>', False)
    y = float(reply.split()[3])

    script_send_command(b'<stepper axis="z">', True)
    reply = script_send_command(b'<status></status>', True)
    script_send_command(b'</stepper>', False)
    z = float(reply.split()[3])

    script_send_command(b'<stepper axis="a">', True)  # PFS offset value (aka autofocus on jurijscope)
    reply = script_send_command(b'<status></status>', True)
    script_send_command(b'</stepper>', False)
    a = float(reply.split()[3])

    script_send_command(b'</microscopeone>', False)

    return [x, y, z, a]

# Function to tell the microscope where to move without pfs
def script_print_move_no_pfs(vec):
    script_send_command("\t<microscopeone>\n", False)
    script_send_command("\t\t<afocus>\n",False)
    script_send_command("\t\t\t<enable>OFF</enable>\n",True)
    script_send_command("\t\t</afocus>\n", False)

    axes = ['x', 'y', 'z']
    speeds = [10000, 10000, 100]

    for axis, value, speed in zip(axes, vec, speeds): #The zip() function takes multiple iterables (like lists) as arguments and aggregates elements from each of them.
        script_send_command(f'\t\t<stepper axis="{axis}">\n',True)
        script_send_command(f"\t\t\t<move_absolute>{value} {speed}</move_absolute>\n",False)
        #time.sleep(1)
        script_send_command("\t\t</stepper>\n", False)

    for axis in axes:
        script_send_command(f'\t\t<stepper axis="{axis}">\n', True)
        script_send_command("\t\t\t<wait_moving_end></wait_moving_end>\n", True)
        script_send_command("\t\t</stepper>\n\n", False)

    script_send_command("\t</microscopeone>\n", False)

# Function to tell the microscope where to move with pfs on
def script_print_move(vec):
    script_send_command("\t<microscopeone>\n", False)

    script_send_command("\t\t<afocus>\n", False)
    script_send_command("\t\t\t<enable>OFF</enable>\n", True)
    script_send_command("\t\t</afocus>\n", False)

    axes = ['x', 'y', 'z', 'a']
    speeds = [10000, 10000, 100, 10000]

    for axis, value, speed in zip(axes, vec, speeds):
        script_send_command(f'\t\t<stepper axis="{axis}">\n', True)
        script_send_command(f"\t\t\t<move_absolute>{value} {speed}</move_absolute>\n", False)
        #time.sleep(3) # change to a bit higher for cappillaries
        script_send_command("\t\t</stepper>\n", False)

    for axis in axes:
        script_send_command(f'\t\t<stepper axis="{axis}">\n', True)
        script_send_command("\t\t\t<wait_moving_end></wait_moving_end>\n", True)
        script_send_command("\t\t</stepper>\n\n", False)

    script_send_command("\t\t<afocus>\n", False)
    script_send_command("\t\t\t<enable>ON</enable>\n", True)
    script_send_command("\t\t\t<wait_lock>0.2 10.3</wait_lock>\n", True) 
    script_send_command("\t\t</afocus>\n", False)

    script_send_command("\t</microscopeone>\n", False)


def peltier_temp(pelt_temp,pelt_idx):
    script_send_command("\t<microscopeone>\n", False)
    script_send_command('\t\t<pwr number="0">\n', False)
    script_send_command("\t\t\t<feedback>\n", False)
    script_send_command(f"\t\t\t\t<set>{pelt_temp}</set>\n", True)
    script_send_command("\t\t\t\t<enable>ON</enable>\n", True)
    script_send_command("\t\t\t</feedback>\n", False)
    script_send_command("\t\t</pwr>\n", False)
    script_send_command("\t</microscopeone>\n", False)
    if pelt_idx==0:
        time.sleep(30)
    else:
        time.sleep(1)


# Function to tell the microscope to go in resting mode, pull fully down the objective and fully up the condeser, no pfs
def script_print_move_z_c_reset():
    script_send_command("\t<microscopeone>\n", False)
    
    #disable the pfs
    script_send_command("\t\t<afocus>\n", False)
    script_send_command("\t\t\t<enable>OFF</enable>\n", True)
    script_send_command("\t\t</afocus>\n", False)

    #move the objective (z axis) to -10000 and woth speed 100
    script_send_command(f'\t\t<stepper axis="z">\n', False)
    script_send_command(f"\t\t\t<move_absolute>-10000 100</move_absolute>\n", False)
    #time.sleep(100)
    script_send_command("\t\t</stepper>\n", False)

    script_send_command(f'\t\t<stepper axis="z">\n', True)
    script_send_command("\t\t\t<wait_moving_end></wait_moving_end>\n", True)
    script_send_command("\t\t</stepper>\n\n", False)
     
    #move the condenser (c axis) to +720000 and woth speed 30000
    script_send_command(f'\t\t<stepper axis="c">\n', False)
    script_send_command(f"\t\t\t<move_absolute>72000 30000</move_absolute>\n", False)
    #time.sleep(3)
    script_send_command("\t\t</stepper>\n", False)

    script_send_command(f'\t\t<stepper axis="c">\n', False)
    script_send_command("\t\t\t<wait_moving_end></wait_moving_end>\n", True)
    script_send_command("\t\t</stepper>\n\n", False)

    script_send_command("\t</microscopeone>\n", False)


# function for vectors to obtain z stack
def script_z_stack(vec,z_range,step):
    x, y, z, a = vec
    return [[x, y, z + offset, a] for offset in range(-z_range, z_range + 1, step)]

# Function for vectors to obtain z stack
def script_print_move_z_move(step):
    script_send_command("\t<microscopeone>\n",False)
 
    script_send_command("\t\t<afocus>\n",False)
    script_send_command("\t\t\t<enable>OFF</enable>\n",True)
    script_send_command("\t\t</afocus>\n",False)
 
    script_send_command(f'\t\t<stepper axis="z">\n',True)
    script_send_command(f"\t\t\t<move_absolute>{step} 100</move_absolute>\n",True)
    
    script_send_command("\t\t</stepper>\n",False)
 
    script_send_command(f'\t\t<stepper axis="z">\n',True)
    script_send_command("\t\t\t<wait_moving_end></wait_moving_end>\n",True)
    script_send_command("\t\t</stepper>\n\n",True)
 
    script_send_command("\t</microscopeone>\n",False)

### Main function which enables does the recording 
def script_print_sample(filename, positions, number):
    #Writes XML commands for imaging all positions in a capillary/well, including metadata, camera/microscope control, and image acquisition.
    #saves the metadata 
    #script_send_command("<!-- Image all positions in a capillary/well with z -->\n", False)
    #script_send_command("\t<!-- Start recording. -->\n", False)
    script_send_command("\t<save>\n", False)
    script_send_command(f"\t\t<header>\n{filename}\nNumber of positions {number}.\n\t\t</header>\n", True)
    script_send_command(f"\t\t<basename>{main_folder}/{filename}</basename>\n", True) # This is the file_location
    script_send_command("\t\t<append>DATE</append>\n", True)
    script_send_command("\t</save>\n", False)

    ########### acquisition

    ## Turn on camera with trigger mode
    script_send_command(f'\t<camera name="{camera_name}">\n', False)
    script_send_command(f'\t<genicam><enumeration feature="TriggerMode">On</enumeration></genicam>\n', False)
    script_send_command("\t\t<record>ON</record>\n", True)
    script_send_command("\t</camera>\n", False)

    ## Turn on metadata for environment
    #script_send_command("\t<microscopeone>\n", False)
    #script_send_command("\t\t<record>ON</record>\n", True)
    #script_send_command("\t</microscopeone>\n\n", False)

    ## Move and acquire image
    #script_send_command("\t<!-- Move to a position and acquire images -->\n", False)

    for i in range(number):

        if z_stack == 1:
            if z_c_order=='zc':

                pos_vec=positions[i] # select vector in position i
                script_print_move(pos_vec) # moves to the given vector

                
                for step in range (-z_range, z_range+1,z_step):
                    script_print_move_z_move(pos_vec[2]+step)

                    for illum, exp, laser in zip(
                        illumination,
                        illum_expose,
                        laser_pwr
                    ):
                        
                        #records the data given the illumination and exposure time
                        script_send_command("\t<microscopeone>\n", False)
                        script_send_command("\t\t<illumination>\n", False)
                        script_send_command(f"\t\t\t<enable>{illum}</enable>\n", True)
                        script_send_command(f'\t\t\t<value number="2">{laser}</value>\n', True) # double check number
                        script_send_command("\t\t</illumination>\n", False)
                        script_send_command("\t</microscopeone>\n", False)
                        script_send_command(f'\t<camera name="{camera_name}">\n', False)
                        script_send_command(f'\t<genicam><float feature="ExposureTime">{exp}</float></genicam>\n', True)
                        script_send_command("\t\t<send_trigger></send_trigger>\n", False)
                        script_send_command("\t</camera>\n\n", True)
                        # Turn off laser power for all channels
                        script_send_command("\t<microscopeone>\n", False)
                        script_send_command("\t\t<illumination>\n", False)
                        script_send_command(f"\t\t\t<enable>0</enable>\n", False)
                        script_send_command("\t\t</illumination>\n", False)
                        script_send_command("\t</microscopeone>\n", False)


                        image_time=time.time()
                        timestamp_time=(image_time-overall_start_time) / 60

                        timestamp.append(timestamp_time)

            elif z_c_order=='cz':

                pos_vec=positions[i] # select vector in position i
                script_print_move(pos_vec) # moves to the given vector

                for illum, exp, laser in zip(
                        illumination,
                        illum_expose,
                        laser_pwr
                    ):

                    
                    
                    for step in range (-z_range, z_range+1,z_step):
                        script_print_move_z_move(pos_vec[2]+step)

                        #records the data given the illumination and exposure time
                        script_send_command("\t<microscopeone>\n", False)
                        script_send_command("\t\t<illumination>\n", False)
                        script_send_command(f"\t\t\t<enable>{illum}</enable>\n", True)
                        script_send_command(f'\t\t\t<value number="2">{laser}</value>\n', True) # double check number
                        script_send_command("\t\t</illumination>\n", False)
                        script_send_command("\t</microscopeone>\n", False)
                        script_send_command(f'\t<camera name="{camera_name}">\n', False)
                        script_send_command(f'\t<genicam><float feature="ExposureTime">{exp}</float></genicam>\n', True)
                        script_send_command("\t\t<send_trigger></send_trigger>\n", False)
                        script_send_command("\t</camera>\n\n", True)
                        # Turn off laser power for all channels
                        script_send_command("\t<microscopeone>\n", False)
                        script_send_command("\t\t<illumination>\n", False)
                        script_send_command(f"\t\t\t<enable>0</enable>\n", False)
                        script_send_command("\t\t</illumination>\n", False)
                        script_send_command("\t</microscopeone>\n", False)

                        image_time=time.time()
                        timestamp_time=(image_time-overall_start_time) / 60

                        timestamp.append(timestamp_time)
                    
                    

        else:
                vec = positions[i]
                script_print_move(vec)

                for illum, exp, laser in zip(illumination, illum_expose, laser_pwr):
                    script_send_command("\t<microscopeone>\n", False)
                    script_send_command("\t\t<illumination>\n", False)
                    script_send_command(f"\t\t\t<enable>{illum}</enable>\n", False)
                    script_send_command(f'\t\t\t<value number="2">{laser}</value>\n', True)
                    script_send_command("\t\t</illumination>\n", False)
                    script_send_command("\t</microscopeone>\n", False)
                    script_send_command(f'\t<camera name="{camera_name}">\n', False)
                    script_send_command(f'\t<genicam><float feature="ExposureTime">{exp}</float></genicam>\n', True)
                    script_send_command("\t\t<send_trigger></send_trigger>\n", False)
                    script_send_command("\t</camera>\n\n", False)
                    # Turn off laser power for all channels
                    script_send_command("\t<microscopeone>\n", False)
                    script_send_command("\t\t<illumination>\n", False)
                    script_send_command(f"\t\t\t<enable>0</enable>\n", False)
                    script_send_command("\t\t</illumination>\n", False)
                    script_send_command("\t</microscopeone>\n", False)

                    image_time=time.time()
                    timestamp_time=(image_time-overall_start_time) / 60

                    timestamp.append(timestamp_time)
                    
    # Stop the recording for this range of values
    script_send_command("\t<!-- Stop recording. -->\n", False)
    script_send_command(f'\t<camera name="{camera_name}">\n', False)
    script_send_command(f'\t\t<genicam><enumeration feature="TriggerMode">Off</enumeration></genicam>\n', True) # turn off trigger mode
    script_send_command("\t\t<record>OFF</record>\n", True)
    script_send_command("\t</camera>\n", False)

    # metadata record off
    #script_send_command("\t<microscopeone>\n", False)
    #script_send_command("\t\t<record>OFF</record>\n", True)
    #script_send_command("\t</microscopeone>\n\n", False)
	



################################### get positions from temika ###################################

script_send_command(b"<temika>", False) #n.b. keep temika open at all times otherwise it cannot close


## If loading positions data, comment out from here
'''
print("p -- Save position\nu -- undo last capillary\nf -- Finish capillary\n")
i = 0 # counter for how many capillaries/wells I am looking at
number = [0] # number of positions in a given capillary/well
positions = [] # stores all positions as a list, every 4 numbers are a position set
while i < samples:
    key = input() # wait for the user to press a key

    # save the current position
    if key == 'p':
        vec = script_get_position() # obtains current position of the microscope
        number[i] += 1 # for a capillary/well, how many positions are being chosen
        positions.append(vec) # saves the positions
        print("capillary/well", i, vec) # prints what has been chosen

    #undo last saved position
    elif key == 'u':
        if number[i] > 0:
            number[i] -= 1 
            removed_vec=positions.pop() #removes last vector which was saved
            print("Last position removed from capillary/well",i,removed_vec)

    #finalise positions for the current well
    elif key == 'f':
        number.append(0)
        i += 1
        print("p -- Save position\nf -- Finish capillary\n")


with open(f"{main_folder}/position_data.pkl", "wb") as f:
    pickle.dump((number, positions), f)
print("Positions saved")
'''

#### Use this to load all positions data after an error. Make sure to comment out the bits above
with open(f"{main_folder}/position_data.pkl", "rb") as f:
    number, positions = pickle.load(f)
    print("Loaded data:", number, positions)



#print all the capillaries/wells, their positions and the vectors
i = 0
index = 0
while i < samples:
    print("capillary/well ", i)
    for j in range(number[i]):
        print("    position ", j, positions[j + index])
    index += number[i]
    i += 1




######################### check if everything is correct before proceeding

print(f'Check all settings')

print(f'z stack = {z_stack}')
print(f'z range = {z_range}')
print(f'z step = {z_step}')

print(f'Peltier = {peltier}')
print(f'Enclosure = {enclosure_heating}')

print(f'timelapse = {timelapse}')
print(f'time interval (mins) = {interval}')
print(f'Duration per interval (mins) = {interval_time}')
print(f'Duration per interval (hrs) = {interval_time/60}')

total_time=sum(interval_time)

print(f'Total time (mins) = {total_time}')


print("y - continue, n - Terminate")

key = input() # wait for the user to press a key

if key == 'y':
    print("Continuing...")
    # your continuation code here
elif key == 'n':
    print("Terminating.")
    # optionally exit the script
    script_send_command(b"</temika>", False) #n.b. keep temika open at all times otherwise it cannot close
    exit()

################################## Start the script
#script_send_command(b"<temika>", False)
# Set the enclosure temperature and wait for stabilization
script_send_command(b"<!-- Set temperature_enclosure_high and wait heating_time_enclosure -->\n", False)

if enclosure_heating==1:

    script_send_command(b"<microscopeone>", False)

    script_send_command(b'\t\t<pwr number="4">\n', True) # 0: peltier, 1:PWR_0, 2:PWR_1 3: PWR_2, 4:encloser, 5:LED, 6:Fan
    script_send_command(b"\t\t\t<feedback>\n", False)
    script_send_command(f"\t\t\t\t<set>{temperature_enclosure}</set>\n", True)
    script_send_command(b"\t\t\t\t<enable>ON</enable>\n", True)
    script_send_command(b"\t\t\t</feedback>\n", False)
    script_send_command(b"\t\t</pwr>\n", False)
    script_send_command(b'<pwr number="5"><duty_cycle>0</duty_cycle></pwr>', True)
    script_send_command(b"\t</microscopeone>\n", False)



    h, m, s = map(int, heating_time.split(":"))
    heating_time_s = h * 3600 + m * 60 + s

    print(f"Enclosure heating started for {h:02}:{m:02}:{s:02}")

    # Countdown loop
    for remaining in range(heating_time_s, 0, -1):
        mins, secs = divmod(remaining, 60)
        hours, mins = divmod(mins, 60)
        print(f"\rTime remaining: {hours:02}:{mins:02}:{secs:02}", end="")
        time.sleep(1)

    print("\nHeating complete. Imaging Start.")
    print('------------------------------------------------------------------')


if timelapse==1:
    num_iterations = np.round(interval_time / interval).astype(int) #ind the number of iterations for each interval as integer
    num_iterations[0]=num_iterations[0]+1


    current_timestamp=[] # counter for timestamp
    counter=[] # counter for iteration used
    current_timestamp=0
    counter=0

    overall_start_time=time.time()
    
    if peltier==1:


        pelt_idx=0
        for interval_count in range(len(num_iterations)):
                for iteration in range(num_iterations[interval_count]):

                    if interval_count == 0 and iteration==0:
                        current_timestamp=0 # counter for timestamp
                    else:
                        current_timestamp=np.round((current_timestamp+(interval[interval_count]*60))).astype(int)
                    
                    #print(iteration)
                    #print(interval_count)
                    counter+=1
                    index=0

                    ptc=0
                    peltier_temp(pelt_temp_list[ptc], pelt_idx)
                    pelt_idx +=1
                    pelt_wait=pelt_wait_time[ptc]

                    

                    start_time=time.time() # start of when imaging takes place

                    pwc=0

                    if current_timestamp==pelt_wait:
                        ptc += 1
                        pwc += 1
                        peltier_temp(pelt_temp_list[ptc], pelt_idx)
                        pelt_idx +=1
                        pelt_wait=pelt_wait + pelt_wait_schedule[pwc]

                    
                    for i in range(samples):

                        if enclosure_heating==1:

                            script_print_sample(f"sample_{sample_names[i]}_enclosure_temperature{temperature_enclosure}_peltier_temp:{pelt_temp_list[ptc]}_timestamp_{current_timestamp}_s", positions[index:], number[i])# gives the file a header and moves to positions and images sample
                            #print(current_timestamp)
                            index += number[i]
                        else: 
                            script_print_sample(f"sample_{sample_names[i]}_peltier_temp:{pelt_temp_list[ptc]}_timestamp_{current_timestamp}_s", positions[index:], number[i])# gives the file a header and moves to positions and images sample
                            #print(current_timestamp)
                            index += number[i]


                    end_time=time.time() # end of when imaging takes place

                    elapsed_time=(end_time-start_time) / 60 # time taken to image all samples in minutes

                    

                    if iteration<num_iterations[interval_count]-1:

                        interval_cycle=interval[interval_count]-elapsed_time # how long to wait till next imaging cycle
                        
                        # identify how long to sleep for
                        sleep_time = (timedelta(minutes=interval_cycle))

                        # Convert total seconds to hours, minutes, and seconds using map

                        rounded_seconds = round(sleep_time.total_seconds())
                        ht, mt, st = map(int, [
                            rounded_seconds // 3600,             # Hours
                            (rounded_seconds % 3600) // 60,      # Minutes
                            rounded_seconds % 60                # Seconds
                        ])
                        sleep_time_s = ht* 3600 + mt * 60 + st


                        # Countdown loop
                        for remaining in range(sleep_time_s, 0, -1):
                            mins, secs = divmod(remaining, 60)
                            hours, mins = divmod(mins, 60)
                            print(f"\rTime until next cycle: {hours:02}:{mins:02}:{secs:02}", end="")
                            time.sleep(1)

                        print("\nImaging")
                    
                    else:
                        if interval[interval_count]!=interval[-1]:
                            interval_cycle=interval[interval_count+1]-elapsed_time # how long to wait till next imaging cycle
                            
                            # identify how long to sleep for
                            sleep_time = (timedelta(minutes=interval_cycle))

                            # Convert total seconds to hours, minutes, and seconds using map

                            rounded_seconds = round(sleep_time.total_seconds())
                            ht, mt, st = map(int, [
                                rounded_seconds // 3600,             # Hours
                                (rounded_seconds % 3600) // 60,      # Minutes
                                rounded_seconds % 60                # Seconds
                            ])
                            sleep_time_s = ht* 3600 + mt * 60 + st


                            # Countdown loop
                            for remaining in range(sleep_time_s, 0, -1):
                                mins, secs = divmod(remaining, 60)
                                hours, mins = divmod(mins, 60)
                                print(f"\rTime until next cycle: {hours:02}:{mins:02}:{secs:02}", end="")
                                time.sleep(1)

                            print("\nImaging")



                        
                    

                        
    else:
        for interval_count in range(len(num_iterations)):
            for iteration in range(num_iterations[interval_count]):

                if interval_count == 0 and iteration==0:
                    current_timestamp=0 # counter for timestamp
                else:
                    current_timestamp=np.round((current_timestamp+(interval[interval_count]*60))).astype(int)
                
                #print(iteration)
                #print(interval_count)
                counter+=1
                index=0

                start_time=time.time() # start of when imaging takes place

                for i in range(samples):

                    if enclosure_heating==1:

                        script_print_sample(f"sample_{sample_names[i]}_enclosure_temperature{temperature_enclosure}_timestamp_{current_timestamp}_s_position_{index}", positions[index:], number[i])# gives the file a header and moves to positions and images sample
                        #print(current_timestamp)
                        index += number[i]
                    else:
                        script_print_sample(f"sample_{sample_names[i]}_timestamp_{current_timestamp}_s_position_{index}", positions[index:], number[i])# gives the file a header and moves to positions and images sample
                        #print(current_timestamp)
                        index += number[i]


                end_time=time.time() # end of when imaging takes place

                elapsed_time=(end_time-start_time) / 60 # time taken to image all samples in minutes

                if iteration<num_iterations[interval_count]-1:

                    interval_cycle=interval[interval_count]-elapsed_time # how long to wait till next imaging cycle
                    
                    # identify how long to sleep for
                    sleep_time = (timedelta(minutes=interval_cycle))

                    # Convert total seconds to hours, minutes, and seconds using map

                    rounded_seconds = round(sleep_time.total_seconds())
                    ht, mt, st = map(int, [
                        rounded_seconds // 3600,             # Hours
                        (rounded_seconds % 3600) // 60,      # Minutes
                        rounded_seconds % 60                # Seconds
                    ])
                    sleep_time_s = ht* 3600 + mt * 60 + st


                    # Countdown loop
                    for remaining in range(sleep_time_s, 0, -1):
                        mins, secs = divmod(remaining, 60)
                        hours, mins = divmod(mins, 60)
                        print(f"\rTime until next cycle: {hours:02}:{mins:02}:{secs:02}", end="")
                        time.sleep(1)

                    print("\nImaging")
                else:
                        if interval[interval_count]!=interval[-1]:
                            interval_cycle=interval[interval_count+1]-elapsed_time # how long to wait till next imaging cycle
                            
                            # identify how long to sleep for
                            sleep_time = (timedelta(minutes=interval_cycle))

                            # Convert total seconds to hours, minutes, and seconds using map

                            rounded_seconds = round(sleep_time.total_seconds())
                            ht, mt, st = map(int, [
                                rounded_seconds // 3600,             # Hours
                                (rounded_seconds % 3600) // 60,      # Minutes
                                rounded_seconds % 60                # Seconds
                            ])
                            sleep_time_s = ht* 3600 + mt * 60 + st


                            # Countdown loop
                            for remaining in range(sleep_time_s, 0, -1):
                                mins, secs = divmod(remaining, 60)
                                hours, mins = divmod(mins, 60)
                                print(f"\rTime until next cycle: {hours:02}:{mins:02}:{secs:02}", end="")
                                time.sleep(1)

                            print("\nImaging")





else: 
    if peltier ==1:
        peltier_temp(pelt_temp_list[0],0) #  only looks at 1st temperature in list as ramp is unnecessary
        index=0
        for i in range(samples):

            if enclosure_heating==1:

                script_print_sample(f"sample_{sample_names[i]}_enclosure_temperature{temperature_enclosure}_peltier_temp_{pelt_temp_list[0]}", positions[index:], number[i]) # gives the file a header and moves to positions and images sample
                index += number[i]
            else:
                script_print_sample(f"sample_{sample_names[i]}_peltier_temp_{pelt_temp_list[0]}", positions[index:], number[i]) # gives the file a header and moves to positions and images sample
                index += number[i]

    else:
        index=0
        for i in range(samples):

            if enclosure_heating==1:

                script_print_sample(f"sample_{sample_names[i]}_enclosure_temperature{temperature_enclosure}", positions[index:], number[i]) # gives the file a header and moves to positions and images sample
                index += number[i]
            else:
                script_print_sample(f"sample_{sample_names[i]}", positions[index:], number[i]) # gives the file a header and moves to positions and images sample
                index += number[i]
                

script_print_move_z_c_reset()


script_send_command(b"</temika>", False) # ends the temika script



############################################ Post Image processing #####################################################


### Create dictionary to find what each frame in each movie corresponds to
for sample_idx in range(len(sample_names)):
    # Build frame mapping
    frame_sequence = []
    frame_seq_id = 1


    if z_c_order == 'zc':

        
            for num in range(1, number[sample_idx - 1] + 1):
                for zi in range (-z_range, z_range+1,z_step):
                    for il in range(len(illumination)):
                        frame_sequence.append({
                            "frame": frame_seq_id,
                            "sample": sample_names[sample_idx],
                            "number": num,
                            "z": zi,
                            "illum_wavelength": illumination[il],
                            "illum_exposure_time": illum_expose[il],
                            "illum_pwr": laser_pwr[il]
                        })
                        frame_seq_id += 1

    elif z_c_order == 'cz':


            for num in range(1, number[sample_idx - 1] + 1):
                for il in range(len(illumination)):
                    for zi in range (-z_range, z_range+1,z_step):                
                        frame_sequence.append({
                            "frame": frame_seq_id,
                            "sample": sample_names[sample_idx],
                            "number": num,
                            "z": zi,
                            "illum_wavelength": illumination[il],
                            "illum_exposure_time": illum_expose[il],
                            "illum_pwr": laser_pwr[il]
                        })

                        frame_seq_id += 1

    csv_file = f"{main_folder}/frame_mapping_{sample_idx}.csv"
    with open(csv_file, mode="w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=["frame", "sample", "number", "z", "illum_wavelength", "illum_exposure_time", "illum_pwr"])
        writer.writeheader()
        for f in frame_sequence:
            writer.writerow(f)

    print(f"Frame mapping export complete")

with open(f"{main_folder}/timestamp.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    for item in timestamp:
        writer.writerow([item])  # each item on a new row

print(f"timestamp export complete")

