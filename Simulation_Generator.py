
#Importing all the libraries
import os
import xml.etree.ElementTree as ET
from pprint import pprint
import sys
import optparse
from  warnings import filterwarnings
filterwarnings("ignore")

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
    
    
from traci import start,simulationStep
from sumolib import checkBinary




def generate_manhattan_grid(net_file = "net.net.xml",x=5,y=3,lanes=3,length=200):
    ''' Generates a Manhattan Grid as per specified requirement
		Manhattan Grid is a simple grid with 90 degree intersections
		
		: net_file	: Location of output network file
		: x			: Number of rows in the grid (default =5)
		: y			: Number of columns in the grid(default = 3)
		: lanes		: Number of lanes in road network(default=3)
		: length	: Length between intersections
	'''
    os.system("netgenerate --grid --grid.x-number="+str(x)+" --grid.y-number="+str(y)+ " -L="+str(lanes)+" --grid.length="+str(length) +" --output-file="+str(net_file)+" -H -j traffic_light")
    
    return net_file
    
    


def generate_cars(route_file,Net_file='net.net.xml',trips_file = "trips.trips.xml",end_time=1000):
    '''   
		Randomly generates cars and their routes for the input network file
		
		route_file 	: Location of output route file
		net_file	: Location of input network file
		trips_file	: Location of intermediate trips file
		end_time	: End time for generating new routes
	'''

    trips_file = trips_file
    # print("randomTrips.py -n "+ str(Net_file) +" -e "+ str(end_time)+" -o "+str(trips_file)+ " --period 0.4")
    os.system("randomTrips.py -n "+ str(Net_file) +" -e "+ str(end_time)+" -o "+str(trips_file)+ " --period 0.4 ")
    os.system("duarouter -n "+str(Net_file)+" --route-files "+str(trips_file) + " -o " +str(route_file) +" --ignore-errors -e " +str(end_time/2))
    return route_file 


def generate_SUMO(config_file, net_file='net.net.xml',route_file = 'route.rou.xml'):
    ''' 
    Generate the SUMO Config file. This file can be used by SUMO to build the final simulation.
	
    Input: Route File, Network File
    Output: Sumo Configuration File
	
    
    '''
    

    #Creating XML    
    configuration = ET.Element('configuration')
    input = ET.SubElement(configuration, 'input')
    N_file = ET.SubElement(input, 'net-file')
    R_files = ET.SubElement(input, 'route-files')
    N_file.set('value',net_file)
    R_files.set('value',route_file)
    time = ET.SubElement(configuration, 'time')
    begin = ET.SubElement(time, 'begin')
    end = ET.SubElement(time, 'end')
    begin.set('value','0')
    end.set('value',str(end_time))
    time_to_teleport =ET.SubElement(configuration, 'time-to-teleport')
    time_to_teleport.set('value','-1')
    config_xml = ET.tostring(configuration)
    
    #Testing
    pprint(ET.tostring(configuration))
    
	#Writing the config file
	with open(config_file, "wb") as f:
        f.write(config_xml)
        
    return config_file


def get_options():
    ''' Collects the options to launch SUMO'''
	
	optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


def run_SUMO(config_file):
	'''
	Launches a SUMO simulator to visualize the mobility simulation
	'''
    
    if __name__ == "__main__":
        options = get_options()
    
        # this script has been called from the command line. It will start sumo as a
        # server, then connect and run
        if options.nogui:
            sumoBinary = checkBinary('sumo')
        else:
            sumoBinary = checkBinary('sumo-gui')
            
    print("Launching SUMO")
    start([sumoBinary, "-c", config_file,'--start'])
    for step in range(end_time+100):
        simulationStep()
    

	
#Main

#Define the parameters and generate a simulation
end_time = 1000
net_file = generate_manhattan_grid('Manhattan5x3.net.xml')
route_file = generate_cars('Manhattan5x3.rou.xml',net_file)
config_file = generate_SUMO('Manhattan5x3.sumocfg',net_file,route_file)
run_SUMO(config_file)





