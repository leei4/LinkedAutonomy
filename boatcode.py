import math

#creates values.
#please change name and eventually remove function
def init_val():
    
    heading = "north"
    lat = 123.22267
    long = -69.65777
    return heading, lat, long

def distance_to_target(start, stop):
    return float( math.sqrt(( (stop[1] - start[1]) ** 2 + (stop[0] - start[0]) ** 2)) ) 

def turn_time():
    #calculates based upon input factors
    #distance, wind direction, speed, other boats, current
    #develop formula
    
    dist = 0

if __name__ == "__main__":
    
    wind_heading = 0
    boat_heading = 0
    relative_heading = 0
    current_distance_to_target = 0
    
    print("booting module 1")
    
    h, lat, long = init_val()
    print(h,lat, long)
    
    #intialize the current_distance to target
    #can be updated in real time via listener objects
    start = (2,4)
    end = (3,7)
    dist = distance_to_target(start, end)
    print(dist)
    print("initial distance set")