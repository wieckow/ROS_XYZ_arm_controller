#====================
#Data byte steps are 0x01:0x0f - number of data byte which didn't come in time
#====================
#############################################################

class cmd_description:
    def __init__(self, name, description = [], argc = [], arglen = [], signed = []):
        self.name = name
        if not isinstance(description, list):
            description = [description]
        self.description = description

        if(len(description) > len(argc)):
            argc.extend([1] * (len(description) - len(argc)))
        if(len(description) > len(arglen)):
            arglen.extend([1] * (len(description) - len(arglen)))
        if(len(description) > len(signed)):
            signed.extend([0] * (len(description) - len(signed)))
        
        argc = argc[:len(description)]
        arglen = arglen[:len(description)]
        signed = signed[:len(description)]

        self.argc = []
        self.arglen = []
        self.signed = []

        for elem in argc:
            self.argc.append(elem if elem >= 0 else 0)
        for elem in arglen:
            self.arglen.append(elem if elem > 0 else 1)
        for elem in signed:
            self.signed.append(elem if elem in range(0, 2) else 0)
        self.bytecount = 0

        for i in range(len(argc)):
            self.bytecount += argc[i] * arglen[i]

    def display(self, _args):
        if len(_args) != self.bytecount:
            print "Wrong argument count: ", len(_args), "/", self.bytecount
            return
        print "--------------------"
        cnt = 0
        print self.name
        for i in range(len(self.description)):
            for j in range(self.argc[i]):
                val = 0
                index = ' ' + str(j + 1) if self.argc[i] > 1 else ''
                for _ in range(self.arglen[i]):
                    val *= 256
                    val += ord(_args[cnt])
                    cnt += 1
                if self.signed[i]:
                    val = val if val < (256**self.arglen[i]) / 2 else val - 256**self.arglen[i]
                print "    ", self.description[i], index, ": ", val
            if i != len(self.description) - 1:
                print '    ---'
        print "--------------------"


#Command frames
CMD_ECHO                = 0x00      #Request for ALIVE frame
CMD_SETPOS_ALL          = 0x10      #Set position of every servo (6 independent values)
CMD_SETSPEED_ALL        = 0x11      #Set speed of every servo (6 independent values)
CMD_SETPOS              = 0x12      #Set position of specific servo by ID
CMD_SETSPEED            = 0x13      #Set speed of specific servo by ID
CMD_RELAX_SERVO_ALL     = 0x1B      #Relax all servos
CMD_RELAX_SERVO         = 0x1C      #Relax specific servo by ID
CMD_REBOOT_SERVO_ALL    = 0x1D      #Reboot all servos
CMD_REBOOT_SERVO        = 0x1E      #Reboot specific servo by ID
CMD_SAFE_POS            = 0x1F      #Return arm to safe position
CMD_GET_POS_ALL         = 0x21      #Request position of all servos
CMD_GET_SPEED_ALL       = 0x22      #Request speed of all servos
CMD_GET_POS             = 0x23      #Request position of specific servo by ID
CMD_GET_SPEED           = 0x24      #Request speed of specific servo by ID
CMD_GET_SERVO_PWM_ALL   = 0x32      #Request PWM values of all servos
CMD_GET_SERVO_PWM       = 0x33      #Request PWM value of specific servo by ID
CMD_SET_SERVO_PWM_ALL   = 0x41      #Set PWM values of all servos (6 independent values)
CMD_SET_SERVO_PWM       = 0x42      #Set PWM value of specific servo by ID 
CMD_AUTO_ON             = 0x51      #Enable automatic position regulation (steering by setpoint, not setpos)
CMD_AUTO_OFF            = 0x52      #Disable automatic position regulation (steering by setpoint, not setpos)
CMD_AUTO_SETPOINT       = 0x53      #Setpoint for automatic regulation
CMD_AUTO_MONITOR        = 0x54      #Enable automatic position reading (without get_pos_all)
CMD_AUTO_SETSPEED       = 0x55      #Set speed of servos in automatic regulation, value 1-255
#Array for checking command validity
COMMAND_LIST = [
    CMD_ECHO,    
    CMD_SETPOS_ALL,          
    CMD_SETSPEED_ALL,        
    CMD_SETPOS,              
    CMD_SETSPEED,            
    CMD_REBOOT_SERVO_ALL,    
    CMD_REBOOT_SERVO,        
    CMD_SAFE_POS,            
    CMD_GET_POS_ALL,         
    CMD_GET_SPEED_ALL,       
    CMD_GET_POS,             
    CMD_GET_SPEED,      
    CMD_GET_SERVO_PWM_ALL,   
    CMD_GET_SERVO_PWM,       
    CMD_SET_SERVO_PWM_ALL,   
    CMD_SET_SERVO_PWM,
    CMD_AUTO_ON,
    CMD_AUTO_OFF,
    CMD_AUTO_SETPOINT,
    CMD_AUTO_MONITOR,
    CMD_AUTO_SETSPEED
]

#Command frames length
LEN_ECHO                = 0         #--
LEN_SETPOS_ALL          = 18        #2 bytes for every servo, ID 0 to 5, 1 byte for every playtime (increments of 50 ms)
LEN_SETSPEED_ALL        = 6         #1 byte for every servo, ID 0 to 5
LEN_SETPOS              = 4         #Servo ID, 2 bytes for position, 1 byte for playtime (increments of 50 ms)
LEN_SETSPEED            = 2         #Servo ID, 1 byte for speed
LEN_RELAX_SERVO         = 1         #Servo ID
LEN_RELAX_SERVO_ALL     = 0         #--
LEN_REBOOT_SERVO_ALL    = 0         #--
LEN_REBOOT_SERVO        = 1         #Servo ID
LEN_SAFE_POS            = 0         #--
LEN_GET_POS_ALL         = 0         #--
LEN_GET_SPEED_ALL       = 0         #--
LEN_GET_POS             = 1         #Servo ID
LEN_GET_SPEED           = 1         #Servo ID
LEN_GET_SERVO_PWM_ALL   = 0         #--
LEN_GET_SERVO_PWM       = 1         #Servo ID
LEN_SET_SERVO_PWM_ALL   = 12        #2 bytes for every servo, ID 0 to 5
LEN_SET_SERVO_PWM       = 3         #Servo ID, 2 bytes for PWM
LEN_AUTO_ON             = 0         #--
LEN_AUTO_OFF            = 0         #--
LEN_AUTO_SETPOINT       = 12        #2 bytes for every servo, ID 0 to 5
LEN_AUTO_MONITOR        = 0         #--
LEN_AUTO_SETSPEED       = 1         #1 byte, value 1-255



#############################################################
#Return frames
RET_ALIVE               = 0x00      #Signal that the device responds
RET_POS_ALL             = 0x10      #Positions of all servos
RET_POS                 = 0x11      #Position of single servo
RET_SPEED_ALL           = 0x12      #Speed of all servos
RET_SPEED               = 0x13      #Speed of single servo
RET_SERVO_PWM_ALL       = 0x22      #PWM values of all servos
RET_SERVO_PWM           = 0x23      #PWM value of single servo

#Array for checking return frame types validity
RETURN_FRAMES = [
    RET_ALIVE,               
    RET_POS_ALL,             
    RET_POS,                 
    RET_SPEED_ALL,           
    RET_SPEED,         
    RET_SERVO_PWM_ALL,       
    RET_SERVO_PWM           
]

#Return frames length
LEN_RET_ALIVE           = 0         #--
LEN_RET_POS_ALL         = 12        #2 bytes for every servo, ID 0 to 5    
LEN_RET_POS             = 3         #Servo ID, 2 bytes for position
LEN_RET_SPEED_ALL       = 6         #1 byte for every servo, ID 0 to 5
LEN_RET_SPEED           = 2         #Servo ID, 1 byte for speed
LEN_RET_SERVO_PWM_ALL   = 12        #2 bytes for every servo, ID 0 to 5
LEN_RET_SERVO_PWM       = 3         #Servo ID, 2 bytes for PWM value

#############################################################
#Dictionary for length lookup
cmdframelength = {
CMD_ECHO                : LEN_ECHO,
CMD_SETPOS_ALL          : LEN_SETPOS_ALL,
CMD_SETSPEED_ALL        : LEN_SETSPEED_ALL,
CMD_SETPOS              : LEN_SETPOS,
CMD_SETSPEED            : LEN_SETSPEED,
CMD_RELAX_SERVO_ALL     : LEN_RELAX_SERVO_ALL,
CMD_RELAX_SERVO         : LEN_RELAX_SERVO,
CMD_REBOOT_SERVO_ALL    : LEN_REBOOT_SERVO_ALL,
CMD_REBOOT_SERVO        : LEN_REBOOT_SERVO,
CMD_SAFE_POS            : LEN_SAFE_POS,
CMD_GET_POS_ALL         : LEN_GET_POS_ALL,
CMD_GET_SPEED_ALL       : LEN_GET_SPEED_ALL,
CMD_GET_POS             : LEN_GET_POS,
CMD_GET_SPEED           : LEN_GET_SPEED,
CMD_GET_SERVO_PWM_ALL   : LEN_GET_SERVO_PWM_ALL,
CMD_GET_SERVO_PWM       : LEN_GET_SERVO_PWM,
CMD_SET_SERVO_PWM_ALL   : LEN_SET_SERVO_PWM_ALL,
CMD_SET_SERVO_PWM       : LEN_SET_SERVO_PWM,
CMD_AUTO_ON             : LEN_AUTO_ON,
CMD_AUTO_OFF            : LEN_AUTO_OFF,
CMD_AUTO_SETPOINT       : LEN_AUTO_SETPOINT,
CMD_AUTO_MONITOR        : LEN_AUTO_MONITOR,
CMD_AUTO_SETSPEED       : LEN_AUTO_SETSPEED 
}
cmdframename = {
CMD_ECHO                : 'CMD_ECHO',
CMD_SETPOS_ALL          : 'CMD_SETPOS_ALL',
CMD_SETSPEED_ALL        : 'CMD_SETSPEED_ALL',
CMD_SETPOS              : 'CMD_SETPOS',
CMD_SETSPEED            : 'CMD_SETSPEED',
CMD_RELAX_SERVO_ALL     : 'CMD_RELAX_SERVO_ALL',
CMD_RELAX_SERVO         : 'CMD_RELAX_SERVO',
CMD_REBOOT_SERVO_ALL    : 'CMD_REBOOT_SERVO_ALL',
CMD_REBOOT_SERVO        : 'CMD_REBOOT_SERVO',
CMD_SAFE_POS            : 'CMD_SAFE_POS',
CMD_GET_POS_ALL         : 'CMD_GET_POS_ALL',
CMD_GET_SPEED_ALL       : 'CMD_GET_SPEED_ALL',
CMD_GET_POS             : 'CMD_GET_POS',
CMD_GET_SPEED           : 'CMD_GET_SPEED',
CMD_GET_SERVO_PWM_ALL   : 'CMD_GET_SERVO_PWM_ALL',
CMD_GET_SERVO_PWM       : 'CMD_GET_SERVO_PWM',
CMD_SET_SERVO_PWM_ALL   : 'CMD_SET_SERVO_PWM_ALL',
CMD_SET_SERVO_PWM       : 'CMD_SET_SERVO_PWM',
CMD_AUTO_ON             : 'CMD_AUTO_ON',
CMD_AUTO_OFF            : 'CMD_AUTO_OFF',
CMD_AUTO_SETPOINT       : 'CMD_AUTO_SETPOINT',
CMD_AUTO_MONITOR        : 'CMD_AUTO_MONITOR',
CMD_AUTO_SETSPEED       : 'CMD_AUTO_SETSPEED' 
}
retframelength = {
RET_ALIVE               : LEN_RET_ALIVE,
RET_POS_ALL             : LEN_RET_POS_ALL,
RET_POS                 : LEN_RET_POS,
RET_SPEED_ALL           : LEN_RET_SPEED_ALL,
RET_SPEED               : LEN_RET_SPEED,
RET_SERVO_PWM_ALL       : LEN_RET_SERVO_PWM_ALL,
RET_SERVO_PWM           : LEN_RET_SERVO_PWM
}
retframename = {
RET_ALIVE               : 'RET_ALIVE',
RET_POS_ALL             : 'RET_POS_ALL',
RET_POS                 : 'RET_POS',
RET_SPEED_ALL           : 'RET_SPEED_ALL',
RET_SPEED               : 'RET_SPEED',
RET_SERVO_PWM_ALL       : 'RET_SERVO_PWM_ALL',
RET_SERVO_PWM           : 'RET_SERVO_PWM'
}

vis_cmd = {                                   #Description                              #Argument description          #Count      #Length     #Signed
    CMD_ECHO                : cmd_description('Echo request',                           []                             ,[]         ,[]         ,[]         ),
    CMD_SETPOS_ALL          : cmd_description('Set all servo positions',                ['Position','Time']            ,[6, 6]     ,[2, 1]     ,[0, 0]     ),
    CMD_SETSPEED_ALL        : cmd_description('Set all servo speeds',                   ['Speed','Time']               ,[6, 6]     ,[1, 1]     ,[1, 0]     ),
    CMD_SETPOS              : cmd_description('Set servo position',                     ['Servo ID','Position','Time'] ,[1, 1, 1]  ,[1, 2, 1]  ,[0, 0, 0]  ),
    CMD_SETSPEED            : cmd_description('Set servo speed',                        ['Servo ID','Speed','Time']    ,[1, 1, 1]  ,[1, 1, 1]  ,[0, 1, 0]  ),
    CMD_RELAX_SERVO_ALL     : cmd_description('Relax all servos',                       []                             ,[]         ,[]         ,[]         ),
    CMD_RELAX_SERVO         : cmd_description('Relax servo',                            ['Servo ID']                   ,[1]        ,[1]        ,[0]        ),
    CMD_REBOOT_SERVO_ALL    : cmd_description('Reboot all servos',                      []                             ,[]         ,[1]        ,[]         ),
    CMD_REBOOT_SERVO        : cmd_description('Reboot servo',                           ['Servo ID']                   ,[1]        ,[1]        ,[0]        ),
    CMD_SAFE_POS            : cmd_description('Safe position',                          []                             ,[]         ,[]         ,[]         ),
    CMD_GET_POS_ALL         : cmd_description('Request all servo positions',            []                             ,[]         ,[]         ,[]         ),
    CMD_GET_SPEED_ALL       : cmd_description('Request all servo speeds',               []                             ,[]         ,[]         ,[]         ),
    CMD_GET_POS             : cmd_description('Request servo position',                 ['Servo ID']                   ,[1]        ,[1]        ,[0]        ),
    CMD_GET_SPEED           : cmd_description('Request servo speed',                    ['Servo ID']                   ,[1]        ,[1]        ,[0]        ),
    CMD_GET_SERVO_PWM_ALL   : cmd_description('Request all servo max PWMs',             []                             ,[]         ,[]         ,[]         ),
    CMD_GET_SERVO_PWM       : cmd_description('Request servo max PWM',                  ['Servo ID']                   ,[1]        ,[1]        ,[0]        ),
    CMD_SET_SERVO_PWM_ALL   : cmd_description('Set all servo PWMs',                     ['Max PWM']                    ,[6]        ,[2]        ,[0]        ),
    CMD_SET_SERVO_PWM       : cmd_description('Set servo PWM',                          ['Servo ID', 'Max PWM' ]       ,[1, 1  ]   ,[1, 2]     ,[0, 0]     ),
    CMD_AUTO_ON             : cmd_description('Enable autoregulation',                  ['Servo ID']                   ,[1]        ,[1]        ,[0]        ),
    CMD_AUTO_OFF            : cmd_description('Disable autoregulation',                 []                             ,[]         ,[]         ,[]         ),
    CMD_AUTO_SETPOINT       : cmd_description('Set autoregulation setpoint',            ['Setpoint']                   ,[6]        ,[2]        ,[0]        ),
    CMD_AUTO_MONITOR        : cmd_description('Toggle automatic position monitoring',   []                             ,[]         ,[]         ,[]         ),
    CMD_AUTO_SETSPEED       : cmd_description('Set servo PWM',                          ['Speed']                      ,[1]        ,[1]        ,[0]        )
}
vis_ret = {
    RET_ALIVE               : cmd_description('Alive signal',                           ['Servo ID']                   ,[1]        ,[1]        ,[0]        ),
    RET_POS_ALL             : cmd_description('All servo positions',                    ['Position']                   ,[6]        ,[2]        ,[0]        ),
    RET_SPEED_ALL           : cmd_description('All servo speeds',                       ['Speed']                      ,[6]        ,[1]        ,[1]        ),
    RET_POS                 : cmd_description('Servo position',                         ['Servo ID','Position']        ,[1, 1]     ,[1, 1]     ,[0, 0]     ),
    RET_SPEED               : cmd_description('Servo speed',                            ['Servo ID','Speed']           ,[1, 1]     ,[1, 1]     ,[0, 1]     ),
    RET_SERVO_PWM_ALL       : cmd_description('All servo PWMs',                         ['Max PWM']                    ,[6]        ,[2]        ,[0]        ),
    RET_SERVO_PWM           : cmd_description('Servo PWM',                              ['Servo ID', 'Max PWM']        ,[1, 1]     ,[1, 1]     ,[0, 0]     ),
   }
