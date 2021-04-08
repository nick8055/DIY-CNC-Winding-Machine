from pickle import dump,load
import time
import sys
import serial
import os
import subprocess
import RPi.GPIO as GPIO
import threading

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

wg = serial.Serial("/dev/ttyUSB1", 9600)
sp = serial.Serial("/dev/ttyUSB0", 9600)

job_comp = 0
direction = 0
if (os.stat("jc.pickle").st_size > 0):
 jc = open("jc.pickle", "rb")
 job_comp = load(jc)
 jc.close()

pause_ip = 12
fg_reset = 25
sp_ena = 19
wg_ena = 6
wg_speed = 0
spndl_enc_ip = 5
wg_enc_ip = 13
GPIO.setup(pause_ip,GPIO.IN)
GPIO.setup(sp_ena,GPIO.OUT)
GPIO.setup(wg_ena,GPIO.OUT)
GPIO.setup(spndl_enc_ip, GPIO.IN)
GPIO.setup(wg_enc_ip,GPIO.IN)
GPIO.setup(fg_reset,GPIO.OUT)
GPIO.setup(26,GPIO.OUT)
GPIO.output(fg_reset,0)
x = 0.1
t = 1
sec = 0
rev = 0
rev1 = 0
pulse_sp = 0
pulse_sp2 = 0
pulse_wg = 0
pulse_wg2 = 0
pulse_overshoot = 0
w = 1
wg_dir_state = 1   #For direction toggling of wire guide(Left -> 1, Right -> 0)
no_of_layers = 0
minu = 0
pckl = 0

shape = ''
shape_num = 1
start_end_width = 0.0

print("\n\tWIND-BY-WIRE CNC COIL WINDING MACHINE\n")
time.sleep(1)

###########INTERUPTS#####################################################
def isr_sp(r): #Interrupt service routine function for SPINDLE pulse counter
    global pulse_sp, pulse_sp2, rev
    rev += 1
    if(GPIO.input(pause_ip) == 0):
     pulse_sp += 0.125
     pulse_sp2 += 0.125

def isr_wg(r): #Interrupt service routine function for WG pulse counter
    global pulse_wg
    global pulse_wg2
    global w
    global wg_dir_state,rev1
    rev1 += 1
    if(w == wg_dir_state):
     pulse_wg += 0.125
     pulse_wg2 += 0.125
    else:
     pulse_wg -= 0.125
     pulse_wg2 -= 0.125
     
GPIO.add_event_detect(wg_enc_ip, GPIO.BOTH, callback=isr_wg)
GPIO.add_event_detect(spndl_enc_ip, GPIO.BOTH, callback=isr_sp)
###############################################################################################
   
#GPIO.add_event_detect(spndl_enc_ip, GPIO.BOTH, callback=troubleshoot)      
wg.write(str.encode('F8.22'))
sp.write(str.encode('F600'))
GPIO.output(sp_ena,0)
GPIO.output(wg_ena,0)
print("\nInitiate troubleshooting for USB ports of PWM generators?\n")
print("Press 'y' or 'Y' to initiate")
print("Press 'n' or 'N' to skip")
print("Press 'c' or 'C' to cancel/exit program\n")
e1 = input(">>")
if(e1 == 'c' or e1 == 'C'):
        sys.exit()
if(e1 == 'y' or e1 == 'Y'):
     GPIO.output(fg_reset,1)
     time.sleep(0.5)
     GPIO.output(fg_reset,0)
     time.sleep(0.5)
     GPIO.output(26,1)
     while(1):
        time.sleep(x)
        print("\nChecking for errors in USB addresses of Frequency Generators\n")
        sec += x
        wg.write(str.encode('F8.22'))
        GPIO.output(wg_ena,1)
        if(sec > 2 and rev1 <5 ):
            GPIO.output(wg_ena,0)
            print("\nError in USB addresses detected\n")
            t = sp
            sp = wg
            wg = t
            time.sleep(1)
            sec = 0
        if(sec > 2 and rev1>=5):
            GPIO.output(wg_ena,0)
            if(t == 1):
                print("\nNo errors found. USB addresses are static\n")
                GPIO.output(fg_reset,0)
                time.sleep(0.5)
                GPIO.output(fg_reset,1)
                time.sleep(0.5)
                GPIO.output(26,0)
                GPIO.output(wg_ena,1)
                wg.write(str.encode('F8.22'))
                time.sleep(2)
                GPIO.output(wg_ena,0)
                wg.write(str.encode('F500'))
                time.sleep(0.5)
            if(t != 1):
             print("Automatic Swapping of USB addresses for Freq.Generators has been done\n")      
             GPIO.output(fg_reset,1)
             time.sleep(0.5)
             GPIO.output(fg_reset,0)
             time.sleep(0.5)
             GPIO.output(26,0)
             GPIO.output(wg_ena,1)
             wg.write(str.encode('F8.22'))
             time.sleep(2)
             GPIO.output(wg_ena,0)
             time.sleep(1)
             GPIO.output(wg_ena,1)
             wg.write(str.encode('F600'))
             time.sleep(2)
             GPIO.output(wg_ena,0)
             wg.write(str.encode('F500'))
             time.sleep(0.5)
            wg.write(str.encode('F500'))
            sp.write(str.encode('F500'))
            break
##############END OF TOUBLESHOOTING#####################################################
 
############ GPIO PIN NUMBER ASSIGNMENT #######################
spndl_enc_ip = 5
wg_enc_ip = 13
wg_ena = 6
sp_ena = 19
wg_dir = 26
pause_ip = 12
dir_reset = 22
stop_button = 4
wg_manual_dir = 23
led_left = 2
led_right = 3
temp_pin = 17
fg_reset = 25 #Reset Pin for freq.generators

#Pins for rotary encoder:-
input_CLK = 20
input_SW = 16
input_DT = 21
counter_sp = 0
counter_wg = 0
currentStateCLK = 0

####GPIO_SETUP########################
GPIO.setup(temp_pin, GPIO.OUT)
GPIO.setup(dir_reset, GPIO.IN)   #Input from horizontal sensor
GPIO.setup(wg_ena,GPIO.OUT)
GPIO.setup(sp_ena,GPIO.OUT)
GPIO.setup(wg_dir,GPIO.OUT)
GPIO.setup(wg_enc_ip,GPIO.IN)
GPIO.setup(spndl_enc_ip, GPIO.IN)
GPIO.setup(pause_ip,GPIO.IN)
GPIO.setup(stop_button, GPIO.IN)
GPIO.setup(wg_manual_dir,GPIO.IN)
GPIO.setup(led_left,GPIO.OUT)
GPIO.setup(led_right,GPIO.OUT)
GPIO.setup(fg_reset,GPIO.OUT)
####Rotary-Encoder############
GPIO.setup(input_CLK,GPIO.IN)
GPIO.setup(input_DT,GPIO.IN)
GPIO.setup(input_SW,GPIO.IN)

left_overshoot = 0
right_overshoot = 0
###############################################################################################
while(1):
  ######Pickling parameters############
    pulse_sp = 0
    pulse_sp2 = 0
    pulse_wg = 0
    pulse_wg2 = 0
    w = 1
    wg_dir_state = 1   #For direction toggling of wire guide(Left -> 1, Right -> 0)
    no_of_layers = 0
    sec = 0
    minu = 0
    pckl = 0
    if(job_comp == 0):
     print("Do you want to restore previous process state or not?\n")
     print("'y' or 'Y' for YES")
     print("'n' or 'N' for NO")
     print("'c' or 'C' to CANCEL/exit Program")
     pckl = input(">>")
    if(pckl == 'y' or pckl == 'Y'):
        fr = open("p.pickle", "rb")
        info = load(fr)
        pulse_sp = float(info[0])
        pulse_wg = info[1]
        pulse_wg2 = info[2]
        w = info[3]
        wg_dir_state = info[4]   #For direction toggling of wire guide(Left -> 1, Right -> 0)
        no_of_layers = info[5]
        sec = info[6]
        minu = info[7]
        pulse_sp2 = info[8]
        fr.close()
    else:
        pulse_sp=0
        pulse_sp2 = 0
        pulse_wg=0
        pulse_wg2=0
        w=1
        wg_dir_state=1
        no_of_layers=0
        sec=0
        minu=0
    if(pckl == 'c' or pckl == 'C'):
        sys.exit()
##########################################################################    
    wire_cat1 = 75 #For wire thickness 0.1 to 0.35 ( Max. RPM = 75)
    wire_cat2 = 60 #For wire thickness 0.35 to 0.55 ( Max. RPM = 60)
    wire_cat3 = 55 #For wire thickness 0.55 to 0.75 ( Max. RPM = 55)
    wire_cat4 = 50 #For wire thickness 0.75 to 1.1 ( Max. RPM = 50)
    wire_cat5 = 40 #For wire thickness 1.1 to 1.25 ( Max. RPM = 40)

    ur = open("user.pickle", "rb")
    uinfo = load(ur)
    wire_thickness = uinfo[0]         #In millimetres.Range is 1.219 to 0.2
    winding_width = uinfo[1]            #In millimetres
    total_turns = uinfo[2]             #Total required turns
    wind_gap_comp_factor = uinfo[3] #Winding Gap Compensation Factor in mm(Only +ve nos.)
    spndl_rpm = uinfo[4];
    shape_num = uinfo[5]
    shape = uinfo[6]
    start_end_width = uinfo[7]
    #job_comp = uinfo[5];
    ur.close()

    print("USER-DEFINED PARAMETERS:-\n")
    print("Wire thickness                  : ", wire_thickness, "mm")
    print("Winding Width                   : ", winding_width, 'mm')
    print("Total no. of turns              : ", total_turns)
    print("Winding gap compensation factor : ", wind_gap_comp_factor, "mm")
    print("Maximum Main Spindle RPM Set at : ", spndl_rpm, " RPM")
    print("Solenoid Cross-section shape    : ", shape)
    print("Width in mm at crawling speed   : ", start_end_width, "mm\n")
    
    print("Edit above Parameters ??\n")
    print("Press 'y' or 'Y' to EDIT.")
    print("Press 'n' or 'N' to SKIP.")
    print("Press 'c' or 'C' to CANCEL/exit program")
    user_ip = input(">>")
    if(user_ip == 'c' or user_ip == 'C'):
        sys.exit()
    if(user_ip == 'y' or user_ip =='Y'):
        while(1):
         print("\nOption no.       PARAMETERS\n")
         print("    1.           Wire thickness(",wire_thickness,"mm",")")
         print("    2.           Winding Width(",winding_width,"mm",")")
         print("    3.           Total No. of turns(",total_turns,")")
         print("    4.           Winding gap comp. factor(",wind_gap_comp_factor,"mm",")")
         print("    5.           Main Spindle RPM(",spndl_rpm,"RPM",")")
         print("    6.           Solenoid Cross-section shape(",shape,")")
         print("    7.           Width in mm at crawling speed(",start_end_width,"mm",")")
         print("    8.           Finished editing\n")
         choice = input("ENTER option no. to edit : ")
         if(choice == '1'):
          while(1):
           # if(job_comp == 0):
           #      print("Wire thickness can't be edited, since it will effect RESTORED  process state\n")
            #     time.sleep(3)
            #     break
           # else:
                 
             wire_thickness = float(input("Wire thickness : "))
             if(wire_thickness <0.1 or wire_thickness >1.25):
              print ("Entered wire thickness out of range! Please stick to permissible thickness range, ie., between 0.1 mm and 1.25 mm")
              time.sleep(2)
              continue
             else:
                 break
             #if(job_comp == 1):    
             print("\nWire thickness changed to ", wire_thickness,"mm\n")
             time.sleep(2)
             if(wire_thickness >= 0.1 and wire_thickness <= 0.35):
              spndl_rpm = wire_cat1
             if(wire_thickness > 0.35 and wire_thickness <= 0.55):
              spndl_rpm = wire_cat2
             if(wire_thickness > 0.55 and wire_thickness <= 0.75):
              spndl_rpm = wire_cat3
             if(wire_thickness > 0.75 and wire_thickness <= 1.1):
              spndl_rpm = wire_cat4
             if(wire_thickness > 1.1 and wire_thickness <= 1.25):
              spndl_rpm = wire_cat5
         elif(choice == '2'):
            # if(job_comp == 1):  
              winding_width = float(input("Winding width in mm : "))
              print("\nWinding Width changed to ", winding_width,"mm\n")
              time.sleep(2)          
            # else:
             # print("Winding Width can't be edited, since it will effect RESTORED process state.\n")
             # time.sleep(3)
         elif(choice == '3'):
            total_turns = int(input("Total no. of turns : "))
            print("\nTotal no. of turns set at ", total_turns, "\n")
            time.sleep(2)        
         elif(choice == '4'):
            wind_gap_comp_factor = float(input("Winding gap compensation factor in mm, if required : "))
            print("\nWinding gap compensation factor set at ", wind_gap_comp_factor, "mm\n")
            time.sleep(2)
         elif(choice == '5'):
             if(wire_thickness >= 0.1 and wire_thickness <= 0.35):
               while(1):  
                wire_cat1 = int(input("Enter RPM : "))
                if(wire_cat1 > 75 and shape_num == 1):
                   print("Entered RPM out of range. Choose less than 75 RPM")
                   time.sleep(2)
                   continue
                else:
                    break
               spndl_rpm = wire_cat1
             if(wire_thickness > 0.35 and wire_thickness <= 0.55):
               while(1):  
                wire_cat2 = int(input("Enter RPM : "))
                if(wire_cat2 > 60 and shape_num == 1):
                   print("Entered RPM out of range. Choose less than 60 RPM")
                   time.sleep(2)
                   continue
                else:
                    break
               spndl_rpm = wire_cat2
             if(wire_thickness > 0.55 and wire_thickness <= 0.75):
               while(1):  
                wire_cat3 = int(input("Enter RPM : "))
                if(wire_cat3 > 55 and shape_num == 1):
                   print("Entered RPM out of range. Choose less than 55 RPM")
                   time.sleep(2)
                   continue
                else:
                    break
               spndl_rpm = wire_cat3
             if(wire_thickness > 0.75 and wire_thickness <= 1.1):
               while(1):  
                wire_cat4 = int(input("Enter RPM : "))
                if(wire_cat4 > 50 and shape_num == 1):
                   print("Entered RPM out of range. Choose less than 50 RPM")
                   time.sleep(2)
                   continue
                else:
                    break
               spndl_rpm = wire_cat4
             if(wire_thickness > 1.1 and wire_thickness <= 1.25):
               while(1):  
                wire_cat5 = int(input("Enter RPM : "))
                if(wire_cat5 > 40 and shape_num == 1):
                   print("Entered RPM out of range. Choose less than 40 RPM")
                   time.sleep(2)
                   continue
                else:
                    break
               spndl_rpm = wire_cat5
         elif(choice == '6'):
          print("1. Quadrilateral")
          print("2. Circular\n")
          shape_num = int(input("Choose shape no. : "))
          if(shape_num == 1):
           shape = 'Quadrilateral'
          else:
           shape = 'Circular'
         elif(choice == '7'):
            start_end_width = float(input("Type width in mm : "))
         elif(choice == '8'):
            print("\nFinalize editing of user-defined parameters?\n")
            print("Press 'y' or 'Y' for YES")
            print("Press 'n' or 'N' for NO")
            fin = input(">>")
            if(fin == 'y' or fin == 'Y'):
             break
            else:
             continue
         else:
            print("\nInvalid option")
        user_list = [wire_thickness,winding_width,total_turns,wind_gap_comp_factor,spndl_rpm, shape_num,shape, start_end_width]
        uw = open("user.pickle","wb")
        dump(user_list,uw)
        uw.close()
       
    print("\nUSER-DEFINED PARAMETERS:-\n")
    print("Wire thickness                  : ", wire_thickness, "mm")
    print("Winding Width                   : ", winding_width, 'mm')
    print("Total no. of turns              : ", total_turns)
    print("Winding gap compensation factor : ", wind_gap_comp_factor, "mm")
    print("Maximum Main Spindle RPM Set at : ", spndl_rpm, " RPM")
    print("Solenoid cross-section shape    : ", shape)
    print("Width in mm at crawling speed   : ", start_end_width, "mm\n")
   
    ################ MACHINE COMPONENT PARAMETERS ##############################
    spr_sp = 3200                 #Stepper Controller Microstepping for spindle
    spr_wg = 3200                 #Stepper Controller Microstepping for wire guide
    decel_window = 1.75
    lower_temperature_threshold = 45
    upper_temperature_threshold = 50
    wg_thread_pitch = 0.62111801242
    spndl_gear_ratio = 2.43478261
    wg_gear_ratio = 2.08333333
    min_freq = 1.01               #Initial Frequency in KHz
    user_set_freq = min_freq
    ###############################################################################
    upper_bound = 70
    lower_bound = 40
    count_at_max = 0
    p = 0
    ################################## COMPUTED PARAMETERS ####################################
    min_freq_str1 = 'F'
    min_freq_str2 = str(float(min_freq))
    min_freq_str1 += min_freq_str2
    effective_wire_thickness = wire_thickness + wind_gap_comp_factor    
    wg_spndl_rpm = effective_wire_thickness * spndl_rpm * wg_thread_pitch                #Derived WG spindle speed in RPM
    upper_lim_freq = ((spndl_rpm * spndl_gear_ratio * spr_sp)/60)/1000
    upl_wg = (((wg_spndl_rpm * wg_gear_ratio * spr_wg)/60)/1000)
    master_ratio = upper_lim_freq/upl_wg   #Spindle_Upper_Lim_frequency/WG_upper_lim_frequency
    master_ratio_dynamic = 0
    turns_per_layer = 0        #Theoretical Value
    actual_turns_per_layer = 0 #Computed value based on winding gap_factor
    wg_pul_dir_change = 0      #WG direction change at pulse count from WG encoder
    f = 0                      #Variable to store RPM of Spindle frequency generator
    a = 0 #This variable stores the last digit of spindle frequency readout
    b = 0 #This variable stores the last digit of wg frequency readout
    resolution = (upper_lim_freq - min_freq) * 100   #Mapping resolution
    count_incrementer_sp = 0
    count_decrementer_sp = 0
    count_incrementer_wg = 0
    count_decrementer_wg = 0
    counter_sp = 0
    counter_wg = 0
    temp = 0
    #############CALIBRATION######################
    initial_freq = 1.01
    final_freq = 19.9

    GPIO.output(sp_ena,0)
    GPIO.output(wg_ena,0)
    GPIO.output(led_left,0)
    GPIO.output(led_right,0)
    previousStateCLK = GPIO.input(input_CLK)
    ##################################
    l = 1
    L = 1
    r = 0
    R = 0
    wg_freq  = 0
    countdown = 0
    #############################################################################
    def mapp(s,a1,a2,b1,b2): ##User-defined map function
        t = b1 + (((s-a1)*(b2-b1))/(a2-a1))
        return t
    ######################## PID parameters ##########################################
    kp = mapp(wire_thickness,0.1,1.25,5,50)
    #kp = 20
    kd = kp * 1.1
    ki = kd * 2.5
    target = 0
    e = 0
    t = 0
    ####################################################################################################################
    x = 0.1
    tp = 0
    ptime = 0
    rev = 0
    t = 0
    sec = 0
    n = 0
    n1 = 2
    ########################## Thread Functions ########################################
    def manual_wg_dir():
        global pulse_wg2, pulse_sp2 
        global no_of_layers
        global wg_dir_state
        global wg_manual_dir
        global w
        global sec
        global minu
        global pulse_sp, total_turns, stop_button, sp_ena, wg_ena
        while(1):
           time.sleep(0.1)
           sec += 0.1
           if(sec >=60):
            minu += 1
            sec = 0
           if(int(pulse_sp) >=total_turns):
             print("Manual")  
             sys.exit()
             break
           if(GPIO.input(pause_ip) == 0):
            if(GPIO.input(stop_button) == 0 or int(pulse_sp) >=total_turns):
                  GPIO.output(sp_ena,0)
                  GPIO.output(wg_ena,0)
                  sys.exit()
                  break  
            if(w != wg_dir_state):
                wg_dir_state = not wg_dir_state
            if(GPIO.input(wg_manual_dir) == 0):  
             wg_dir_state = not wg_dir_state
             w = not w
             pulse_wg2 = 0
             pulse_sp2 = 0
             no_of_layers += 1
             time.sleep(1)
           else:
               if(GPIO.input(wg_manual_dir) == 0):  
                wg_dir_state = not wg_dir_state
                sec += 2
                time.sleep(1)
            
                
               
    def rot_enc(): #Dedicated Thread No.3 for User Rotary Encoder
        global input_CLK
        global input_DT
        global currentStateCLK
        global previousStateCLK
        global counter_sp
        global counter_wg
        global user_set_freq
        global wg_freq
        global total_turns
        global resolution
        global stop_button
        global a
        global b
        global sp
        global count_incrementer_sp
        global count_decrementer_sp
        global count_incrementer_wg
        global count_decrementer_wg
        global master_ratio
        global master_ratio_dynamic
        global pulse_sp
        global min_freq_str1
        global p
        global decel_window
        global wg_ena
        global sp_ena
        global stop_button

        while(1):
            time.sleep(0.01)
            if(p <= decel_window ):
                continue;
            GPIO.output(sp_ena, 1)
            a = user_set_freq % 0.1
            a = a * 100
            b = wg_freq % 0.1
            b = b * 100
            currentStateCLK = GPIO.input(input_CLK)
            if(currentStateCLK != previousStateCLK):
                if(GPIO.input(input_DT) != currentStateCLK):
                    if(a >= 8):
                     count_incrementer_sp = 2
                    if(a<8):
                     count_incrementer_sp = 1
                    counter_sp = counter_sp + count_incrementer_sp
                    if(b >= 8):
                     count_incrementer_wg = 2 #*master_ratio
                    if(b<8):
                     count_incrementer_wg = 1 #*master_ratio
                    counter_wg = counter_wg + count_incrementer_wg
                    if(counter_sp >= resolution):
                        counter_sp = resolution
                        counter_wg = resolution/master_ratio
                    c = mapp(counter_sp,0,resolution,0,100)
                    print ("Rot_Enc_Position :",user_set_freq, "\n")
                else:
                    if(a >= 3):
                     count_decrementer_sp = 1
                    if(a<1):
                     count_decrementer_sp = 2
                    counter_sp = counter_sp - count_decrementer_sp
                    if(b >= 3):
                     count_decrementer_wg = 1 #*master_ratio
                    if(b<1):
                     count_decrementer_wg = 2 #*master_ratio
                    counter_wg = counter_wg - count_decrementer_wg
                    if(counter_sp <= 0):
                        counter_sp = 0
                        counter_wg = 0
                    c = mapp(counter_sp,0,resolution,0,100)
                    print ("Rot_Enc_Position :",user_set_freq, "\n")
            previousStateCLK = currentStateCLK
            if(int(pulse_sp) >=total_turns):
             print("ENC")  
             sys.exit()
             break
            if(master_ratio_dynamic < 0):
             sp.write(str.encode(min_freq_str1))
             GPIO.output(wg_ena,0)
             GPIO.output(sp_ena,0)
             print("Error - Wire Gauge too thin. Range is 1.219 to 0.234","\n")
             break
            if(GPIO.input(pause_ip) == 0):
             if(GPIO.input(stop_button) == 0 or int(pulse_sp) >=total_turns):
                  print("ENC")
                  GPIO.output(sp_ena,0)
                  GPIO.output(wg_ena,0)
                  sys.exit()
                  break

    def wg_enc(): #Dedicated Thread No.1 for reading pulses from  WG Encoder
        global pulse_wg
        global pulse_sp
        global pulse_sp2
        global pulse_wg2
        global sp
        global wg
        global info
        global resolution
        global user_set_freq
        global wg_freq
        global sec
        global minu
        global f
        global b
        global total_turns
        global master_ratio_dynamic
        global master_ratio
        global wg_ena
        global min_freq_str1
        global countdown
        global resolution
        global wg_thread_pitch
        global no_of_layers
        global sp_ena
        global stop_button
        global wg_dir_state
        global led_left
        global led_right
        global winding_width
        global total_turns
        global e
        global temp
        global job_comp
        o1=0
        n1=11
        global lower_temperature_threshold
        global upper_temperature_threshold, pulse_sp2
        status = 1
        global pckl
        if(pckl == 'y' or pckl == 'Y'):
            pulse_sp = info[0]
            pulse_wg = info[1]
            pulse_wg2 = info[2]
            no_of_layers = info[5]
            pulse_sp2 = info[8]
           
        while(1):
            time.sleep(0.3)
            job_comp = 0
            jc = open("jc.pickle", "wb")
            dump(job_comp, jc)
            jc.close()
            fw = open("p.pickle", "wb")
            val = [pulse_sp,pulse_wg,pulse_wg2,w,wg_dir_state,no_of_layers,sec,minu,pulse_sp2]
            dump(val,fw)
            fw.close()
            temp = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True)
            temp = temp.stdout.decode()
            temp = float(temp.split('=')[1].split('\'')[0])
            GPIO.output(sp_ena, 1)
            print ("Current layer PROGRESS   : " ,round(float(pulse_wg2*(1/wg_thread_pitch)),3),"mm" )
            print ("No. of Layers Completed : ", int(no_of_layers) )
            countdown = mapp(pulse_sp,0,total_turns,total_turns,0)
            print ("Main Spindle Countdown : ", int(countdown))
            print ("Main Spindle Count : ", int(pulse_sp), "---->", int(total_turns))
            print ("Main Spindle RPM : ", round(f,2))
            print("Processor Temperature : ", float(temp),"*C")
            print("Current spindle in mm : ", float(pulse_sp2))
            if(temp >= upper_temperature_threshold):
                status = 1
                GPIO.output(temp_pin,status)
            if(temp <= lower_temperature_threshold):    
                status = 0
                GPIO.output(temp_pin,status)
            if(status == 1):    
             print("Cabinet Exhaust Fan Status : ACTIVE")
            else:
             print("Cabinet Exhaust Fan Status : OFF")
            e = f
            minu2 = total_turns/e
            o1 = minu2
            minu2 = (minu2 + (o1*10))/n1
            minu3 = int(minu2)
            if(minu2 >= 1):
             print ("Time taken : ", minu,"minutes",int(sec), "seconds")
            else:
                print ("Time taken : ", minu,"minutes",int(sec), "seconds")
            if(wg_dir_state == 1):
             GPIO.output(led_left,1)
             GPIO.output(led_right,0)
            else:
             GPIO.output(led_left,0)
             GPIO.output(led_right,1)
            print("Selected winding width : ", winding_width,"\n\n")
            if(int(pulse_sp) >= total_turns):
             sp.write(str.encode(min_freq_str1))
             GPIO.output(wg_ena,0)
             GPIO.output(sp_ena,0)
             GPIO.output(temp_pin,0)
             print("\nWINDING PROCESS COMPLETED\n")
             GPIO.output(led_left,1)
             GPIO.output(led_right,1)
             time.sleep(0.25)
             GPIO.output(led_left,0)
             GPIO.output(led_right,0)
             time.sleep(0.25)
             GPIO.output(led_left,1)
             GPIO.output(led_right,1)
             time.sleep(0.25)
             GPIO.output(led_left,0)
             GPIO.output(led_right,0)
             job_comp = 1
             restart = 0
             jc = open("jc.pickle", "wb")
             dump(job_comp, jc)
             jc.close()
             #time.sleep(1)
             sys.exit()
             break
            if(master_ratio_dynamic < 0):
             sp.write(str.encode(min_freq_str1))
             GPIO.output(wg_ena,0)
             GPIO.output(sp_ena,0)
             print("Error - Wire Gauge too thin. Choose a thicker wire gauge and try again!!","\n")
             break
            if(GPIO.input(pause_ip) == 0):
             if(int(pulse_sp) >= total_turns):
                 sys.exit()
                 break
             if(GPIO.input(stop_button) == 0):
                  GPIO.output(wg_ena,0)
                  GPIO.output(sp_ena,0)
                  sp.write(str.encode('F500'))
                  wg.write(str.encode('F500'))
                  GPIO.output(led_left,0)
                  GPIO.output(led_right,0)
                  print("\nPROCESS ABORTED")
                  fw = open("p.pickle", "wb")
                  val = [pulse_sp,pulse_wg,pulse_wg2,w,wg_dir_state,no_of_layers,sec,minu,pulse_sp2]
                  dump(val,fw)
                  fw.close()
                  job_comp = 0
                  jc = open("jc.pickle", "wb")
                  dump(job_comp, jc)
                  jc.close()
                  sys.exit()
                  break
            else:
              print("\nPAUSED\n")

    def pwm_gen(): #Dedicated Thread No.4 for updating frequency generators via user rotary encoder
     global counter_sp
     global counter_wg
     global sp
     global pulse_sp
     global resolution
     global wg
     global f
     global total_turns
     global wire_thickness
     global winding_width
     global wind_gap_comp_factor
     global spr_sp
     global spr_wg
     global upper_lim_freq
     global upl_wg
     global wg_dir_switch
     global guide_travel
     global pulse_wg
     global wg_dir
     global wg_thread_pitch
     global turns_per_layer
     global actual_turns_per_layer
     global user_set_freq
     global wg_freq
     global master_ratio_dynamic
     global spr_sp
     global spr_wg
     global a
     global b
     global master_ratio
     global pause_ip
     global min_freq
     global min_freq_str1
     global wg_ena
     global sp_ena
     global stop_button, counter_sp
     pause_freq = 999
     pause_str = 'F'
     pause_strfreq = ''
     while(1):
        time.sleep(0.1)
        #GPIO.output(sp_ena, 1)
        user_set_freq = mapp(counter_sp,0,resolution,min_freq,upper_lim_freq)
        if(user_set_freq >= 10.00):
         user_set_freq = round(user_set_freq,1)
        else:
         user_set_freq = round(user_set_freq,2)
        wg_freq = mapp(counter_wg,0,resolution, min_freq/master_ratio,upl_wg)
        if(wg_freq < 1):
         wg_freq = wg_freq * 1000
         wg_freq = round(wg_freq,0)
         str_wg2 = str(int(wg_freq))
         master_ratio_dynamic = user_set_freq/(wg_freq/1000)
        else:
         if(wg_freq >= 10.00):
          wg_freq = round(wg_freq,1)
         else:
          wg_freq = round(wg_freq,2)
         str_wg2 = str((wg_freq))
         master_ratio_dynamic = user_set_freq/wg_freq
        str_sp = 'F'
        str_wg = 'F'
        str_sp2 = str(float(user_set_freq))
        str_sp += str_sp2
        str_wg += str_wg2
        if(GPIO.input(pause_ip) == 0):
         sp.write(str.encode(str_sp))
         wg.write(str.encode(str_wg))
         GPIO.output(wg_ena,1)
        if(GPIO.input(pause_ip) == 1):
         #pause_freq -= 10
         #pause_strfreq = str(pause_freq)
         #pause_str += pause_strfreq
         sp.write(str.encode('F000'))    
         if(spr_wg == 6400):
          wg.write(str.encode('F19.9'))
         else:
          wg.write(str.encode('F9.95'))
         if(GPIO.input(stop_button) == 0):    
          GPIO.output(wg_ena,1)
         else:
          GPIO.output(wg_ena,0)  
         counter_sp = 0
        f = (user_set_freq * 7.70089285301) * (3200/spr_sp)
        turns_per_layer = winding_width/wire_thickness
        actual_turns_per_layer = turns_per_layer * wind_gap_comp_factor
        if(int(pulse_sp) >=total_turns):
             GPIO.output(sp_ena,0)
             GPIO.output(wg_ena,0)
             print("\nWINDING PROCESS COMPLETED\n")
             sys.exit()
             break
        if(master_ratio_dynamic < 0):
             sp.write(str.encode(min_freq_str1))
             GPIO.output(wg_ena,0)
             GPIO.output(sp_ena,0)
             print("Error - Wire Gauge too thin. Choose a thicker wire gauge and try again!!","\n")
             break
        if(GPIO.input(pause_ip) == 0):    
         if(GPIO.input(stop_button) == 0 or int(pulse_sp) >=total_turns):
                  GPIO.output(sp_ena,0)
                  GPIO.output(wg_ena,0)
                  sys.exit()
                  break

    def pid(): #Dedicated Thread No.2 for PID
            global master_ratio_dynamic
            global master_ratio
            global min_freq_str1
            global kp
            global kd
            global ki
            global total_turns
            global counter_wg
            prev_error = 0
            sum_error = 0
            global pulse_sp
            global wg_ena
            global sp_ena
            global stop_button
            while(1):
                    if(GPIO.input(pause_ip) == 0):
                     time.sleep(0.101)
                     #GPIO.output(wg_ena, 1)
                     #GPIO.output(sp_ena, 1)
                     
                     error =  1*(master_ratio_dynamic - master_ratio)
                     counter_wg = (error * kp) + (prev_error * kd) + (sum_error * ki)
                     prev_error = error
                     sum_error += error
                     if(int(pulse_sp) >=total_turns):
                      GPIO.output(sp_ena,0)
                      GPIO.output(wg_ena,0)
                      print("\nWINDING PROCESS COMPLETED\n")
                      sys.exit()
                      break
                    else:
                     counter_sp = 0  
                    if(master_ratio_dynamic < 0):
                     sp.write(str.encode(min_freq_str1))
                     GPIO.output(wg_ena,0)
                     GPIO.output(sp_ena,0)
                     print("Error - Wire Gauge too thin. Choose a thicker wire gauge and try again!!","\n")
                     break
                    if(GPIO.input(pause_ip) == 0):
                     if(GPIO.input(stop_button) == 0 or int(pulse_sp) >=total_turns):
                      GPIO.output(sp_ena,0)
                      GPIO.output(wg_ena,0)
                      sys.exit()
                      break

    def play():
          global input_SW
          global counter_sp
          global min_freq_str1
          global resolution
          global wg_dir
          global wg_ena
          global master_ratio_dynamic
          global wg_dir_state
          global pulse_wg2
          global pulse_sp
          global wg_dir_state
          global winding_width
          global wg_thread_pitch
          global total_turns
          global dir_reset
          global no_of_layers
          global sp_ena
          global stop_button
          global wind_gap_comp_factor, wg_speed
          global w, wire_thickness, pulse_sp2
          while(1):
           time.sleep(0.1)
           GPIO.output(sp_ena, 1)
           #if(wg_dir_state == 1):
           # if(float(pulse_sp2) >= (float(winding_width/wire_thickness))):
               #wg_dir_state = not wg_dir_state
               #w = not w
               #pulse_wg2 = 0
               #pulse_sp2 = 0
               #no_of_layers += 1
          # else:
             # if(GPIO.input(dir_reset) == 1):
                #wg_dir_state = not wg_dir_state
                #w = not w
                #pulse_wg2 = 0
               # pulse_sp2 = 0
               # no_of_layers += 1
            
           GPIO.output(wg_dir, wg_dir_state)
           if(int(pulse_sp) >=total_turns):
             print("\nWINDING PROCESS COMPLETED\n")
             GPIO.output(sp_ena,0)
             GPIO.output(wg_ena,0)  
             sys.exit()
             break
           if(master_ratio_dynamic < 0):
             sp.write(str.encode(min_freq_str1))
             GPIO.output(wg_ena,0)
             GPIO.output(sp_ena,0)
             print("Error - Wire Gauge too thin. Choose a thicker wire gauge and try again!!","\n")
             break
           if(GPIO.input(pause_ip) == 0):
            if(GPIO.input(stop_button) == 0 or int(pulse_sp) >=total_turns):
                  GPIO.output(sp_ena,0)
                  GPIO.output(wg_ena,0)
                  sys.exit()
                  break

    def accel():
        global counter_sp
        global pulse_sp
        global total_turns
        global min_freq_str1
        global wg_ena
        global resolution
        global count_at_max
        global master_ratio_dynamic, master_ratio
        global f
        global spndl_rpm
        global pulse_sp
        global sp_ena, sp, start_end_width
        global stop_button, pause_ip, pulse_wg2, wg_thread_pitch, winding_width
        while(GPIO.input(pause_ip) == 0):
         while(1):
             time.sleep(0.068)
             if(float(pulse_wg2*(1/wg_thread_pitch)) >= (float(winding_width - start_end_width)) or float(pulse_wg2*(1/wg_thread_pitch)) <= float(start_end_width)):
                if(counter_sp >= 40):
                    counter_sp -= (20 * spndl_rpm/40) 
                elif(counter_sp < 40):
                    counter_sp += (20 * spndl_rpm/40) 
                else:
                    counter_sp = 40
             elif(float(pulse_wg2*(1/wg_thread_pitch)) >= float(start_end_width)):    
              if(counter_sp < resolution):
               counter_sp += (20 * spndl_rpm/40) 
              else:
               counter_sp = resolution
              if(f == spndl_rpm):
                 count_at_max = pulse_sp
             if(int(pulse_sp) >=total_turns):
              print("\nWINDING PROCESS COMPLETED\n")  
              GPIO.output(sp_ena,0)
              GPIO.output(wg_ena,0)  
              sys.exit()  
              break
             if(master_ratio_dynamic < 0):
              sp.write(str.encode(min_freq_str1))
              GPIO.output(wg_ena,0)
              GPIO.output(sp_ena,0)
              print("Error - Wire Gauge too thin. Choose a thicker wire gauge and try again!!","\n")
              break
             if(GPIO.input(pause_ip) == 0):
              if(GPIO.input(stop_button) == 0 or int(pulse_sp) >=total_turns):
                   GPIO.output(sp_ena,0)
                   GPIO.output(wg_ena,0)
                   sys.exit()
                   break
    
    def decel():
        global counter_sp
        global countdown
        global pulse_sp, pulse_wg2, wg_thread_pitch, winding_width
        global master_ratio_dynamic
        global total_turns
        global resolution
        global spndl_rpm
        global upper_bound
        global lower_bound
        global upper_lim_freq
        global min_freq
        global user_set_freq
        global decel_window
        global p
        global min_freq_str1
        global wg_ena
        global sp_ena
        global stop_button
        global e
        global spndl_rpm, pause_ip, stop_button
        while(1):
            time.sleep(0.068)
            p = total_turns - pulse_sp        
            if(p <= decel_window ):
                counter_sp = mapp(p,decel_window,0,resolution,0)
                e = spndl_rpm
            if(int(pulse_sp) >=total_turns):
             print("\nWINDING PROCESS COMPLETED\n")  
             GPIO.output(sp_ena,0)
             GPIO.output(wg_ena,0)
             sys.exit()  
             break
            if(master_ratio_dynamic < 0):
             sp.write(str.encode(min_freq_str1))
             GPIO.output(wg_ena,0)
             GPIO.output(sp_ena,0)
             print("Error - Wire Gauge too thin. Choose a thicker wire gauge and try again!!","\n")
             break
            if(GPIO.input(pause_ip) == 0):
             if(GPIO.input(stop_button) == 0 or int(pulse_sp) >=total_turns):
                  GPIO.output(sp_ena,0)
                  GPIO.output(wg_ena,0)
                  sys.exit()
                  break
    ######################################################################################################3
    t1 = threading.Thread(target = wg_enc)
    t2 = threading.Thread(target = pid)
    t3 = threading.Thread(target = rot_enc)
    t4 = threading.Thread(target = pwm_gen)
    t5 = threading.Thread(target = play)
    t6 = threading.Thread(target = accel)
    t7 = threading.Thread(target = decel)
    t8 = threading.Thread(target = manual_wg_dir)

    time.sleep(0.1)
    calibration  = 0
    GPIO.output(wg_ena,0)
    GPIO.output(sp_ena,0)

    if(pckl == 'n' or pckl == 'N'):
     print("Engage free running mode? (In this mode, wire-guide motor is disabled)\n")
     print("Press 'Y/y' for YES")
     print("Press 'N/n' for NO\n")
     free_run = input(">>")
     if(free_run == 'y' or free_run == 'Y'):
      while(1):
       time.sleep(0.1)
       GPIO.output(sp_ena, 1) 
       if(spr_sp == 6400):  
        sp.write(str.encode('F15.1'))
       else:
        sp.write(str.encode('F7.51'))
       if(GPIO.input(stop_button) == 0):
        GPIO.output(sp_ena, 0)
        time.sleep(0.5)
        sp.write(str.encode('F500')) 
        time.sleep(0.5)
        break    

    if(job_comp == 1 or pckl == 'n' or pckl == 'N'):
     print("\nInitiate calibration/wire guide adjustment for wire guide start point?\n")
     print("Press 'y' or 'Y' for YES")
     print("Press 'n'or 'N' to skip")
     print("Press 'a' or 'A' to adjust wire guide and skip calibration")
     print("Press 'c' or 'C' to CANCEL/exit program")
     e2 = input(">>")
    else:
        e2 = 'n'
        calibration  = 0
    if(e2 == 'c' or e2 == 'C'):
        sys.exit()
    if( e2 == 'Y' or e2 == 'y' or e2 == 'a' or e2 == 'A'):
     calibration == 1  
     GPIO.output(fg_reset,1)
     time.sleep(0.5)
     GPIO.output(fg_reset,0)
     manual_after_cali = 0
     while(1):
         cali_timer = 0
         if(manual_after_cali == 1 or e2 == 'A' or e2 == 'a'):
          while(1):
           time.sleep(0.1)   
           print("1) To manually adjust the wire guide, use the ABORT/MANUAL(black) button\n")
           print("2) Use the WG_DIR(red) button to change direction of wire guide for adjustment\n")
           print("3) Press/hold the WG_DIR(red) button for 2 seconds to skip this stage\n\n")
           if(spr_wg == 6400):
            wg.write(str.encode('F20.9'))
           else:
            wg.write(str.encode('F10.5'))
           GPIO.output(wg_dir, wg_dir_state)
           if(GPIO.input(stop_button) == 0):
              GPIO.output(wg_ena, 1)
           else:
              GPIO.output(wg_ena, 0)
           if(GPIO.input(wg_manual_dir) == 0):
              wg_dir_state = not wg_dir_state
              time.sleep(1)
              cali_timer += 1.1 
           else:
              cali_timer = 0
           if(wg_dir_state == 0):
            GPIO.output(led_left,0)
            GPIO.output(led_right,1)
            direction = 1
           else: 
            GPIO.output(led_left,1)
            GPIO.output(led_right,0)
            direction = 0
           if(cali_timer > 2):
              break
           else:
              continue
         if(e2 == 'a' or e2 == 'A'):
             wg.write(str.encode('F500'))
             sp.write(str.encode('F500'))
             time.sleep(0.1)
             sp.write(str.encode('D025'))
             wg.write(str.encode('D025'))
             GPIO.output(led_left,1)
             GPIO.output(led_right,1)
             GPIO.output(fg_reset,1)
             time.sleep(0.25)
             GPIO.output(led_left,0)
             GPIO.output(led_right,0)
             time.sleep(0.25)
             GPIO.output(led_left,1)
             GPIO.output(led_right,1)
             GPIO.output(fg_reset,0)
             time.sleep(0.25)
             GPIO.output(led_left,0)
             GPIO.output(led_right,0)
             time.sleep(0.5)
             GPIO.output(led_left,1)
             GPIO.output(led_right,0)
             print("Begin winding from :\n")
             print("1. Right?")
             print("2. Left?\n")
             direction = int(input("Choose option no.>> "))
             if(direction == 1):
                 wg_dir_state = 1
                 w = 1
                 GPIO.output(led_left,1)
                 GPIO.output(led_right,0)
             else:
                 wg_dir_state = 0
                 w = 0
                 GPIO.output(led_left,0)
                 GPIO.output(led_right,1)
             break
         else:   
            manual_after_cali = 1
            if(cali_timer <= 2):
             while(1):
              time.sleep(0.1)
              GPIO.output(wg_ena,1)  
              print("\nCalibration In Progress....Please Wait", "\n")
              if(GPIO.input(dir_reset) == 0): #When red light is on
               GPIO.output(wg_dir,0)  
               initial_freq += 1.25
               if(initial_freq > 1.31):
                   initial_freq = round(initial_freq,1)
               if(spr_wg == 6400):
                if(initial_freq > 19.9):
                    initial_freq = 19.9
               else:
                if(initial_freq > 9.95):
                    initial_freq = 9.95
               freq_str = 'F' + str(float(initial_freq))
               wg.write(str.encode(freq_str))
               time.sleep(0.02)
              else:                   # When red light is OFF
               GPIO.output(wg_ena, 0)  
               GPIO.output(wg_dir, 1)  
               break
             time.sleep(0.1)
             while(GPIO.input(dir_reset) == 1):
              GPIO.output(wg_ena, 1)
              time.sleep(0.5)
              wg.write(str.encode('F8.05'))
              time.sleep(0.01)
             wg.write(str.encode('F9.01'))
             GPIO.output(wg_ena, 0)
             time.sleep(0.01)
             GPIO.output(wg_dir, 0)
             GPIO.output(wg_ena, 1)

             time.sleep(0.1)
             while(1):
               time.sleep(0.1)  
               wg.write(str.encode('F900'))
               if(GPIO.input(dir_reset) == 1):
                wg.write(str.encode('F000'))
                time.sleep(0.3)
                GPIO.output(wg_ena,0)
                wg.write(str.encode('F500'))
                sp.write(str.encode('F500'))
                time.sleep(0.1)
                sp.write(str.encode('D025'))
                wg.write(str.encode('D025'))
                print("\nCALIBRATION COMPLETE!!")
                GPIO.output(led_left,1)
                GPIO.output(led_right,1)
                GPIO.output(fg_reset,1)
                time.sleep(0.25)
                GPIO.output(led_left,0)
                GPIO.output(led_right,0)
                time.sleep(0.25)
                GPIO.output(led_left,1)
                GPIO.output(led_right,1)
                GPIO.output(fg_reset,0)
                time.sleep(0.25)
                GPIO.output(led_left,0)
                GPIO.output(led_right,0)
                time.sleep(0.5)
                GPIO.output(led_left,1)
                GPIO.output(led_right,0)
                direction = 0
                break
         
         print("Do you want to re-adjust wire guide manually?\n")
         print("Press 'Y/y' for YES")
         print("Press 'N/n/ for NO\n")
         e7 = input(">>")
         if(e7 == 'y' or e7 == 'Y'):
            continue
         else:
            break
    if(e2 == 'n' or e2 == 'N'):
     wg.write(str.encode('F500'))
     sp.write(str.encode('F500'))
     time.sleep(0.1)
     sp.write(str.encode('D025'))
     wg.write(str.encode('D025'))
     GPIO.output(led_left,1)
     GPIO.output(led_right,1)
     GPIO.output(fg_reset,1)
     time.sleep(0.25)
     GPIO.output(led_left,0)
     GPIO.output(led_right,0)
     time.sleep(0.25)
     GPIO.output(led_left,1)
     GPIO.output(led_right,1)
     GPIO.output(fg_reset,0)
     time.sleep(0.25)
     GPIO.output(led_left,0)
     GPIO.output(led_right,0)
     time.sleep(0.5)
     GPIO.output(led_left,1)
     GPIO.output(led_right,0)

     if(pckl == 'n' or pckl == 'N'):
      print("Begin winding from :\n")
      print("1. Right?")
      print("2. Left?\n")
      direction = int(input("Choose option no.>> "))
     if(direction == 1):
       wg_dir_state = 1
       w = 1
       GPIO.output(led_left,1)
       GPIO.output(led_right,0)
     else:
       wg_dir_state = 0
       w = 0
       GPIO.output(led_left,0)
       GPIO.output(led_right,1)


    if(wg_dir_state == w == 1):
     GPIO.output(led_left,1)
     GPIO.output(led_right, 0)
    else:
     GPIO.output(led_left,0)
     GPIO.output(led_right, 1)     
    GPIO.output(fg_reset,1)
    time.sleep(0.5)
    GPIO.output(fg_reset,0)
    time.sleep(0.5)
    wg.write(str.encode('F500'))
    sp.write(str.encode('F500'))
    no_of_layers = 0
    pulse_sp = 0
    pulse_sp2 = 0


    print("\nNOTE : PLEASE 'ZERO' THE MECHANICAL COUNTER BEFORE PROCEEDING!\n")
    print("Begin Winding Process?\n")
    print("Press 'y' or 'Y' for YES")
    print("Press 'n' or 'N' to CANCEL")
    e = input(">>")
    if(e == 'y' or e == "Y"):
     time.sleep(0.1)
     no_of_layers = 0
     pulse_sp = 0
     pulse_sp2 = 0
     pulse_wg = 0
     pulse_wg2 = 0
     print("Winding will start in 3")
     time.sleep(1)
     print("Winding will start in 2")
     time.sleep(1)
     print("Winding will start in 1")
     time.sleep(1)
     

     
     t1.start()
     t2.start()
     t3.start()
     t4.start()
     t5.start()
     t6.start()
     t7.start()
     t8.start()

     t1.join()
     t2.join()
     t4.join()
     t5.join()
     t6.join()
     t7.join()
     t8.join()
     GPIO.output(fg_reset,1)
     time.sleep(0.5)
     print("Would you like to START a NEW SESSION?\n")
     GPIO.output(fg_reset,0)
     time.sleep(0.5)
     print("Press 'y' or 'Y' for YES")
     print("Press any other KEY to EXIT")
     wp = input(">>")
     if(wp == 'y' or wp == 'Y'):
         continue
     else:
         break
     t3.join()
     
    if(e == 'n' or e == 'N'):
        GPIO.output(led_left,0)
        GPIO.output(led_right,0)
        sys.exit()

   

