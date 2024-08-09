import math
import time
import pyvisa as pv
import threading

rm = pv.ResourceManager()
print(rm.list_resources())

SIG1HP = pv.ResourceManager().open_resource('GPIB0::28::INSTR')
SIG2 = pv.ResourceManager().open_resource('GPIB0::19::INSTR')
SIG3AG = pv.ResourceManager().open_resource('GPIB0::29::INSTR')
OSC = pv.ResourceManager().open_resource('GPIB0::7::INSTR')

#initalizes SIG1
def SIG1HPInitalize():
    SIG1HP.write(f'*RST')
    SIG1HP.write(f'FREQuency:CW 10 MHZ')
    SIG1HP.write(f'AMPLitude:LEVEL 10 DBM')
    SIG1HP.write(f'AMPLitude:STATE ON')

#sets the phase resolution
def setSIG1HPPhaseStep(phase):
    SIG1HP.write(f'PHAS:STEP {phase}DEG')

#changes the phase
def SIG1HPPhaseUp():
    SIG1HP.write(f'PHAS UP')
    #compare with oscilloscope, to 0 deg, then set reference phase
    #SIG1HP.write(f'PHAS:REF')

#initializes SIG2
def SIG2Initialize():
    SIG2.write(f'*RST')
    SIG2.write(f'FREQuency:CW 10 MHZ')
    SIG2.write(f'POW:AMPLITUDE 10 DBM')
    SIG2.write(f'OUTP:MOD OFF')
    SIG2.write(f'OUTP:STAT ON')

#changes SIG2 phase in degrees
def setSIG2Phase(phase):
    SIG2.write(f'PHAS:ADJ {phase} DEG')
#compare with oscilloscope, to 0 deg, then set reference phase
#SIG2.write(f'PHAS:REF')

#initializes SIG3AG
def SIG3AGInitialize():
    SIG3AG.write(f'*RST')
    SIG3AG.write(f'FREQuency:CW 10 MHZ')
    SIG3AG.write(f'POW:AMPLITUDE 10 DBM')
    SIG3AG.write(f'OUTP:MOD OFF')
    SIG3AG.write(f'OUTP:STAT ON')

#changes SIG3AG phase in degrees
def setSIG3AGPhase(phase):
    SIG3AG.write(f'PHAS:ADJ {phase} DEG')

#initalizes Oscilloscope
def oscInitalize():
    OSC.write(f'AUToscale CHANnel1,CHANnel2,CHANnel3')
    OSC.write(f'CHAN1:IMP FIFTy')
    OSC.write(f'CHAN2:IMP FIFTy')
    OSC.write(f'CHAN3:IMP FIFTy')
    OSC.write(f'CHAN4:IMP FIFTy')
    OSC.write(f'TIMebase:RANGe 50E-8')
    OSC.write(f'ACQ:TYPE AVER')
    OSC.write(f'ACQ:COUNt 4096')
    OSC.write(f'CHAN1:SCAL 1V')
    OSC.write(f'CHAN2:SCAL 1V')
    OSC.write(f'CHAN3:SCAL 1V')
    OSC.write(f'CHAN4:SCAL 1V')
    OSC.write(f'CHAN1:RANGe 8V')
    OSC.write(f'CHAN2:RANGe 8V')
    OSC.write(f'CHAN3:RANGe 8V')
    OSC.write(f'CHAN4:RANGe 8V')
    OSC.write(f'CHAN1:OFFSet 0V')
    OSC.write(f'CHAN2:OFFSet 0V')
    OSC.write(f'CHAN3:OFFSet 0V')
    OSC.write(f'CHAN4:OFFSet 0V')
    OSC.write(f'TRIG:SOURce CHAN3')
    OSC.write(f'TRIG:EDGE:SLOP POS')
     #CMD to turn on ch4 once it's fixed
    #OSC.write(f'CHAN4:DISp 1')

#checks phase after each sync
def checksPhase(channel):
    """for x in range(3):
        phaseMeasured = int(float(OSC.query(f'MEAS:PHASe? CHAN{channel},CHAN3')))
        if(phaseMeasured!=0):
            return False
        time.sleep(1)"""
    return True

#syncs Signal Generator 2 on the top (w/ binary)  
def syncSIG2Binary(target_phase=0.0, iterations=50):
    #while(not checksPhase(1)):
        low = -180.0
        high = 180.0
        for x in range(iterations):
            mid = (low + high) / 2.0
            setSIG2Phase(mid)
            time.sleep(0.2) 
            measured_phase = int(float(OSC.query(f'MEAS:PHASe? CHAN1,CHAN3')))
            phase_diff = measured_phase - target_phase
            if abs(phase_diff) < 2:
                break

            if phase_diff > 0:
                high = mid
            else:
                low = mid
        SIG2.write(f'PHAS:REF')
        resetMeasureStatistics()
        

#Syncs Signal Generator 2 (w/o binary)
def syncSIG2():
    while(not checksPhase(1)):
        phaseMeasured = int(float(OSC.query(f'MEAS:PHASe? CHAN1,CHAN3')))
        if(phaseMeasured>0):
            setSIG2Phase(-phaseMeasured)
        else:
            setSIG2Phase(phaseMeasured)
        SIG2.write(f'PHAS:REF')
        time.sleep(2)

#syncs Signal Generator 3 on the left (w/ binary)
def syncSIG3AGBinary(target_phase=0.0, iterations=10):
    #while(not checksPhase(2)):
        low = -180.0
        high = 180.0
        for x in range(iterations):
            mid = (low + high) / 2.0
            setSIG3AGPhase(round(mid,1))
            time.sleep(0.2) 
            measured_phase = int(float(OSC.query(f'MEAS:PHASe? CHAN2,CHAN3')))
            phase_diff = measured_phase - target_phase
            if abs(phase_diff) < 2:
                break

            if phase_diff > 0:
                high = mid
            else:
                low = mid
        SIG3AG.write(f'PHAS:REF')
        resetMeasureStatistics()

#Syncs Signal Generator 3 (w/o binary)
def syncSIG3AG():
    while(not checksPhase(2)):
        phaseMeasured = int(float(OSC.query(f'MEAS:PHASe? CHAN2,CHAN1')))
        if(phaseMeasured>0):
            setSIG3AGPhase(-phaseMeasured)
        else:
            setSIG3AGPhase(phaseMeasured)
        SIG3AG.write(f'PHAS:REF')
        time.sleep(0.5)

#Resets measure statistics (counter starts at 0)
def resetMeasureStatistics():
    OSC.write(f'MEAS:STAT:RES')

def voltage_to_dBm(vpp, impedance=50):
    # Convert peak-to-peak voltage to RMS voltage
    voltage_rms = vpp / (2 * math.sqrt(2))
    # Calculate power in watts
    power_watts = (voltage_rms ** 2) / impedance
    # Convert watts to milliwatts
    power_milliwatts = power_watts * 1000
    # Calculate dBm
    dBm = 10 * math.log10(power_milliwatts)
    return dBm

def syncSIG2Voltage():
    while True:
        #notes down currentdBm of the signal generator
        currentdBm = (float(SIG2.query(f'POW:AMPLITUDE?')))
        #calculates adjustment based on the dBms from the oscilloscope readings
        adjustment = voltage_to_dBm((float(OSC.query(f'MEAS:VPP? CHAN3')))) - voltage_to_dBm((float(OSC.query(f'MEAS:VPP? CHAN1'))))
        #calculates new_dBm based on adjustment
        new_dBm = currentdBm + adjustment
        #signal generator changes to new_dBm
        SIG2.write(f'POW:AMPLITUDE {new_dBm} DBM')
        #newVpp is measured on the oscilloscope
        newVpp = (float(OSC.query(f'MEAS:VPP? CHAN1')))
        #condition
        if (abs(float(OSC.query(f'MEAS:VPP? CHAN3')) - newVpp) < 0.005):
            return
        
def syncSIG3AGVoltage():
    while True:
        #notes down currentdBm of the signal generator
        currentdBm = (float(SIG3AG.query(f'POW:AMPLITUDE?')))
        #calculates adjustment based on the dBms from the oscilloscope readings
        adjustment = voltage_to_dBm((float(OSC.query(f'MEAS:VPP? CHAN3')))) - voltage_to_dBm((float(OSC.query(f'MEAS:VPP? CHAN2'))))
        #calculates new_dBm based on adjustment
        new_dBm = currentdBm + adjustment
        #signal generator changes to new_dBm
        SIG3AG.write(f'POW:AMPLITUDE {new_dBm} DBM')
        #newVpp is measured on the oscilloscope
        newVpp = (float(OSC.query(f'MEAS:VPP? CHAN2')))
        #condition
        if (abs(float(OSC.query(f'MEAS:VPP? CHAN3')) - newVpp) < 0.005):
            return
#Helper function for Watchdog
def adjPhaseSIG2(phaseMeasured,calibrationFactor=0.9):
    phaseCorrection = (phaseMeasured) * calibrationFactor
    SIG2.write(f'PHAS:REF')
    #phaseMeasured = int(float(OSC.query(f'MEAS:PHASe? CHAN1,CHAN3')))
    setSIG2Phase(-phaseCorrection)
    print(f'Adjusted phase of Signal Generator 2 by {phaseCorrection} degrees')
def adjPhaseSIG3AG(phaseMeasured,calibrationFactor=0.9):
    phaseCorrection = phaseMeasured * calibrationFactor
    SIG3AG.write(f'PHAS:REF')
    #phaseMeasured = int(float(OSC.query(f'MEAS:PHASe? CHAN1,CHAN3')))
    setSIG3AGPhase(-phaseCorrection)
    print(f'Adjusted phase of Signal Generator 3 by {phaseCorrection} degrees')   
#watchdog
def phase_watchdog(tolerance=1, interval=10,calibrationFactor=0.9):
    while True:
        phaseMeasuredSIG2 = (float(OSC.query(f'MEAS:PHASe? CHAN1,CHAN3'))) 
        phaseMeasuredSIG3AG = (float(OSC.query(f'MEAS:PHASe? CHAN2,CHAN3'))) 
        if abs(phaseMeasuredSIG2) > tolerance:
            adjPhaseSIG2(phaseMeasuredSIG2,calibrationFactor)
        if abs(phaseMeasuredSIG3AG) > tolerance:
            adjPhaseSIG3AG(phaseMeasuredSIG3AG,calibrationFactor)
        if abs(phaseMeasuredSIG2 <= tolerance and phaseMeasuredSIG3AG <= tolerance):
            print("Phase within tolerance, no adjustment needed")
        time.sleep(interval)

""" plan to change to arbitrary phase
def syncSIG3AGBinary(target_phase, iterations=50):
    while(not checksPhase(2)):
        low = -180.0
        high = 180.0
        for x in range(iterations):
            mid = (low + high) / 2.0
            setSIG3AGPhase(round(mid,1))
            time.sleep(0.1) 
            measured_phase = int(float(OSC.query(f'MEAS:PHASe? CHAN2,CHAN3')))
            phase_diff = measured_phase - target_phase
            if abs(phase_diff) == target_phase:
                return mid  

            if phase_diff > 0:
                high = mid
            else:
                low = mid
        SIG3AG.write(f'PHAS:REF')
    resetMeasureStatistics()
    """
"""to-do for tomorrow
implement a watchdog system that continuously changes the phase to 0!

:MEASure:VPP? [<source>
Vpp = float(OSC.query(f'MEAS:VPP?'))
"""
#main

#SIG1HPInitalize()
#SIG2Initialize()
#SIG3AGInitialize()

#setSIG1HPPhaseStep(30)
#SIG1HPPhaseUp()

#setSIG2Phase(30)
#setSIG3AGPhase(30)

#OSC.write(f'*RST')
#oscInitalize()

#syncSIG2Voltage()
syncSIG2Binary(30)
#syncSIG3AGVoltage()
syncSIG3AGBinary(60)

#watchdog_thread = threading.Thread(target=phase_watchdog)
#watchdog_thread.start()



"""
time.sleep(2)
currentdBm = voltage_to_dBm((float(OSC.query(f'MEAS:VPP? CHAN1'))))
adjustment = target_dBm - currentdBm
print(currentdBm)
print(adjustment)
new_dBm = currentdBm + adjustment
SIG2.write(f'POW:AMPLITUDE {new_dBm} DBM')

currentdBm = voltage_to_dBm(float(OSC.query(f'MEAS:VPP? CHAN1')))
adjustment = target_dBm - currentdBm
new_dBm = currentdBm + adjustment
SIG2.write(f'POW:AMPLITUDE {new_dBm} DBM')"""




"""random (useful?) commands
OSC.write(f'MEASure:STATistics CURRent') (pick current to measure)
current = OSC.query(f'MEAS:RES?') (measures it)
print(current)
"""


