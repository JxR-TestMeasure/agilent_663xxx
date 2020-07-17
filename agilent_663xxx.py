# -*- coding: utf-8 -*-

import pyvisa
import numpy as np

'''
Instrument Driver for:  Agilent / Keysight
                        Model: 663XX Mobile Communication DC Power Source
Requires: pyvisa, numpy
                    
                    import agilent_663XX as dev
Class flow          dev = Device(VISA::ADDRESS)
Device:             dev
    Common:         dev.com
    Display:        dev.display
    Format:         dev.format
    Channel:        dev.ch<1,2>
        Measure:    dev.ch<1,2>.meas
        Protection: dev.ch<1,2>.prot
    Log:            dev.log
                     dev.log.log_data{nd.array(meas), nd.array(time)} : trig measurement data
        Common:     dev.log.com
        Status:     dev.log.status
        Trigger:    dev.log.trig
    Status:         dev.status

*****
Note:   Many functions provide both 'get' and 'set' type behavior 
*****   Example:
            
            dev.ch1.voltage()               ; returns the set voltage
            dev.ch1.voltage(5)              ; Ch1 set V = 5V
            dev.ch1.meas.voltage()          ; get measurement - Measure:(meas) has no 'set' types
            dev.ch1.meas.v()                ; same as meas.voltage()
            dev.ch1.meas.cmax()             : return a peak current measurement cmax() = current_max()
            dev.com.sre()                   ; query the standard event register
            dev.com.sre(128)                ; set bit 7 in SRE
            dev.status.get_error_queue()    ; function name implicitly states it returns a value
            dev.log.start_measure_sample()  ; return sampling data to log.log_data (run after trig/chan/log config)

Error Handling:
    On error due to different invalid inputs:
    All methods 'should' raise TypeError, and provide the correct types
    All methods 'should' raise ValueError, and provide a set of valid inputs, or a range

Device:   
    Establishes communication to instrument.
    Provides access structure for instrument.
    Methods for direct SCPI communication
Common:
    Provides methods for common 488.2 commands
    Most methods provide 'get' with no params, and set with the passed value
    Provides methods for accessing standard byte, standard event registers
Display:
    System commands for changing main display elements
Format:
    System commands to format :ARRay type return queries
    and byte order.  Default settings are fastest
Channel:
    Turn channel output on and off
    Includes all main channel settings for normal operations
    Most functions provide 'get' with no params, and 'set' with the passed value
    Provides access to Measure class
Measure:
    All methods are of type 'get'
    Provides all available 'single' measurement values: voltage, current, power(calculated)
    Provides all statistical: min, max, avg, high, low, rms values
Log:
    Sets and returns values related to integration time, sample points, etc
    Provides function: start_measure_sample to initiate a high speed data acquisition
    Provides the data storage for high speed data acquisition
    Provides access to the Trigger class
Trigger:
    Provides many get/set methods for configuring high speed data acquisition
    Provides functions for peak triggering: slope, trigger level, hysteresis, etc
    Provides methods to initialize measurements for bus, int, ext triggering
Status
    Provides many get/set methods for configuring the operational, and questionable registers
Protection
    Channel protection, ovp, ocp, open sense, etc.  Unfinished    
DigitalIO
    Not started
'''


global_input_values = {}
class Device:
    def __init__(self, visa_addr='GPIB0::2::INSTR'):
        self._address = str(visa_addr)
        self._visa_driver = pyvisa.ResourceManager()
        self._bus = self._visa_driver.open_resource(self._address)
        self._bus.read_termination = '\n'
        self._bus.write_termination = '\n'
        # Extended timeout due to slow transfer of transient data
        # Make sure 10s timeout for GPIB adapter as well!!
        self._bus.read_timeout = 5000
        self._bus.timeout = 5000

        # Validate device model
        model = str(self._bus.query('*IDN?')).split(',')[1]
        global_input_values['model'] = model
        global_input_values['ch2'] = model in ('66319B', '66319D')
        global_input_values['dvm'] = model in \
            ('66321D', '66319D', '66309D')
        global_input_values['expanded_features'] = model in \
            ('66321B', '66321D', '66319B', '66319D')

        # Device class shortcuts
        self.com = Common(self._bus)
        self.display = Display(self._bus)
        self.format = Format(self._bus, 'ASC')
        self.ch1 = Channel(self._bus, '1')
        self.log = Log(self._bus)
        self.status = Status(self._bus)
        if global_input_values['ch2']:
            self.ch2 = Channel(self._bus, '2')

    def write(self, command):
        self._bus.write(command)

    def read(self):
        self._bus.read()

    def query(self, command):
        return self._bus.query(command)

    def read_raw(self):
        return self._bus.read_raw()

    def disconnect(self):
        self._bus.close()


class Common:
    def __init__(self, bus):
        self._bus = bus
        self._validate = ValidateRegister()
        self._command = Command(self._bus)

    # Clears event registers and errors
    def cls(self):
        write = "*CLS"
        self._command.write(write)

    # Read standard event enable register (no param)
    # Write with param
    def ese(self, reg_value=None):
        query = '*ESE?'
        write = '*ESE'
        return self._command.read_write(
            query, write, self._validate.register_8,
            reg_value)

    # Read and clear standard event enable register
    def esr(self):
        query = "*ESR?"
        return self._command.read(query)

    # Read instrument identification
    def idn(self):
        query = "*IDN?"
        return self._command.read(query)

    # Set the operation complete bit in the standard event register or queue
    # (param=1) places into output queue when operation complete
    def opc(self, reg_value=None):
        query = '*OPC?'
        write = '*OPC'
        return self._command.read_write(
            query, write, None, reg_value)

    # Returns the power supply to the saved setup (0...9)
    def rcl(self, preset_value=None):
        query = '*RCL?'
        write = '*RCL'
        return self._command.read_write(
            query, write, self._validate.preset,
            preset_value)

    # Returns the power supply to the *RST default conditions
    def rst(self):
        write = "*RST"
        self._command.write(write)
        self.cls()

    # Saves the present setup (1..9)
    def sav(self, preset_value=None):
        query = '*SAV?'
        write = '*SAV'
        return self._command.read_write(
            query, write, self._validate.preset,
            preset_value)

    # Programs the service request enable register
    def sre(self, reg_value=None):
        query = '*SRE?'
        write = '*SRE'
        return self._command.read_write(
            query, write, self._validate.register_8,
            reg_value)

    # Reads the status byte register
    def stb(self):
        query = "*STB?"
        return self._command.read(query)

    # command to trigger
    def trg(self):
        write = "*TRG"
        self._command.write(write)

    # Waits until all previous commands are executed
    def wait(self):
        write = "*WAI"
        self._command.write(write)

    # Perform self-tests
    def tst(self):
        query = "*TST"
        return self._command.read(query)


class Channel:
    def __init__(self, bus, channel: str):
        self._bus = bus
        self._validate = ValidateChannel()
        self._command = Command(self._bus)
        self._channel = channel
        self._ch_set = {}
        self._ch_set = {
            'output':   self.is_on(),
            'voltage': self.voltage(),
            'current': self.current(),
            'impedance': self.impedance(),
            'current_range': self.current_range(),
            'output_compensation': self.output_compensation()}
        self.values = {
            'device': global_input_values,
            'settings': self._ch_set}

        # Channel objects
        self.meas = Measure(self._bus, self._channel)
        self.prot = Protection(self._bus, self._channel)

    # ############################
    # Channel settings functions #
    # ############################

    # Turn channel output on
    def on(self):
        write = 'OUTP' + self._channel + ' ON'
        self._command.write(write)
        self._ch_set['output'] = self.is_on

    # Turn channel output off
    def off(self):
        write = 'OUTP' + self._channel + ' OFF'
        self._command.write(write)
        self._ch_set['output'] = self.is_on

    # get channel output state
    def is_on(self):
        query = 'OUTP' + self._channel + ':STAT?'
        return self._command.read(query)

    # resolution: 1mV
    def voltage(self, set_voltage=None):
        if self._channel == '1':
            query = 'VOLT?'
            write = 'VOLT'
        else:
            query = 'VOLT2?'
            write = 'VOLT2'
        self._validate.voltage(set_voltage, self._channel)
        return self._command.read_write(
            query, write, None,
            set_voltage, self._ch_set, 'voltage')

    # Sets current limit in Amps
    # resolution: 1mA
    def current(self, set_current=None):
        if self._channel == '1':
            query = 'CURR?'
            write = 'CURR'
        else:
            query = 'CURR?'
            write = 'CURR2'
        self._validate.current(set_current, self._channel)
        return self._command.read_write(
            query, write, None,
            set_current, self._ch_set, 'current')

    def current_range(self, set_current_range=None):
        if self._channel == '1':
            query = 'SENS:CURR:RANG?'
            write = 'SENS:CURR:RANG'
            return self._command.read_write(
                query, write, self._validate.current_range,
                set_current_range, self._ch_set, 'current_range')
        elif set_current_range is None:
            return 'MAX'

    def output_compensation(self, set_output_compensation=None):
        query = 'OUTP:COMP:MODE?'
        write = 'OUTP:COMP:MODE'
        return self._command.read_write(
            query, write, self._validate.output_compensation,
            set_output_compensation, self._ch_set, 'output_compensation')

    # Set output impedance for channel (Battery simulation)
    # -0.04 - 1 Ohms; 1 mOhm resolution
    def impedance(self, set_impedance=None):
        if global_input_values['expanded_features']:
            query = 'RES?'
            write = 'RES'
            return self._command.read_write(
                query, write, self._validate.impedance,
                set_impedance, self._ch_set, 'impedance')
        else:
            return 'NA'


class Display:
    def __init__(self, bus):
        self._bus = bus
        self._command = Command(self._bus)
        self._is_on = self.is_on()
        self._has_ch2 = global_input_values['ch2']

    # Enables or disables the LC Display
    def on(self):
        write = 'DISP ON'
        self._command.write(write)

    def off(self):
        write = 'DISP OFF'
        self._command.write(write)

    def is_on(self):
        query = 'DISP?'
        return self._command.read(query)

    def show_ch1(self):
        write = 'DISP:CHAN 1'
        self._command.write(write)

    def show_ch2(self):
        if self._has_ch2:
            write = 'DISP:CHAN 2'
            self._command.write(write)

    def set_text(self, text_msg):
        if isinstance(text_msg, str):
            self._bus.write('DISP:TEXT "' + text_msg + '"')
        else:
            raise TypeError('{}\n Valid:{}'.format(
                type(text_msg), str))

    def show_text(self):
        write = 'DISP:MODE TEXT'
        self._command.write(write)

    def show_normal(self):
        write = 'DISP:MODE NORM'
        self._command.write(write)


class Format:
    def __init__(self, bus, data='ASC', border='NORM'):
        self._bus = bus
        self._validate = ValidateFormat()
        self._command = Command(self._bus)
        self._format = {}
        self._format = {
            'data_format':   self.data_format(),
            'byte_order': self.byte_order()}
        self.values = {
            'device': global_input_values,
            'settings': self._format}
        self.data_format(data)
        self.byte_order(border)

    # Specifies the output data format for MEAS:ARR: ; FETC:ARR:
    def data_format(self, set_data_format=None):
        query = ':FORM:DATA?'
        write = ':FORM:DATA'
        return self._command.read_write(
            query, write, self._validate.data,
            set_data_format, self._format, 'data_format')

    # Specifies byte order for non ASCII output formats.
    def byte_order(self, set_byte_order=None):
        query = ':FORM:BORD?'
        write = ':FORM:BORD'
        return self._command.read_write(
            query, write, self._validate.border,
            set_byte_order, self._format, 'byte_order')


class Log:
    def __init__(self, bus):
        self._bus = bus
        self._validate = ValidateLog()
        self._command = Command(self._bus)
        self._log = {}
        self._log = {
            'sample_points':   self.sample_points(),
            'integration_time': self.integration_time(),
            'sample_offset': self.sample_offset()}
        self.values = {
            'device': global_input_values,
            'settings': self._log}
        self.log_data = {}

        # Log class objects
        self.trig = Trigger(self._bus)
        self.status = Status(self._bus)
        self.com = Common(self._bus)

    # #######################
    # Log setting functions #
    # #######################

    # Get the set number of sample points to log (arg=None)
    # Set the number of sample points to log
    def sample_points(self, set_sample_points=None):
        query = 'SENS:SWE:POIN?'
        write = 'SENS:SWE:POIN'
        return self._command.read_write(
            query, write, self._validate.sample_points,
            set_sample_points, self._log, 'sample_points')

    # Get the integration time for sampling (arg=None)
    # Set the integration time for sampling
    def integration_time(self, set_integration_time=None):
        query = 'SENS:SWE:TINT?'
        write = 'SENS:SWE:TINT'
        return self._command.read_write(
            query, write, self._validate.integration_time,
            set_integration_time, self._log, 'integration_time')

    # Get the sample (trigger) offset in seconds (arg=None)
    # Set the sample offset in seconds
    def sample_offset(self, set_sample_offset=None):
        query = 'SENS:SWE:OFFS:POIN?'
        write = 'SENS:SWE:OFFS:POIN'
        return self._command.read_write(
            query, write, self._validate.sample_offset,
            set_sample_offset, self._log, 'sample_offset')

    #def sample_to_csv(self):
     #   np.savetxt('agilent663XX_log.csv', self.log_data, delimiter=',')

    # Start a sample
    # All trigger and sample settings must be configured before running
    def start_meas_sample(self, bus_triggered=False):
        self.log_data = {}
        # Clear operational event status register
        self.status.get_opr_event_reg()
        # Save the operational prt/ntr/enable registers
        opr_ptr_reg = int(self.status.opr_ptr_reg())
        opr_ntr_reg = int(self.status.opr_ntr_reg())
        opr_enable_reg = int(self.status.opr_enable_reg())

        # WTG(wait_for_trig) goes high when measurement
        # trigger is initialized, and goes low when the
        # trigger activates. A negative transition event
        # indicates the device has triggered successfully
        wait_for_trig = 32

        # The bit in STB that is forwarded from the operational event register
        # An SRQ will be generated for this bit
        sre_status_bit = 128

        # Remove 'WTG' bit from PTR if present
        if opr_ptr_reg & wait_for_trig:
            self.status.opr_ptr_reg(opr_ptr_reg - wait_for_trig)

        # Add 'WTG' bit to NTR if not present
        if not opr_ntr_reg & wait_for_trig:
            self.status.opr_ntr_reg(opr_ntr_reg + wait_for_trig)

        # Set enable bit for WTG
        self.status.opr_enable_reg(wait_for_trig)

        # Enable service request for operation bit (OPR)
        self.com.sre(sre_status_bit)

        self.com.wait()
        # Initialize measurement trigger
        self.trig.initialize_meas_trig()

        # Calling the function with no parameters assumes an INT or EXT trigger source
        # passing True, will trigger an immediate BUS trigger (BUS must be the trigger_source)
        if bus_triggered:
            self.trig.generate_bus_trig()

        # Wait for SRQ
        self._bus.wait_for_srq(10000)
        self.com.sre(0)

        event_reg = int(self.status.get_opr_event_reg())
        if event_reg & wait_for_trig:
            if self.trig._trig['sense'] in '"CURR"':
                data = self._bus.query('FETC:ARR:CURR?')
                self.log_data['current'] = np.array(
                        data.split(','), dtype='f')
            elif self.trig._trig['sense'] in '"VOLT"':
                data = self._bus.query('FETC:ARR:VOLT?')
                self.log_data['voltage'] = np.array(
                        data.split(','), dtype='f')
            else:
                data = self._bus.query('FETC:ARR:DVM?')
                self.log_data['voltage'] = np.array(
                        data.split(','), dtype='f')
            self.log_data['seconds'] = np.arange(
                    0, float(self._log['integration_time']) *
                    float(self._log['sample_points']),
                    float(self._log['integration_time'])
            )
        else:
            print('Unknown error: event reg {}'.format(str(event_reg)))

        # Restore Registers
        self.status.opr_ptr_reg(opr_ptr_reg)
        self.status.opr_ntr_reg(opr_ntr_reg)
        self.status.opr_enable_reg(opr_enable_reg)


class Measure:
    def __init__(self, bus, channel):
        self._bus = bus
        self._channel = channel
        self._command = Command(self._bus)

    def __get_stat(self, meas_source: str, stat_type: str):
        query = 'MEAS:' + meas_source + ':' + stat_type + '?'
        return self._command.read(query)

    # ###############################
    # Channel measurement functions #
    # ###############################

    def voltage(self):
        if self._channel == '1':
            query = 'MEAS:VOLT?'
        else:
            query = 'MEAS:VOLT2?'
        return self._command.read(query)

    def current(self):
        if self._channel == '1':
            query = 'MEAS:CURR?'
        else:
            query = 'MEAS:CURR2?'
        return self._command.read(query)

    def power(self):
        volts = np.single(self.voltage())
        curr = np.single(self.current())
        return str(volts * curr)

    def current_low(self):
        return self.__get_stat('CURR', 'LOW')

    def current_high(self):
        return self.__get_stat('CURR', 'HIGH')

    def current_min(self):
        return self.__get_stat('CURR', 'MIN')

    def current_max(self):
        return self.__get_stat('CURR', 'MAX')

    def current_acdc(self):
        return self.__get_stat('CURR', 'ACDC')

    def voltage_low(self):
        return self.__get_stat('VOLT', 'LOW')

    def voltage_high(self):
        return self.__get_stat('VOLT', 'HIGH')

    def voltage_min(self):
        return self.__get_stat('VOLT', 'MIN')

    def voltage_max(self):
        return self.__get_stat('VOLT', 'MAX')

    def voltage_acdc(self):
        return self.__get_stat('VOLT', 'ACDC')

    def c(self):
        return self.current()

    def clow(self):
        return self.current_low()

    def chigh(self):
        return self.current_high()

    def crms(self):
        return self.current_acdc()

    def cmax(self):
        return self.current_max()

    def cmin(self):
        return self.current_min()

    def v(self):
        return self.voltage()

    def vlow(self):
        return self.voltage_low()

    def vhigh(self):
        return self.voltage_high()

    def vrms(self):
        return self.voltage_acdc()

    def vmax(self):
        return self.voltage_max()

    def vmin(self):
        return self.voltage_min()

    def p(self):
        return self.power()


class Trigger:
    def __init__(self, bus):
        self._bus = bus
        self._validate = ValidateTrigger()
        self._command = Command(self._bus)
        self._trig = {}
        self._trig = {
            'source':   self.source(),
            'sense': self.sense(),
            'current_level': self.current_level(),
            'current_hysteresis': self.current_hysteresis(),
            'current_slope': self.current_slope(),
            'current_count': self.current_count(),
            'voltage_level': self.voltage_level(),
            'voltage_hysteresis': self.voltage_hysteresis(),
            'voltage_slope': self.voltage_slope(),
            'voltage_count': self.voltage_count(),
            'dvm_level': self.dvm_level(),
            'dvm_hysteresis': self.dvm_hysteresis(),
            'dvm_slope': self.dvm_slope()}
        self.values = {
            'device': global_input_values,
            'settings': self._trig}

    def generate_bus_trig(self):
        write = 'TRIG:ACQ'
        self._command.write(write)

    def initialize_meas_trig(self):
        write = 'INIT:NAME ACQ'
        self._command.write(write)

    def front_panel_continuous_trig_on(self):
        write = 'INIT:CONT:SEQ1 ON'
        self._command.write(write)

    def front_panel_continuous_trig_off(self):
        write = 'INIT:CONT:SEQ1 OFF'
        self._command.write(write)

    def source(self, set_source=None):
        query = 'TRIG:ACQ:SOUR?'
        write = 'TRIG:ACQ:SOUR'
        return self._command.read_write(
            query, write, self._validate.source,
            set_source, self._trig, 'source')

    def sense(self, set_sense=None):
        val = self._validate.sense(set_sense)
        query = 'SENS:FUNC?'
        write = 'SENS:FUNC "' + str(val) + '"'
        if set_sense is None:
            return self._command.read(query)
        else:
            self._command.write(write)
            self._trig['settings'] = self._command.read(query)
            return None

    def current_count(self, set_current_count=None):
        query = 'TRIG:ACQ:COUN:CURR?'
        write = 'TRIG:ACQ:COUN:CURR'
        return self._command.read_write(
            query, write, self._validate.count,
            set_current_count, self._trig, 'current_count')

    def voltage_count(self, set_voltage_count=None):
        query = 'TRIG:ACQ:COUN:VOLT?'
        write = 'TRIG:ACQ:COUN:VOLT'
        return self._command.read_write(
            query, write, self._validate.count,
            set_voltage_count, self._trig, 'voltage_count')

    def current_hysteresis(self, set_current_hysteresis=None):
        query = 'TRIG:ACQ:HYST:CURR?'
        write = 'TRIG:ACQ:HYST:CURR'
        return self._command.read_write(
            query, write, self._validate.current,
            set_current_hysteresis, self._trig, 'current_hysteresis')

    def voltage_hysteresis(self, set_voltage_hysteresis=None):
        query = 'TRIG:ACQ:HYST:VOLT?'
        write = 'TRIG:ACQ:HYST:VOLT'
        return self._command.read_write(
            query, write, self._validate.voltage,
            set_voltage_hysteresis, self._trig, 'voltage_hysteresis')

    def dvm_hysteresis(self, set_dvm_hysteresis=None):
        if global_input_values['dvm']:
            query = 'TRIG:ACQ:HYST:DVM?'
            write = 'TRIG:ACQ:HYST:DVM'
            return self._command.read_write(
                query, write, self._validate.dvm,
                set_dvm_hysteresis, self._trig, 'dvm_hysteresis')
        else:
            return 'NA'

    def current_level(self, set_current_level=None):
        query = 'TRIG:ACQ:LEV:CURR?'
        write = 'TRIG:ACQ:LEV:CURR'
        return self._command.read_write(
            query, write, self._validate.current,
            set_current_level, self._trig, 'current_level')

    def voltage_level(self, set_voltage_level=None):
        query = 'TRIG:ACQ:LEV:VOLT?'
        write = 'TRIG:ACQ:LEV:VOLT'
        return self._command.read_write(
            query, write, self._validate.voltage,
            set_voltage_level, self._trig, 'voltage_level')

    def dvm_level(self, set_dvm_level=None):
        if global_input_values['dvm']:
            query = 'TRIG:ACQ:LEV:DVM?'
            write = 'TRIG:ACQ:LEV:DVM'
            return self._command.read_write(
                query, write, self._validate.dvm,
                set_dvm_level, self._trig, 'dvm_level')
        else:
            return 'NA'

    def current_slope(self, set_current_slope=None):
        query = 'TRIG:ACQ:SLOP:CURR?'
        write = 'TRIG:ACQ:SLOP:CURR'
        return self._command.read_write(
            query, write, self._validate.slope,
            set_current_slope, self._trig, 'current_slope')

    def voltage_slope(self, set_voltage_slope=None):
        query = 'TRIG:ACQ:SLOP:VOLT?'
        write = 'TRIG:ACQ:SLOP:VOLT'
        return self._command.read_write(
            query, write, self._validate.slope,
            set_voltage_slope, self._trig, 'voltage_slope')

    def dvm_slope(self, set_dvm_slope=None):
        if global_input_values['dvm']:
            query = 'TRIG:ACQ:SLOP:DVM?'
            write = 'TRIG:ACQ:SLOP:DVM'
            return self._command.read_write(
                query, write, self._validate.slope,
                set_dvm_slope, self._trig, 'dvm_slope')
        else:
            return 'NA'


class Status:
    def __init__(self, bus):
        self._bus = bus
        self.com = Common(self._bus)
        self._validate = ValidateRegister()
        self._command = Command(self._bus)

    def get_opr_event_reg(self):
        query = 'STAT:OPER?'
        return self._command.read(query)

    def get_opr_condition_reg(self):
        query = 'STAT:OPER:COND?'
        return self._command.read(query)

    def opr_enable_reg(self, reg_value=None):
        query = ':STAT:OPER:ENAB?'
        write = ':STAT:OPER:ENAB'
        return self._command.read_write(
            query, write, self._validate.register_16,
            reg_value)

    def opr_ptr_reg(self, reg_value=None):
        query = ':STAT:OPER:PTR?'
        write = ':STAT:OPER:PTR'
        return self._command.read_write(
            query, write, self._validate.register_16,
            reg_value)

    def opr_ntr_reg(self, reg_value=None):
        query = ':STAT:OPER:NTR?'
        write = ':STAT:OPER:NTR'
        return self._command.read_write(
            query, write, self._validate.register_16,
            reg_value)

    def get_ques_event_reg(self):
        query = 'STAT:QUES:EVEN?'
        return self._command.read(query)

    def get_ques_condition_reg(self):
        query = 'STAT:QUES:COND?'
        return self._command.read(query)

    def ques_enable_reg(self, reg_value=None):
        query = ':STAT:QUES:ENAB?'
        write = ':STAT:QUES:ENAB'
        return self._command.read_write(
            query, write, self._validate.register_16,
            reg_value)

    def ques_ptr_reg(self, reg_value=None):
        query = ':STAT:QUES:PTR?'
        write = ':STAT:QUES:PTR'
        return self._command.read_write(
            query, write, self._validate.register_16,
            reg_value)

    def ques_ntr_reg(self, reg_value=None):
        query = ':STAT:QUES:NTR?'
        write = ':STAT:QUES:NTR'
        return self._command.read_write(
            query, write, self._validate.register_16,
            reg_value)

    def reset_all_status_reg(self):
        write = ':STAT:PRES'
        self._command.write(write)

    def clear_error_queue(self):
        self.com.cls()

    def get_error_queue(self):
        query = ':SYST:ERR?'
        return self._command.read(query)


# @TODO Not completed
class Protection:
    def __init__(self, bus, channel):
        self._bus = bus
        self._channel = channel
        self._open_sense_protect = self.get_open_sense_protect()

    def set_open_sense_protect(self, on=False):
        self._open_sense_protect = on
        if on is True:
            self._bus.write('SENS:PROT:STAT ON')
        else:
            self._bus.write('SENS:PROT:STAT OFF')

    def get_open_sense_protect(self):
        return self._bus.query('SENS:PROT:STAT?')


# @TODO Not implemented
class DigitalIO:
    def __init__(self, bus):
        self._bus = bus


class Validate:

    def float_range(self):
        return lambda x, y: y[0] <= x <= y[1]

    def int_range(self):
        return lambda x, y: x in range(y[0], y[1] + 1)

    def find_element(self):
        return lambda x, y: x in y

    def error_text(self, warning_type, error_type):
        ansi_esc_seq = {'HEADER': '\033[95m',
                        'OKBLUE': '\033[94m',
                        'OKGREEN': '\033[92m',
                        'WARNING': '\033[93m',
                        'FAIL': '\033[91m',
                        'ENDC': '\033[0m',
                        'BOLD': '\033[1m',
                        'UNDERLINE': '\033[4m'
                        }
        return str(ansi_esc_seq[warning_type] + str(error_type) + ansi_esc_seq['ENDC'])

    def float_rng_and_str_tuples(self, validation_set, value, round_to):
        if isinstance(value, (float, int)):
            val = round(float(value), round_to)
            validator = self.float_range()
            if validator(val, validation_set[0]):
                return str(value)
            else:
                return ValueError('ValueError!\n'
                                  'Not in range:(float, int) {}\n'
                                  'or in set:(str) {}'.format(
                    validation_set[0],
                    validation_set[1]))
        elif isinstance(value, str):
            val = value.lower()
            validator = self.find_element()
            if validator(val, str(validation_set[1]).lower()):
                return val.upper()
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(str) {}\n'
                                  'or in range:(float, int) {}'.format(
                    validation_set[1],
                    validation_set[0]))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}, {}, {}'.format(
                type(value), int, float, str))

    def int_rng_and_str_tuples(self, validation_set, value):
        if isinstance(value, int):
            val = value
            validator = self.int_range()
            if validator(val, validation_set[0]):
                return str(value)
            else:
                return ValueError('ValueError!\n'
                                  'Not in range:(int) {}\n'
                                  'or in set:(str) {}'.format(
                    validation_set[0],
                    validation_set[1]))
        elif isinstance(value, str):
            val = value.lower()
            validator = self.find_element()
            if validator(val, str(validation_set[1]).lower()):
                return val.upper()
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(str) {}\n'
                                  'or in range:(int) {}'.format(
                    validation_set[1],
                    validation_set[0]))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}, {}'.format(
                type(value), int, str))

    def float_and_str_tuples(self, validation_set, value):
        if isinstance(value, (float, int)):
            validator = self.find_element()
            val = float(value)
            if validator(val, validation_set[0]):
                return str(value)
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(float, int) {}\n'
                                  'or in set:(str) {}'.format(
                    validation_set[0],
                    validation_set[1]))
        elif isinstance(value, str):
            val = value.lower()
            validator = self.find_element()
            if validator(val, str(validation_set[1]).lower()):
                return val.upper()
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(str) {}\n'
                                  'or in set:(float, str) {}'.format(
                    validation_set[1],
                    validation_set[0]))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}, {}, {}'.format(
                type(value), int, float, str))

    def int_and_str_tuples(self, validation_set, value):
        if isinstance(value, int):
            validator = self.find_element()
            val = float(value)
            if validator(val, validation_set[0]):
                return str(value)
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(int) {}\n'
                                  'or in set:(str) {}'.format(
                    validation_set[0],
                    validation_set[1]))
        elif isinstance(value, str):
            val = value.lower()
            validator = self.find_element()
            if validator(val, str(validation_set[1]).lower()):
                return val.upper()
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(str) {}\n'
                                  'or in set:(int) {}'.format(
                    validation_set[1],
                    validation_set[0]))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}, {}'.format(
                type(value), int, str))

    def float_rng_tuple(self, validation_set, value, round_to):
        if isinstance(value, (float, int)):
            val = round(float(value), round_to)
            validator = self.float_range()
            if validator(val, validation_set):
                return str(value)
            else:
                return ValueError('ValueError!\n'
                                  'Not in range:(float, int) {}'
                                  .format(validation_set))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}, {}'.format(
                type(value), int, float))

    def str_tuple(self, validation_set, value):
        if isinstance(value, str):
            val = value.lower()
            validator = self.find_element()
            if validator(val, str(validation_set).lower()):
                return val.upper()
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(str) {}'.format(
                    validation_set))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}'.format(
                type(value), str))

    def int_tuple(self, validation_set, value):
        if isinstance(value, int):
            val = value
            validator = self.find_element()
            if validator(val, validation_set):
                return str(val)
            else:
                return ValueError('ValueError!\n'
                                  'Not in set:(int) {}'.format(
                    validation_set))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}'.format(
                type(value), int))

    def int_rng_tuple(self, validation_set, value):
        if isinstance(value, int):
            val = value
            validator = self.int_range()
            if validator(val, validation_set):
                return str(val)
            else:
                return ValueError('ValueError!\n'
                                  'Not in range:(int) {}'.format(
                    validation_set))
        else:
            return TypeError('TypeError!\n'
                             'Received type: {}\n'
                             'Valid types: {}'.format(
                type(value), int))


class ValidateChannel(Validate):
    def __init__(self):
        super().__init__()

    def voltage(self, value, channel):
        if channel == '1':
            voltage_values = (0.0, 15.535), ('min', 'max')
        else:
            voltage_values = (0.0, 12.25), ('min', 'max')
        return self.float_rng_and_str_tuples(voltage_values, value, 3)

    def voltage_ch2(self, value):
        voltage_values = (0.0, 12.25), ('min', 'max')
        return self.float_rng_and_str_tuples(voltage_values, value, 3)

    def current(self, value, channel):
        if channel == '1':
            current_values = (0.0, 3.0712), ('min', 'max')
        else:
            current_values = (0.0, 1.52), ('min', 'max')
        return self.float_rng_and_str_tuples(current_values, value, 3)

    def impedance(self, value):
        impedance_values = (-0.4, 1.0), ('min', 'max')
        return self.float_rng_and_str_tuples(impedance_values, value, 3)

    def current_range(self, value):
        if global_input_values['expanded_features']:
            current_range_values = (3.0, 1.0, 0.02), ('min', 'max')
        else:
            current_range_values = (3.0, 0.02), ('min', 'max')
        return self.float_and_str_tuples(current_range_values, value)

    def measurement_interval(self, value):
        meas_interval_values = (0.0000156, 31200.0), ('max', 'min')
        return self.float_rng_and_str_tuples(meas_interval_values, value, 7)

    def output_compensation(self, value):
        if global_input_values['expanded_features']:
            output_compensation_values = ('llocal', 'hlocal', 'lremote', 'hremote')
        else:
            output_compensation_values = ('low', 'high')
        return self.str_tuple(output_compensation_values, value)

    def channel(self, value):
        channel_values = ('1', '2')
        return self.str_tuple(channel_values, value)


class ValidateRegister(Validate):
    def __init__(self):
        super().__init__()

    def register_8(self, value):
        register_values = (0, 128)
        return self.int_rng_tuple(register_values, value)

    def register_16(self, value):
        register_values = (0, 32767)
        return self.int_rng_tuple(register_values, value)

    def preset(self, value):
        preset_values = (0, 9)
        return self.int_rng_tuple(preset_values, value)


class ValidateFormat(Validate):
    def __init__(self):
        super().__init__()

    def data(self, value):
        data_values = ('ascii', 'asc', 'real')
        return self.str_tuple(data_values, value)

    def border(self, value):
        border_values = ('normal', 'norm',
                         'swapped', 'swap',
                         )
        return self.str_tuple(border_values, value)


class ValidateLog(Validate):
    def __init__(self):
        super().__init__()

    def sample_points(self, value):
        sample_length_values = (1, 4096), ('min', 'max')
        return self.int_rng_and_str_tuples(sample_length_values, value)

    def integration_time(self, value):
        sample_interval_values = (0.0000156, 31200.0), ('min', 'max')
        return self.float_rng_and_str_tuples(sample_interval_values, value, 7)

    def sample_offset(self, value):
        sample_offset_values = (-4095, 2000000000), ('min', 'max')
        return self.int_rng_and_str_tuples(sample_offset_values, value)


class ValidateTrigger(Validate):
    def __init__(self):
        super().__init__()

    def source (self, value):
        source_values = ('int', 'ext', 'bus')
        return self.str_tuple(source_values, value)

    def sense(self, value):
        if global_input_values['dvm']:
            sense_values = ('volt', 'curr', 'dvm')
        else:
            sense_values = ('volt', 'curr')
        return self.str_tuple(sense_values, value)

    def voltage(self, value):
        voltage_values = (0.0, 15.535), ('min', 'max')
        return self.float_rng_and_str_tuples(voltage_values, value, 3)

    def current(self, value):
        current_values = (0.0, 3.0712), ('min', 'max')
        return self.float_rng_and_str_tuples(current_values, value, 3)

    def dvm(self, value):
        dvm_values = (-4.5, 25.0), ('min', 'max')
        return self.float_rng_and_str_tuples(dvm_values, value, 3)

    def count(self, value):
        count_values = (1, 100), ('min', 'max')
        return self.int_rng_and_str_tuples(count_values, value)

    def slope(self, value):
        slope_values = ('pos', 'neg', 'eith')
        return self.str_tuple(slope_values, value)

    def timeout(self, value):
        timeout_values = (0.001, 60), ('inf', 'min', 'max',
                                       'def', 'default')
        return self.float_rng_and_str_tuples(timeout_values, value, 3)


class Command(Validate):
    def __init__(self, bus):
        super().__init__()
        self._bus = bus

    def read_write_old(self, query: str, write: str,
                       validator=None, value=None,
                       value_set=None, value_key=None):
        if value is None:
            return self._bus.query(query)
        else:
            if validator is not None:
                val = validator
                if isinstance(val, (ValueError, TypeError)):
                    print(self.error_text('WARNING', val))
                else:
                    self._bus.write(write)
                    if value_set is not None:
                        value_set[value_key] = self._bus.query(query)
                    else:
                        return None

            else:
                self._bus.write(write)
                if value_set is not None:
                    value_set[value_key] = self._bus.query(query)
                else:
                    return None

    def read_write(self, query: str, write: str,
                   validator=None, value=None,
                   value_dict=None, value_key=None):
        if value is None:
            return self._bus.query(query)
        else:
            if validator is not None:
                val = validator(value)
                if isinstance(val, (ValueError, TypeError)):
                    print(self.error_text('WARNING', val))
                else:
                    write = write + ' ' + str(value)
                    self._bus.write(write)
                    if value_dict is not None:
                        value_dict[value_key] = self._bus.query(query)
                    return None

            else:
                write = write + ' ' + str(value)
                self._bus.write(write)
                if value_dict is not None:
                    value_dict[value_key] = self._bus.query(query)
                return None

    def read_write_2arg(self, query: str, write: str,
                   validator=None, value=None,
                   value_dict=None, value_key=None):
        if value is None:
            return self._bus.query(query)
        else:
            if validator is not None:
                val = validator(value)
                if isinstance(val, (ValueError, TypeError)):
                    print(self.error_text('WARNING', val))
                else:
                    write = write + ' ' + str(value)
                    self._bus.write(write)
                    if value_dict is not None:
                        value_dict[value_key] = self._bus.query(query)
                    return None

            else:
                write = write + ' ' + str(value)
                self._bus.write(write)
                if value_dict is not None:
                    value_dict[value_key] = self._bus.query(query)
                return None

    def read(self, query: str):
        return self._bus.query(query)

    def write(self, write: str, validator=None):
        if validator is None:
            self._bus.write(write)
        else:
            val = validator
            if isinstance(val, (ValueError, TypeError)):
                print(self.error_text('WARNING', val))
            else:
                self._bus.write(write)