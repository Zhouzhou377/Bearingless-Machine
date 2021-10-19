import numpy as np
import sys
sys.path.insert(1, r'C:\Users\zwang2299\Zhouzhou_AMDC\AMDC-Firmware\scripts')
sys.path
#sys.path.append(r'..\AMDC-Firmware\scripts')

from AMDC import AMDC
import time

class cmd():
    
    def __init__(self, Vdc, amdc, debug = True):
        
        self.amdc = amdc
        self.debug = debug
        
        self.disconnect()
        self.connect()
        self.Vdc_init = Vdc
        self.setup(Vdc)
        self.begin_callback()
         
    def connect(self):
        self.amdc.connect()
        
    def disconnect(self):
        self.amdc.disconnect()
        
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, type, value, traceback):
        self.stop_callback()
        self.disconnect()
        
    def setup(self, Vdc):
        
        out = self.amdc.cmd(f'cabinet setup {Vdc:.5f}')
        if self.debug:
            return out
        
        
    def begin_callback(self):
        
        out = self.amdc.cmd('cabinet init_cb')
        if self.debug:
            return out
        
    def stop_callback(self):
        
        out = self.amdc.cmd('cabinet deinit_cb')
        if self.debug:
            return out
        
    def filter_signals(self, val = True):
        
        self.filter_current_signal(val)
        self.filter_position_signal(val)
        
    def filter_position_signal(self, val = True):
        
        out = self.amdc.cmd(f'cramb fil_pos {int(val)}')
        if self.debug:
            return out
    
    def filter_current_signal(self, val = True):
        
        out = self.amdc.cmd(f'cramb fil_cur {int(val)}')
        if self.debug:
            return out
        
    def enable_current_control(self, inverter):
        
        inverter = int(inverter)
        
        if inverter in range(1,7):
            out = self.amdc.cmd(f'cabinet enable_cc {inverter}')
            if self.debug:
                return out
            
    def disable_current_control(self, inverter):
        
        inverter = int(inverter)
        
        if inverter in range(1,7):
            out = self.amdc.cmd(f'cabinet dis_cc {inverter}')
            if self.debug:
                    return out
            
    def disable_all_current_control(self):
        
        out = self.amdc.cmd(f'cabinet dis_cc_all')
        if self.debug:
            return out
            
    def zero(self):
        
        #disables all current controllers, sets all pwm to zero
        #and resets PI controllers
        out = self.disable_all_current_control()
        if self.debug:
            return out
        
    def set_vdc(self, Vdc, inverter):
        
        inverter = int(inverter)
        
        if inverter in range(1,7):
            out = self.amdc.cmd(f'cabinet set_vdc {inverter} {Vdc:.5f}')
            if self.debug:
                return out
            
    def set_vdc_all(self, Vdc):
        
        out = self.amdc.cmd(f'cabinet set_vdc_all {Vdc:.5f}')
        if self.debug:
            return out
        
    
            
    def read_currents(self, inverter):
        
        inverter = int(inverter)
        
        if inverter in [1,2,3,4]:
            out = self.amdc.cmd(f'cabinet read_cur_1 {inverter}')
            currents = float(out[1])

        if inverter in [5,6]:
            out = self.amdc.cmd(f'cabinet read_cur_3 {inverter}')
            currents = [float(out[k]) for k in range(1,4)]
            
        return currents
            
    def pole_volts_3p(self, Va, Vb, Vc, inverter):
        
        inverter = int(inverter)
        
        if inverter in [5,6]:
            out = self.amdc.cmd(f'cabinet pv3 {inverter} {Va:.5f} {Vb:.5f} {Vc:.5f}')
            if self.debug:
                return out
            
    def phase_volts_3p(self, Va, Vb, Vc, inverter):
            
        inverter = int(inverter)
        
        if inverter in [5,6]:
            out = self.amdc.cmd(f'cabinet phv3 {inverter} {Va:.5f} {Vb:.5f} {Vc:.5f}')
            if self.debug:
                return out          
    
    def pole_volts_1p(self, Va, Vb, inverter):
        
        inverter = int(inverter)
        
        if inverter in [1,2,3,4]:
            out = self.amdc.cmd(f'cabinet pv1 {inverter} {Va:.5f} {Vb:.5f}')
            if self.debug:
                return out
            
    def phase_volts_1p(self, V, inverter):
        
        inverter = int(inverter)
        
        if inverter in [1,2,3,4]:
            out = self.amdc.cmd(f'cabinet phv1 {inverter} {V:.5f}')
            if self.debug:
                return out
            

    def enable_pwm(self):
        out = self.amdc.cmd('hw pwm on')
        if self.debug:
            return out

    def disable_pwm(self):
        out = self.amdc.cmd('hw pwm off')
        if self.debug:
            return out
    
    def levitate(self):

        out = self.amdc.cmd('cabinet enable')
        if self.debug:
            return out
        
    def stop_levitate(self):
        
        out = self.amdc.cmd('cabinet disable')
        if self.debug:
            return out 

    def openloop_vsi_3(self, inverter, freq, amp):

        out = self.amdc.cmd(f'cabinet openloop_vsi_3 {inverter} {freq:.6f} {amp:.6f}')
        if self.debug:
            return out
        
    def openloop_vsi_3_enable(self, inverter):
        
        out = self.amdc.cmd(f'cabinet openloop_vsi_3_enable  {inverter}')
        if self.debug:
            return out 

    def openloop_vsi_3_disable(self, inverter):
        
        out = self.amdc.cmd(f'cabinet openloop_vsi_3_disable  {inverter}')
        if self.debug:
            return out 
        
    def set_cc_sensor_cutoff_freq(self, fc, inverter):
        
        tau = (1/(2*np.pi*fc))*1000 #in ms not s
        
        out = self.amdc.cmd(f'cabinet cc_sen_tau {inverter} {tau:.6f}')
        if self.debug:
            return out
        
    def read_stats(self):
        
        out = self.amdc.cmd('cabinet stats print')
        if self.debug:
            return out
        
    def stats_reset(self):
        
        out = self.amdc.cmd('cabinet stats reset')
        if self.debug:
            return out

    def init_twin_ctrl(self):
        
        out = self.amdc.cmd('twin init')
        if self.debug:
            return out 

    def deinit_twin_ctrl(self):
        
        out = self.amdc.cmd('twin deinit')
        if self.debug:
            return out 
    def reset_twin_ctrl(self):
        
        out = self.amdc.cmd('twin reset')
        if self.debug:
            return out 
    
    def twin_sel_config(self, sel_config):
        
        out = self.amdc.cmd(f'twin sel_config {sel_config}')
        if self.debug:
            return out 

    def twin_set_vdc(self, vdc):
        
        out = self.amdc.cmd(f'twin set_vdc {vdc:.6f}')
        if self.debug:
            return out 

    def twin_enable_ctrl(self):
        
        out = self.amdc.cmd('twin enable_ctrl')
        if self.debug:
            return out 

    def enable_log(self):
        
        out = self.amdc.cmd('twin enable_log')
        if self.debug:
            return out 

    def twin_set_trq(self, i_d, i_q):
        
        out = self.amdc.cmd(f'twin set_trq {i_d:.6f} {i_q:.6f}')
        if self.debug:
            return out 
    
    def twin_set_s1(self, i_d, i_q):
        
        out = self.amdc.cmd(f'twin set_s1 {i_d:.6f} {i_q:.6f}')
        if self.debug:
            return out 

    def twin_set_s2(self, i_d, i_q):
        
        out = self.amdc.cmd(f'twin set_s2 {i_d:.6f} {i_q:.6f}')
        if self.debug:
            return out 

    def twin_set_freq(self, freq):
        
        out = self.amdc.cmd(f'twin set_freq {freq:.6f}')
        if self.debug:
            return out 

    def twin_disable_ctrl(self):
        
        out = self.amdc.cmd('twin disable_ctrl')
        if self.debug:
            return out 
    
    def twin_enable_testloop(self):
        out = self.amdc.cmd('twin enable_testloop')
        if self.debug:
            return out 

    def BIM_init(self):
        out = self.amdc.cmd('BIM init')
        if self.debug:
            return out
    
    def BIM_deinit(self):
        out = self.amdc.cmd('BIM deinit')
        if self.debug:
            return out

    def BIM_reset(self):
        out = self.amdc.cmd('BIM reset')
        if self.debug:
            return out
    
    def BIM_set_Vdc(self, vdc):
        out = self.amdc.cmd(f'BIM set_vdc {vdc:.6f}')
        if self.debug:
            return out
    def BIM_set_Vdc_tq(self, vdc):
        out = self.amdc.cmd(f'BIM set_vdc_tq {vdc:.6f}')
        if self.debug:
            return out
    def BIM_set_Vdc_s1(self, vdc):
        out = self.amdc.cmd(f'BIM set_vdc_s1 {vdc:.6f}')
        if self.debug:
            return out

    def BIM_enable_ctrl(self):
        out = self.amdc.cmd('BIM enable_ctrl')
        if self.debug:
            return out

    def BIM_enable_vctrl(self):
        out = self.amdc.cmd('BIM enable_vctrl')
        if self.debug:
            return out       

    def BIM_enable_levctrl(self):
        out = self.amdc.cmd('BIM enable_levctrl')
        if self.debug:
            return out 
    def BIM_enable_ob(self):
        out = self.amdc.cmd('BIM enable_ob')
        if self.debug:
            return out 
    def BIM_disable_vctrl(self):
        out = self.amdc.cmd('BIM disable_vctrl')
        if self.debug:
            return out       

    def BIM_disable_levctrl(self):
        out = self.amdc.cmd('BIM disable_levctrl')
        if self.debug:
            return out 
    def BIM_disable_ob(self):
        out = self.amdc.cmd('BIM disable_ob')
        if self.debug:
            return out 
            
    def BIM_set_w(self, w):
        out = self.amdc.cmd(f'BIM set_w {w:.6f}')
        if self.debug:
            return out 

    def BIM_set_Te(self, Te):
        out = self.amdc.cmd(f'BIM set_Te {Te:.6f}')
        if self.debug:
            return out 
    
    def BIM_set_id(self, id):
        out = self.amdc.cmd(f'BIM set_id {id:.6f}')
        if self.debug:
            return out 

    def BIM_set_deltaxy(self, deltax, deltay):
        out = self.amdc.cmd(f'BIM set_deltaxy {deltax:.6f} {deltay:.6f}')
        if self.debug:
            return out 

    def BIM_set_ixy(self, ix_ref, iy_ref):
        out = self.amdc.cmd(f'BIM set_ixy_ref {ix_ref:.6f} {iy_ref:.6f}')
        if self.debug:
            return out
    def BIM_set_Fxy(self, Fx_ref, Fy_ref):
        out = self.amdc.cmd(f'BIM set_Fxy_ref {Fx_ref:.6f} {Fy_ref:.6f}')
        if self.debug:
            return out
    
    def BIM_set_theta_offset(self, theta_offset):
        out = self.amdc.cmd(f'BIM set_theta_offset {theta_offset:.6f} ')
        if self.debug:
            return out

    def BIM_set_injection_Fxy(self, w, mag):
        out = self.amdc.cmd(f'BIM set_injection_Fxy {w:.6f} {mag:.6f}')
        if self.debug:
            return out
    def BIM_disable_injection(self):
        out = self.amdc.cmd('BIM disable_injection')
        if self.debug:
            return out 
    def BIM_disable(self):
        out = self.amdc.cmd('BIM disable_ctrl')
        if self.debug:
            return out 

  

if __name__ == "__main__":
        
    try:
        cmd.disconnect() #disconnect any previous crambs that may still be active
    except:
        Vdc = 30
        amdc = AMDC(port = 'COM13', cmdEcho = False)
        cmd = cmd(Vdc, amdc, debug = True)     
        
    