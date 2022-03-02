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
        
        #self.disconnect()
        #self.connect()
        self.cabinet_init()
        self.Vdc_init = Vdc
        self.set_vdc_all(Vdc)
        
         
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
        
        
    def cabinet_init(self):
        
        out = self.amdc.cmd('cabinet init_cb')
        if self.debug:
            return out
        
    def cabinet_deinit(self):
        
        out = self.amdc.cmd('cabinet deinit_cb')
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

    def init_c_loop_ctrl(self):
        
        out = self.amdc.cmd('c_loop init')
        if self.debug:
            return out 

    def deinit_c_loop_ctrl(self):
        
        out = self.amdc.cmd('c_loop deinit')
        if self.debug:
            return out 
    def reset_c_loop_ctrl(self):
        
        out = self.amdc.cmd('c_loop reset')
        if self.debug:
            return out 
    
    def c_loop_sel_config(self, sel_config):
        
        out = self.amdc.cmd(f'c_loop sel_config {sel_config}')
        if self.debug:
            return out 

    def c_loop_set_vdc(self, vdc):
        
        out = self.amdc.cmd(f'c_loop set_vdc {vdc:.6f}')
        if self.debug:
            return out 

    def c_loop_enable_ctrl(self):
        
        out = self.amdc.cmd('c_loop enable_ctrl')
        if self.debug:
            return out 

    def enable_log(self):
        
        out = self.amdc.cmd('c_loop enable_log')
        if self.debug:
            return out 

    def c_loop_set_trq(self, i_d, i_q):
        
        out = self.amdc.cmd(f'c_loop set_trq {i_d:.6f} {i_q:.6f}')
        if self.debug:
            return out 
    
    def c_loop_set_s1(self, i_d, i_q):
        
        out = self.amdc.cmd(f'c_loop set_s1 {i_d:.6f} {i_q:.6f}')
        if self.debug:
            return out 

    def c_loop_set_s2(self, i_d, i_q):
        
        out = self.amdc.cmd(f'c_loop set_s2 {i_d:.6f} {i_q:.6f}')
        if self.debug:
            return out 

    def c_loop_set_freq(self, freq):
        
        out = self.amdc.cmd(f'c_loop set_freq {freq:.6f}')
        if self.debug:
            return out 

    def c_loop_disable_ctrl(self):
        
        out = self.amdc.cmd('c_loop disable_ctrl')
        if self.debug:
            return out 
    
    def c_loop_enable_testloop(self):
        out = self.amdc.cmd('c_loop enable_testloop')
        if self.debug:
            return out 
#BP3 CMD
    def bp3_init(self):
        out = self.amdc.cmd('bp3 init')
        if self.debug:
            return out
    
    def bp3_deinit(self):
        out = self.amdc.cmd('bp3 deinit')
        if self.debug:
            return out

    def bp3_reset(self):
        out = self.amdc.cmd('bp3 reset')
        if self.debug:
            return out
    
    def bp3_set_Vdc(self, vdc):
        out = self.amdc.cmd(f'bp3 set_vdc {vdc:.6f}')
        if self.debug:
            return out
    def bp3_set_Vdc_tq(self, vdc):
        out = self.amdc.cmd(f'bp3 set_vdc_tq {vdc:.6f}')
        if self.debug:
            return out
    def bp3_set_Vdc_s1(self, vdc):
        out = self.amdc.cmd(f'bp3 set_vdc_s1 {vdc:.6f}')
        if self.debug:
            return out

    def bp3_enable_ctrl(self):
        out = self.amdc.cmd('bp3 enable_ctrl')
        if self.debug:
            return out

    def bp3_enable_align(self):
        out = self.amdc.cmd('bp3 enable_align')
        if self.debug:
            return out

    def bp3_enable_vctrl(self):
        out = self.amdc.cmd('bp3 enable_vctrl')
        if self.debug:
            return out       

    def bp3_enable_levctrl(self):
        out = self.amdc.cmd('bp3 enable_levctrl')
        if self.debug:
            return out 
    def bp3_enable_ob(self):
        out = self.amdc.cmd('bp3 enable_ob')
        if self.debug:
            return out 
    def bp3_disable_vctrl(self):
        out = self.amdc.cmd('bp3 disable_vctrl')
        if self.debug:
            return out       

    def bp3_disable_levctrl(self):
        out = self.amdc.cmd('bp3 disable_levctrl')
        if self.debug:
            return out 
    def bp3_disable_ob(self):
        out = self.amdc.cmd('bp3 disable_ob')
        if self.debug:
            return out 
            
    def bp3_set_w(self, w):
        out = self.amdc.cmd(f'bp3 set_w {w:.6f}')
        if self.debug:
            return out 

    def bp3_set_Te(self, Te):
        out = self.amdc.cmd(f'bp3 set_Te {Te:.6f}')
        if self.debug:
            return out 
    
    def bp3_set_id(self, id):
        out = self.amdc.cmd(f'bp3 set_id {id:.6f}')
        if self.debug:
            return out 

    def bp3_set_deltaxy(self, deltax, deltay):
        out = self.amdc.cmd(f'bp3 set_deltaxy {deltax:.8f} {deltay:.8f}')
        if self.debug:
            return out 

    def bp3_set_ixy(self, ix_ref, iy_ref):
        out = self.amdc.cmd(f'bp3 set_ixy_ref {ix_ref:.6f} {iy_ref:.6f}')
        if self.debug:
            return out
    def bp3_set_Fxy(self, Fx_ref, Fy_ref):
        out = self.amdc.cmd(f'bp3 set_Fxy_ref {Fx_ref:.6f} {Fy_ref:.6f}')
        if self.debug:
            return out
    
    def bp3_set_theta_offset(self, theta_offset):
        out = self.amdc.cmd(f'bp3 set_theta_offset {theta_offset:.6f} ')
        if self.debug:
            return out

    def bp3_set_injection_Fxy(self, w, mag):
        out = self.amdc.cmd(f'bp3 set_injection_Fxy {w:.6f} {mag:.6f}')
        if self.debug:
            return out
    def bp3_disable_injection(self):
        out = self.amdc.cmd('bp3 disable_injection')
        if self.debug:
            return out 
    def bp3_disable(self):
        out = self.amdc.cmd('bp3 disable_ctrl')
        if self.debug:
            return out 

# BIM CMD
    def bim_init(self):
        out = self.amdc.cmd('bim init')
        if self.debug:
            return out
    
    def bim_deinit(self):
        out = self.amdc.cmd('bim deinit')
        if self.debug:
            return out

    def bim_reset(self):
        out = self.amdc.cmd('bim reset')
        if self.debug:
            return out
    
    def bim_set_Vdc(self, vdc):
        out = self.amdc.cmd(f'bim set_vdc {vdc:.6f}')
        if self.debug:
            return out
    def bim_set_Vdc_tq(self, vdc):
        out = self.amdc.cmd(f'bim set_vdc_tq {vdc:.6f}')
        if self.debug:
            return out
    def bim_set_Vdc_s1(self, vdc):
        out = self.amdc.cmd(f'bim set_vdc_s1 {vdc:.6f}')
        if self.debug:
            return out

    def bim_enable_ctrl(self):
        out = self.amdc.cmd('bim enable_ctrl')
        if self.debug:
            return out

    def bim_enable_vctrl(self):
        out = self.amdc.cmd('bim enable_vctrl')
        if self.debug:
            return out       

    def bim_enable_levctrl(self):
        out = self.amdc.cmd('bim enable_levctrl')
        if self.debug:
            return out 
    def bim_enable_ob(self):
        out = self.amdc.cmd('bim enable_ob')
        if self.debug:
            return out 
    def bim_disable_vctrl(self):
        out = self.amdc.cmd('bim disable_vctrl')
        if self.debug:
            return out       

    def bim_disable_levctrl(self):
        out = self.amdc.cmd('bim disable_levctrl')
        if self.debug:
            return out 
    def bim_disable_ob(self):
        out = self.amdc.cmd('bim disable_ob')
        if self.debug:
            return out 
            
    def bim_set_w(self, w):
        out = self.amdc.cmd(f'bim set_w {w:.6f}')
        if self.debug:
            return out 

    def bim_set_Te(self, Te):
        out = self.amdc.cmd(f'bim set_Te {Te:.6f}')
        if self.debug:
            return out 
    
    def bim_set_id(self, id):
        out = self.amdc.cmd(f'bim set_id {id:.6f}')
        if self.debug:
            return out 

    def bim_set_deltaxy(self, deltax, deltay):
        out = self.amdc.cmd(f'bim set_deltaxy {deltax:.8f} {deltay:.8f}')
        if self.debug:
            return out 

    def bim_set_ixy(self, ix_ref, iy_ref):
        out = self.amdc.cmd(f'bim set_ixy_ref {ix_ref:.6f} {iy_ref:.6f}')
        if self.debug:
            return out
    def bim_set_Fxy(self, Fx_ref, Fy_ref):
        out = self.amdc.cmd(f'bim set_Fxy_ref {Fx_ref:.6f} {Fy_ref:.6f}')
        if self.debug:
            return out
    
    def bim_set_theta_offset(self, theta_offset):
        out = self.amdc.cmd(f'bim set_theta_offset {theta_offset:.6f} ')
        if self.debug:
            return out

    def bim_set_injection_Fxy(self, w, mag):
        out = self.amdc.cmd(f'bim set_injection_Fxy {w:.6f} {mag:.6f}')
        if self.debug:
            return out
    def bim_disable_injection(self):
        out = self.amdc.cmd('bim disable_injection')
        if self.debug:
            return out 
    def bim_disable(self):
        out = self.amdc.cmd('bim disable_ctrl')
        if self.debug:
            return out 
    
  

if __name__ == "__main__":
        
    try:
        cmd.disconnect() #disconnect any previous crambs that may still be active
    except:
        Vdc = 30
        amdc = AMDC(port = 'COM13', cmdEcho = False)
        cmd = cmd(Vdc, amdc, debug = True)     
        
    