# URT EtherCAT Maindevice Progress Log
This file is used to keep a progress log of URT EtherCAT maindevice.  

**20/09/26 Log**  
Updates:
- 6 motors working in CSV reliably since updating CiA 402 state machine control
- 6 motors working in PV reliably

Changes include:  
- maindevice waits for all slaves to reach the same state before issuing new control word
- does this by only incrementing the `step` variable if all slaves are at the same state

Issues:  
- Tried increasing the target velocity:
    - In CSV mode, the motor applies the target velocity straight away, it does not do internal ramping using profile accel/decel values
    - this means we have to implement the accel/decel of the motor
    - when the target velocity is too high, couple motors errored (most likely due to exceeding the max acceleration threshold)

To do:
- Fix state machine control so that the control word is issued per drive based on its status word, instead of when all motors reach same state
- Handle faults - at the moment when a fault occurs, there is no code to bring the motor back to operation enable state
- tune max accel/decel/velocity parameters to acceptable values