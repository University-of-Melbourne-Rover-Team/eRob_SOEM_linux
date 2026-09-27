# URT EtherCAT Maindevice
CANopen over EtherCAT (CoE) maindevice for the rover's EtherCAT network.  
This URT EtherCAT maindevice uses the SOEM library to handle EtherCAT service and process data exchange.  
https://github.com/openethercatsociety/soem  
This implementation is adapted from the eRob_SOEM_Linux repository from ZeroErr.  
https://github.com/ZeroErrControl/eRob_SOEM_linux  
  
`./demo/` contains multiple independent programs which implement deomnstrations of CiA-402 control modes.  
`./src/main.cpp` contains the main program that runs on the URT rover's maindevice.  
> **Note:** This program works with Ubuntu Linux with real-time kernel capabilities. Compatability with other OS not been tested.

### URT maindevice usage
1. **Check the EtherCAT port network interface is configured correctly:**  

First run:  
```bash
ip link
```
To obtain network interface string of EtherCAT port (Usually `enx...` or `enp...`).  
In `main()`, check the `if_name` string matches the network interface of EtherCAT port.  
```c
const char *if_name = "enp...";
```

To verify linkstate, run:
```bash
ethtool <if_name>
```
Should output `Link detected: yes`

2. **Build executable:**
```bash
mkdir build  
cd build  
cmake ..  
make  
```
3. **Run executable:**  
```bash
sudo ./build/src/main
```

## eRob Demo Usage
### Running demo:

1. CSV mode:
```bash
sudo ./build/demo/eRob_CSV
```

2. Launch the position subscriber (PP):
```bash
sudo ./build/demo/eRob_PP_subscriber
python3 src/erob_ros/src/eCoder_fake.py
```

3. Launch the position subscriber (CSP):
```bash
sudo ./build/demo/eRob_CSP_subscriber
python3 src/erob_ros/src/eCoder_fake.py
``` 

4. Launch the cyclic synchronous position mode (CSP):
```bash
sudo ./build/demo/eRob_CSP
``` 

5. Launch the profile torque mode (PT):
In this mode, we can control the torque of the servo motor and have added PDO mapping to obtain the position, speed, torque, and status word of the servo motor. 

```bash
sudo ./build/demo/eRob_PT
``` 
If you want to consult the object dictionary, you can run the following command and then run `sudo ./build/test/linux/slaveinfo <ethercat_device> -map` to view the object dictionary.


6. Launch the cyclic synchronous torque mode (CST):
In this mode, we can control the torque of the servo motor and have added PDO mapping to obtain the position, speed, torque, and status word of the servo motor. 

```bash
sudo ./build/demo/eRob_CST
``` 
If you want to consult the object dictionary, you can run the following command and then run `sudo ./build/test/linux/slaveinfo <ethercat_device> -map` to view the object dictionary.

## Recommendations for EtherCAT Open-Source Master Users  

1. **Use a Real-Time Kernel System**  
   Ensure your operating system has a real-time kernel to guarantee consistent and precise communication.

2. **Isolate CPU Cores**  
   Perform CPU isolation to dedicate specific cores to EtherCAT processes, reducing interruptions and improving stability.

3. **Troubleshooting OP State Issues**  
   - Failure to enter OP state may be caused by errors in the **object dictionary mapping** or improper configuration of **DC (Distributed Clock) mode**.  
   - **eRob** only supports **DC mode**, and proper configuration of DC mode is crucial for system synchronization and precision.

4. **Read Mode-Specific Instructions**  
   Before using each mode, read the relevant operational instructions to ensure correct configuration and usage.

5. **Use the Official eRob Upper Computer Software**  
   eRob provides official upper computer software. Mastering the built-in **oscilloscope tool** will allow you to quickly locate issues with the EtherCAT master.

6. **Capture and Analyze EtherCAT Data**  
   Use packet capture tools to analyze EtherCAT output and log information to identify errors.
