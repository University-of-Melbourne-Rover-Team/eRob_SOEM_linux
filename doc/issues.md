# URT Issues Log
This file is used to keep a log of issues in URT EtherCAT maindevice.  

### #1.
ZeroErr eRob actuators always fail to enter OP mode on 1st attempt after Power On Reset (POR). This issue only occurs on the first attempt, rerunning the process clears this error, and never happens again until next POR.  
This is a known issue from the manufacturer. See link below.  
https://github.com/ZeroErrControl/eRob_SOEM_linux/issues/19  

### #2.
There exists a thread priority bug in the `/demo/` examples. The examples initially set the entire process priority to 99 (`/demo/eRob_CSV.cpp`, line 779). In lines 384-392, `erob_test()` calls:  
```C
osal_thread_create_rt(&ecatrt_thread, ...);
osal_thread_create(&ecat_check_thread, ...);
```
`osal_thread_create()` doesn't explicity set thread priority of `ecat_check_thread`, so it inherits priority=99. `osal_thread_create_rt()` explicitly sets thread priority to 40.  
When running `./build/demo/eRob_CSV`, and in another terminal run the below command.
```bash
$ ps -T -o tid,cls,rtprio,psr,comm -p $(pgrep -x eRob_CSV)
    TID CLS RTPRIO PSR COMMAND
  17474  FF     99   2 eRob_CSV
  17488  FF     40   2 eRob_CSV
  17489  FF     99   3 eRob_CSV
```
As seen above, `eRob_CSV` process has 3 threads under FIFO policy (`SCHED_FIFO`), with two threads at priority=99 (`main` and `ecat_check_thread`) and one at priority=40 (`ecatrt_thread`). The real-time thread should always have highest priority due to deterministic requirements of PDO exchange.  

This issue has been fixed in the `/src/` files. When running `./build/src/main`, and running same command in another terminal:
```bash
$ ps -T -o tid,cls,rtprio,psr,comm -p $(pgrep -x main)
    TID CLS RTPRIO PSR COMMAND
  20628  TS      -   2 main
  20641  FF     90   2 ecat_rt
  20642  FF     20   3 ecat_check
```
The process `./build/src/main` has three threads. The `main` thread is under time sharing scheduling policy (`SCHED_NORMAL`), and `ecat_rt` and `ecat_check` are under FIFO policy. with `ecat_rt` at a higher priority.  

### #3.
The `ecat_check` thread monitors the state of the EtherCAT bus. When EtherCAT communication is lost with a motor, or a motor is no longer is OP mode, the `ecat_check` thread attempts to recover it.  
Currently the `ecat_check` thread can recover a slave into SAFE-OP mode, but not OP mode.  
When running a test where a cable between motor 4 and 5 is physically unplugged, motors 5 and 6 enter `INIT->PRE-OP->SAFE-OP`, but never reenters `OP`.  
Potential cause:
 - While cable was unplugged, the motors Distributed Clocks (DC) became out of sync. The motor internal software may prevent motor from entering OP if its DC sync is within a tolerance.
 