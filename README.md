1 Instructions for using Drone's sound code
1.1 main_quad is the source of the processor block

+core:Library folder

+Inc: no fild header

+main.h Library

+mpu6050.h IMU library + Kalman algorithm

+nRF24L01.h nRF24L01 control library

+Src: contains file soure

**+main.c Program Source**

+mpu6050.c Source IMU + Kalman algorithm for Drone

+nRF24L01.c Source nRF24L01 Control Drone

+Debug: Debug folder

+core : contains related files

+main_quad.elf: source will be generated after build (used to load programs and debug with cube IDE))

+main_quad.hex: source will be generated after build (only used to load stlink program)

1.2 control_stmf103 is the source of the control block

+core:Library folder

+Inc: no fild header

+main.h Library

+nRF24L01.h nRF24L01 control library

+Src: contains file soure

+main.c Program Source

+nRF24L01.c Source nRF24L01 Control Drone

+Debug: Debug folder

+core : contains related files

+main_quad.elf: source will be generated after build (used to load programs and debug with cube IDE))

+main_quad.hex: source will be generated after build (only used to load stlink program)

**2 In main.c**
********************** Main Program************************ ***

**1: Initialize #define**
 
 
Used to store the value received from the sensor is to process the output value Drone


3 jaws
                   HAL_GPIO_EXTI_Callback :External interrupt

                   set_val_for_nfr24 : Set the same nrf24 value (cast and process)

                   quad_up : Control the drone to fly up

                   quad_down : Control the Drone down

                   quad_right :Drone to the right

                   quad_left: Drone to the left

                   quad_front: Drone flying forward

                   quad_behind: The drone flies backwards

                   quad_giudocao : keep the drone at 1 height

                   hacanh_quad:emergency landing

                   quad_stop:Sudden stop

                   quad_start: Start the engine

                   quad_reset: Reset variables

                   calibrate_gyro: return the balance value for pmu6050 . IMU

                   correct_data_and_calibrate_3truc: Adjust the 3 axes to the correct coordinates and subtract to the default value

                   calculate_agl_roll_pitch : Calculates the angle and slip of 3 axes Roll, Pitch, Yaw


                   calculate_setpoint_pid:Calculate PID for all 3 axes

                   read_hc05: balance is complete, then mid-elevation (unfinished - because hc05 k meets the needs)

                   read_hc05_and_fillter: filter read value from IMU mpu6050 (LOW FILTER AND KALMAN) (2 FILTER WAYS)

                   check_looptime :SET PER PROCESSING TIME IS 4 MS

                   main : CENTRAL PROCESSING (SETTING INITIAL VALUE FOR SYSTEM)


4 while (1):PROCESSING LOCATION
        1. Read value from controller
        2. Read sensor value
        3. Adjust the 3 axes to the correct orbit
        4. Calculation of 3-axis angle
        5. Put the setting value of the angle into the Drone (if you want to balance, the angle of the 3 axes will be approximately 0)
        6. Read altitude sensor
        7. Filter what the operator wants to do (see in the report)
        8. Calculate PID
        9. Run motor program according to PID
        10. Check if it is enough to make 4ms (if not, wait)
 
        Read the full text of the while loop in the attached code
 
**5 Hardware Settings:**
         Open cubeIDE and choose to open this file: **.Cproject**
**6 Kalman and MPU6050**
         6.1 Soure :mpu6050.c
         6.2 Header: mpu6050.h (value of sensor used)
         6.3 Wings used:
1: declare before while to initialize the value: see section 2 of main
2: read and filter noise:
Read and use Kalman to filter out the noise (putting this value in place of the angle value is kalman filterable).
**7 Engines**
           7.1 Declare:
           7.2 Enter value:
           7.3 Engine run

**VIDEO fly:**


https://user-images.githubusercontent.com/89458703/227752750-f6cc4690-7326-44f0-b012-78029e7baa67.mp4


https://user-images.githubusercontent.com/89458703/227752757-7c7d1b00-2199-4672-9143-83ebd9aedcb8.mp4

done:


![481ed68a86335b6d0222](https://user-images.githubusercontent.com/89458703/227752763-4ef92b88-5658-42f1-a598-b891ab66bf89.jpg)

