/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float Max_PWM  = 1800;
float Max_THR  = 1450;
float Min_PWM  = 1050;
#define thr_start  1300
#define speed_up_down  0.1
float throttle  = 1000;
float throttle_PID  = 1000;
float pid_throttle  = 0;
float  acc_total_vector = 4250;
#define speed_LRFB 100
#define KP_xy  0.92		//0.88
#define KI_xy  0.022		//0.02
#define KD_xy  16		//15
#define KP_z  3.2		//3
#define KI_z  0.019		//0.018
#define KD_z  0
#define KP_dc  0.2
#define KI_dc  0.001//0.001
#define KD_dc  2//3
float Setting_docao = 150.0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint16_t PWM[4]={1000,1000,1000,1000};
NRF24L01_config_TypeDef nrf_rx_cfg;
uint8_t data_rx_real = 0 ;
uint8_t button_press = 0;
uint8_t old_data_rx_real = 0;
uint16_t count_data_rx_real = 0;
MPU6050_t MPU6050_Data;
PIDSingle PID_ROLL;
PIDSingle PID_PITCH;
PIDSingle PID_YAW;
PIDSingle PID_HG;
uint32_t loop_time = 0;
uint32_t loop_time_hacanh = 0;
uint32_t timer_before = 0;
float ax_cal_tt=0,ay_cal_tt=0;
float gyro_roll_cal_tt =0,gyro_pitch_cal_tt= 0,gyro_yaw_cal_tt= 0;
float angle_roll_acc =0, angle_pitch_acc=0, angle_pitch=0, angle_roll=0;
float pitch_level_adjust = 0,roll_level_adjust = 0;  //Set the pitch angle correction to zero.
float pid_pitch_setpoint = 0,pid_roll_setpoint = 0,pid_yaw_setpoint = 0;  //Set point
// INIT kh value
uint32_t IC_Val = 0;
float_t Distance  = 0;
float_t Kalman_Distance = 5;
uint8_t ok = 0;
uint8_t first_read_hc = 0;
bool loi_xxx = false;
bool is_running = false;
bool is_giudocao = false;
bool hacanh = false;
bool firt_start = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 	bool status_ex = 0;
	status_ex = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin);
  if(GPIO_Pin == GPIO_PIN_1){
	  if(status_ex == true)
	  {
		  __HAL_TIM_SET_COUNTER(&htim2,0);
		  HAL_TIM_Base_Start(&htim2);
		  ok = 1;
	  }
	  else if(status_ex == false)
	  {
		  IC_Val = __HAL_TIM_GET_COUNTER(&htim2);
		  Distance = (float)IC_Val * 0.034/2;
		  ok = 2;
	  }
  }
}
void set_val_for_nfr24(NRF24L01_config_TypeDef* nfr24_dummy)
{
	  HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	  	 //nrf_rx_cfg.RX_TX           = 1; //RX
	  nfr24_dummy->CE_pin          = CE_pin_Pin;
	  nfr24_dummy->CE_port         = CE_pin_GPIO_Port;
	  nfr24_dummy->CSN_pin         = CSN_pin_Pin;
	  nfr24_dummy->CSN_port        = CSN_pin_GPIO_Port;
	  nfr24_dummy->SPI             = &hspi1;
	  nfr24_dummy->radio_channel   = 15;
	  nfr24_dummy->baud_rate       = TM_NRF24L01_DataRate_1M;
	  nfr24_dummy->payload_len     = 1;
	  nfr24_dummy->crc_len         = 1;
	  nfr24_dummy->output_power    = TM_NRF24L01_OutputPower_0dBm;
	  nfr24_dummy->rx_address[ 0 ] = 0x7E;
	  nfr24_dummy->rx_address[ 1 ] = 0x7E;
	  nfr24_dummy->rx_address[ 2 ] = 0x7E;
	  nfr24_dummy->rx_address[ 3 ] = 0x7E;
	  nfr24_dummy->rx_address[ 4 ] = 0x7E;
	  nfr24_dummy->tx_address[ 0 ] = 0xE7;
	  nfr24_dummy->tx_address[ 1 ] = 0xE7;
	  nfr24_dummy->tx_address[ 2 ] = 0xE7;
	  nfr24_dummy->tx_address[ 3 ] = 0xE7;
	  nfr24_dummy->tx_address[ 4 ] = 0xE7;
}
void quad_up(){
	throttle = throttle + speed_up_down ;
	if(throttle < Min_PWM){throttle = Min_PWM;}
	else if(throttle > Max_THR){throttle = Max_THR;}
}
void quad_down(){
	throttle = throttle - speed_up_down*1.5 ;
	if(throttle > Max_THR){throttle = Max_THR;}
}
void quad_right(){
	pid_pitch_setpoint= 0;
	pid_roll_setpoint = -speed_LRFB;
}
void quad_left(){
	pid_pitch_setpoint = 0;
	pid_roll_setpoint = speed_LRFB;
}
void quad_front(){
	pid_pitch_setpoint = speed_LRFB;
	pid_roll_setpoint = 0;
}
void quad_behind(){
	pid_pitch_setpoint = -speed_LRFB;
	pid_roll_setpoint = 0;
}
void quad_giudocao(){
	is_giudocao  = true;
	Setting_docao = 150.0;
	throttle = throttle_PID;
	Reset_PID_Integrator(&PID_HG);
}
void hacanh_quad(){
	loop_time_hacanh = HAL_GetTick();
	throttle = throttle_PID;
	hacanh =true;
	is_giudocao  = false;
}
void quad_stop(){
	is_running = false;
	throttle  = 1000;
	//Reset all
	Reset_PID_Integrator(&PID_ROLL);
	Reset_PID_Integrator(&PID_PITCH);
	Reset_PID_Integrator(&PID_YAW);
	Reset_PID_Integrator(&PID_HG);
}
void quad_start(){
	is_running = true;
	throttle  = thr_start;
	angle_pitch = angle_pitch_acc;
	angle_roll = angle_roll_acc;
	//Reset all
	Reset_PID_Integrator(&PID_ROLL);
	Reset_PID_Integrator(&PID_PITCH);
	Reset_PID_Integrator(&PID_YAW);
	Reset_PID_Integrator(&PID_HG);
}
void quad_reset(){
PWM[0] = 1000;PWM[1] = 1000;
PWM[2] = 1000;PWM[3] = 1000;
}
void calibrate_gyro() {
	uint16_t cal_int = 0;
	gyro_roll_cal_tt = 0;
	gyro_pitch_cal_tt = 0;
	gyro_yaw_cal_tt = 0;
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.                  //Change the led status every 125 readings to indicate calibration.
      MPU6050_Read_All(&hi2c2, &MPU6050_Data);                                                                //Read the gyro output.
      ax_cal_tt += (float)MPU6050_Data.Accel_X_RAW;
      ay_cal_tt += (float)MPU6050_Data.Accel_Y_RAW;
      gyro_roll_cal_tt += (float)MPU6050_Data.Gyro_X_RAW;                                                     //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal_tt += (float)MPU6050_Data.Gyro_Y_RAW;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal_tt += (float)MPU6050_Data.Gyro_Z_RAW;                                                       //Ad yaw value to gyro_yaw_cal.
    }                                                                  //Set output PB3 low.
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
//  ax_cal_tt  /= 2000;
//	ay_cal_tt  /= 2000;
//	gyro_roll_cal_tt /= 2000;              //Divide the roll total by 2000.
//  gyro_pitch_cal_tt /= 2000;             //Divide the pitch total by 2000.
//  gyro_yaw_cal_tt /= 2000;                //Divide the yaw total by 2000.
    ax_cal_tt = 305;
    ay_cal_tt = 170;
	gyro_roll_cal_tt = -248;
	gyro_pitch_cal_tt = -142;
	gyro_yaw_cal_tt = -48;
}

void correct_data_and_calibrate_3truc(){

	MPU6050_Data.G_roll	 = 	(float)(MPU6050_Data.Gyro_X_RAW - gyro_roll_cal_tt	) ;
	MPU6050_Data.G_pitch = 	(float)(MPU6050_Data.Gyro_Y_RAW - gyro_pitch_cal_tt	)*(-1);
	MPU6050_Data.G_yaw 	 = 	(float)(MPU6050_Data.Gyro_Z_RAW - gyro_yaw_cal_tt	)*(-1);

	MPU6050_Data.A_pitch 	= (float)(MPU6050_Data.Accel_X_RAW - ax_cal_tt)	*(1);
	MPU6050_Data.A_roll 	= (float)(MPU6050_Data.Accel_Y_RAW - ay_cal_tt)	*(-1);
	MPU6050_Data.A_yaw 		= (float)(MPU6050_Data.Accel_Z_RAW)	*(-1);

}

void calculate_agl_roll_pitch(){
	//bo loc
	MPU6050_Data.Gx_roll  = MPU6050_Data.Gx_roll*0.8	+(float)(MPU6050_Data.G_roll /65.5)*0.2;//Gyro pid input is deg/sec.
	MPU6050_Data.Gy_pitch = MPU6050_Data.Gy_pitch*0.8	+(float)(MPU6050_Data.G_pitch /65.5)*0.2;//Gyro pid input is deg/sec.
	MPU6050_Data.Gz_yaw   = MPU6050_Data.Gz_yaw*0.8		+(float)(MPU6050_Data.G_yaw /65.5)*0.2;//Gyro pid input is deg/sec.

	angle_roll  +=  MPU6050_Data.G_roll  * 0.00007634;
	angle_pitch +=  MPU6050_Data.G_pitch  * 0.00007634;   //Calculate the traveled pitch angle and add this to the angle_pitch variable.

	angle_pitch -= angle_roll * sin(MPU6050_Data.G_yaw * 0.000001332);//If the IMU has yawed transfer the roll angle to the pitch angel.
	angle_roll  += angle_pitch* sin(MPU6050_Data.G_yaw * 0.000001332);//If the IMU has yawed transfer the pitch angle to the roll angel.

		//Tinh toan goc
	acc_total_vector = sqrt((MPU6050_Data.A_roll * MPU6050_Data.A_roll) +
			(MPU6050_Data.A_pitch * MPU6050_Data.A_pitch) + (MPU6050_Data.A_yaw * MPU6050_Data.A_yaw));
	if(abs(MPU6050_Data.A_roll) < acc_total_vector)
		{angle_roll_acc = asin(MPU6050_Data.A_roll/acc_total_vector)*(-57.296);	}	//(0 ->90)
	if(abs(MPU6050_Data.A_pitch) < acc_total_vector)
		{angle_pitch_acc = asin(MPU6050_Data.A_pitch/acc_total_vector)* (57.296);}	//(0 ->90)
	angle_pitch_acc -= (0);
	angle_roll_acc -=  (0);
	angle_pitch = angle_pitch * 0.9995 + angle_pitch_acc * 0.0005;
	angle_roll = angle_roll * 0.9995 + angle_roll_acc * 0.0005;
}
void calculate_setpoint_pid(){
	//Set point roll pitch yal
	pid_roll_setpoint  = 0;
	pid_pitch_setpoint = 0;
	pid_yaw_setpoint = 0;
	pitch_level_adjust = angle_pitch * 15 ;
	roll_level_adjust =  angle_roll * 15;
	if(data_rx_real != 0 && ((data_rx_real & 0b11111) == 0)){
			if(data_rx_real == 0x20){		quad_front();	}
			else if(data_rx_real == 0x40){	quad_right();	}
			else if(data_rx_real == 0x60){	quad_behind();	}
			else if(data_rx_real == 0x80){	quad_left();	}
			else if(data_rx_real == 0x10){	quad_stop();	}
	}
	else{
	}
	pid_roll_setpoint  -= roll_level_adjust;
	pid_pitch_setpoint -= pitch_level_adjust;
	pid_roll_setpoint  /= 3.0;
	pid_pitch_setpoint /= 3.0;
}
void read_hc05(){
	HAL_GPIO_WritePin(GPIOB, hc_trigger_pin_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, hc_trigger_pin_Pin, GPIO_PIN_RESET);
}
void read_hc05_and_fillter(){
	if(first_read_hc == 0){
		read_hc05();
		Kalman_Distance =Kalman_Distance*0.8 + Distance*0.2;
		if(Kalman_Distance <4 )		{Kalman_Distance =4;}
		if(Kalman_Distance >450 )	{Kalman_Distance =450; }
		first_read_hc++;
	}
	else if(first_read_hc >= 1){
		first_read_hc = 0;
	}
}
void check_looptime(){
	//check loop time for 1 time calculate
	loop_time =(HAL_GetTick() - timer_before) ;
	if(loop_time > 5){loi_xxx = 1;}
 	while(loop_time <= 3){
		loop_time =(HAL_GetTick() - timer_before);
	}
	timer_before = HAL_GetTick();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 	Int_PID_Integrator(&PID_ROLL,	KP_xy	,	KI_xy	,	KD_xy	);
	Int_PID_Integrator(&PID_PITCH,	KP_xy	,	KI_xy	,	KD_xy	);
	Int_PID_Integrator(&PID_YAW,	KP_z	,	KI_z	,	KD_z	);
	Int_PID_Integrator(&PID_HG,		KP_dc	,	KI_dc	,	KD_dc	);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
  while(MPU6050_Data.MPU6050_address != 0x68){
	  MPU6050_Data.MPU6050_address =MPU6050_Init(&hi2c2);
	  HAL_Delay(50);
  }
  set_val_for_nfr24(&nrf_rx_cfg);
  mbal_NRF24L01_Init(&nrf_rx_cfg );
  HAL_Delay(100);
  // Innit value for NRF24
MPU6050_Data.A_roll = 0;
MPU6050_Data.A_pitch = 0;
MPU6050_Data.A_yaw = 0;
  MPU6050_Data.KalmanAngleX = 0;
  MPU6050_Data.KalmanAngleY = 0;
  MPU6050_Data.KalmanAngleZ = 0;
  MPU6050_Data.Gx_roll  = 0;
  MPU6050_Data.Gy_pitch =0;
  MPU6050_Data.Gz_yaw   =0;
  HAL_Delay(100);
  // Innit value for MORTOR
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1000);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1000);
  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1000);
  //get first value
  calibrate_gyro();
  timer_before = HAL_GetTick();//get time when starting
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*------------------------------READ DATA----------------------*/
	  mbal_NRF24L01_GetData(&nrf_rx_cfg, &data_rx_real);
	  MPU6050_Read_All(&hi2c2, &MPU6050_Data);//read data
	  correct_data_and_calibrate_3truc();
	  calculate_agl_roll_pitch();
	  calculate_setpoint_pid();
	  read_hc05_and_fillter();
///*-------------------------Check right left front behind----------------------*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  button_press = 0;
	  for(button_press = 0 ;button_press < 5 ; button_press ++){
		  if((data_rx_real>>button_press)&1){
			  break;
		  }
		  if(button_press == 4){
			  if(!((data_rx_real>>button_press)&1)){
				  button_press = 10;
				  break;
			  }
		  }
	  }
	if(button_press==3)		{	quad_stop();	}
	else if(button_press==2){	if (hacanh == false){quad_start();	}}
	else if(button_press==0){	quad_down();	}
	else if(button_press==1){	if (hacanh == false){quad_up();		}}
	else {}
///*------------------------------Operation Run Motor ----------------------*/
  	  	PID_Calculation(&PID_ROLL,pid_roll_setpoint,MPU6050_Data.Gx_roll );
  		PID_Calculation(&PID_PITCH,pid_pitch_setpoint,MPU6050_Data.Gy_pitch );
  		PID_Calculation(&PID_YAW,pid_yaw_setpoint,MPU6050_Data.Gz_yaw );
  		PID_Calculation_thr(&PID_HG,Setting_docao,Kalman_Distance);

if(hacanh == true && is_running == true){
	uint32_t time = (HAL_GetTick() - loop_time_hacanh);
	if(time > 500){
		throttle = throttle - 20;
		loop_time_hacanh = HAL_GetTick();
	}
	if((Kalman_Distance <= 8)&& (throttle <= 1200))
	{	quad_stop();	}
	throttle_PID = throttle;
}
else{
	throttle_PID = throttle;
}
if((is_running== true)){
	  		PWM[0] = (uint16_t)(throttle_PID + PID_ROLL.pid_result + PID_PITCH.pid_result + PID_YAW.pid_result);
	  		PWM[1] = (uint16_t)(throttle_PID + PID_ROLL.pid_result - PID_PITCH.pid_result - PID_YAW.pid_result);
	  		PWM[2] = (uint16_t)(throttle_PID - PID_ROLL.pid_result + PID_PITCH.pid_result - PID_YAW.pid_result);
	  		PWM[3] = (uint16_t)(throttle_PID - PID_ROLL.pid_result - PID_PITCH.pid_result + PID_YAW.pid_result);
	  for(int i=0 ; i <4 ; i++){
	  	if (PWM[i] > Max_PWM) {PWM[i] = Max_PWM;}
	  	if (PWM[i] < Min_PWM) {if(hacanh == false){PWM[i] = Min_PWM;}}
	  }
  		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,PWM[0]);
  		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,PWM[1]);
  		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,PWM[2]);
  		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,PWM[3]);
	  }
	  else{
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1000);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1000);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1000);
	  }
///*-----------------------------End-Operation ----------------------*/
  check_looptime();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSN_pin_Pin|CE_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(hc_trigger_pin_GPIO_Port, hc_trigger_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_pin_Pin CE_pin_Pin */
  GPIO_InitStruct.Pin = CSN_pin_Pin|CE_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : hc_trigger_pin_Pin */
  GPIO_InitStruct.Pin = hc_trigger_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(hc_trigger_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : irq_nrf_Pin */
  GPIO_InitStruct.Pin = irq_nrf_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(irq_nrf_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
