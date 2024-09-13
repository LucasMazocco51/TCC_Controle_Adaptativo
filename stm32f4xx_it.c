/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FS (float)(50000.0f)   // frequencia de amostragem

#define KPi_BOOST (0.01282607473f)
#define KIi_BOOST (0.0007515158832f)
#define KWi_BOOST (0.2f)

#define KPv_BOOST (1.899604662f)
#define KIv_BOOST (0.004753836575f)
#define KWv_BOOST (0.2f)

// Ganho do PI da etapa de saída
#define KP_INJ (0.2351204998)
#define KI_INJ (0.01596757568)
#define KW_INJ (0.01)

//numero de pontos do vetor da referência de corrente
#define N_I_REF (209)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// frequência do filtro passa-baixas discreto da medida do barramento CC
float Pf_Vcc = 0.99f;

uint32_t cont1=0, cont2=0;

// Debouncing do trigger de entrada
int trigger_inj1_OK=1, k_trigger_inj1=0;
uint8_t trigger_inj1 = 0;

float k_ref=0.5f;

float OFFSET_Iinj1=379.49302f, OFFSET_Iboost=355.01f, OFFSET_Vin=-2.34589148f, OFFSET_Vo = -4.72751f;
float Ganho_Iinj1=0.00655728f, Ganho_Iboost=0.00318268f, Ganho_Vin=0.00463391f, Ganho_Vo = 0.025344f;
float Vin=1.0f, Vinf=1.0f;

uint8_t start=1, Enable_boost=0, Vcc_OK=0, Enable_Inv=1;

uint8_t FLAG_PROT_IBOOST = 0, FLAG_PROT_VIN = 0, FLAG_PROT_VO = 0, FLAG_PROT_I_INJ = 0, FLAG_PROT=0, FLAG_PROT_VO_MIN=0;
float IBOOST_PROT = 8.0f, I_INJ_PROT = 16.0f, VIN_PROT=15.0f, Vo_PROT = 80.0f;

float COMP1=0.0f, COMP2=1680.0f, COMPb=1.0f;

float Iinj1_ref=0.0f;
float i_inj1=0.0f, x_inj1=0.0f, u_inj1=0.0f, u_inj1_sat=0.0f, erro_i_inj1=0.0f;
int index1=0;

//float i_inj1f=0.0f;

float COMP1_OFF = 0.0f, COMP2_OFF = 1681.0f;

float TPER = 1.0f, TPERd2=0.5f;


// Referência periódica de corrente da etapa de saída
 static float Iinj_ref_vec[N_I_REF] = {0.279167f, 0.558333f, 1.116667f, 1.675000f, 2.233333f, 2.791667f, 3.350000f, 3.908333f, 4.466667f, 5.025000f, 5.583333f, 6.141667f, 6.700000f, 7.258333f, 7.816667f, 8.375000f, 8.933333f, 9.491667f, 10.050000f, 10.608333f, 11.166667f, 11.725000f, 12.283333f, 12.841667f, 13.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 9.400000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f, 3.700000f};

// Controle repetitivo
float K_filtro_q = 0.998f, cr = 0.15f;
int D = 2, kp1 = 0, kpd = 0, k0 = 0, km1 = 0;
float ur_inj1 = 0.0f, ur_inj1_ant = 0.0f;
float  erro_i_inj1_vec[N_I_REF], ur_inj1_vec[N_I_REF];


// Controle de corrente
float Iboost = 0.0f, Iboostf=0.0f;
float Iboost_ref = 0.0f, Iboost_ref_sat = 1.0f;
float erro_Iboost=0.0f, x_Iboost=0.7f;
float u_boost=0.0f, u_boost_sat=0.0f;
float U_BOOST_MIN = 0.000596f, U_BOOST_MAX = 0.95f;
float COMP_BOOST_MIN = 1.0f;
float IBOOST_MAX=5.0f, IBOOST_MIN=-1.0f;

// Controle de tensão
float VCC_REF = 70.0f;
float Vo = 10.0f, Vof=10.0f, Vo_ref = 10.0f;
float erro_Vo=0.0f;
float x_Vo=0.0f;

// Rampa Partida
float step_rampa_Vcc=0.0f, t_rampa_Vcc=1.0f;
uint8_t init_Vcc = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
/* USER CODE BEGIN EV */

extern TIM_HandleTypeDef htim8;

extern union Switches Sw;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

	HAL_GPIO_WritePin(Time_Rotina_GPIO_Port, Time_Rotina_Pin, GPIO_PIN_SET);

	 if(HAL_GPIO_ReadPin(trigger_inj1_GPIO_Port, trigger_inj1_Pin)){
		 if(trigger_inj1_OK){
			 trigger_inj1 = 1;
			 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			 trigger_inj1_OK = 0;
		 }
	 }else{
		 trigger_inj1 = 0;
		 HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		 k_trigger_inj1++;
		 if(k_trigger_inj1>=10){
			 k_trigger_inj1=10;
			 trigger_inj1_OK= 1;
		 }
	 }


	/*
	Sw.bit.Sw1 = HAL_GPIO_ReadPin(GPIOE, Sw1_Pin);
	Sw.bit.Sw2 = HAL_GPIO_ReadPin(GPIOE, Sw2_Pin);
	Sw.bit.Sw3 = HAL_GPIO_ReadPin(GPIOE, Sw3_Pin);
	Sw.bit.Sw4 = HAL_GPIO_ReadPin(GPIOE, Sw4_Pin);
	Sw.bit.Sw5 = HAL_GPIO_ReadPin(GPIOE, Sw5_Pin);
	Sw.bit.Sw6 = HAL_GPIO_ReadPin(Sw6_GPIO_Port, Sw6_Pin);
	*/

	//if(Sw.all){}

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

	/*if(cont1++ > FS) {
		cont1 = 0;
		HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_SET);
	}else if(cont1 == 25000) HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_RESET);
	*/

	while(! (hadc1.Instance->SR & ADC_FLAG_JEOC)>>ADC_SR_JEOC_Pos ){ }
	Vo = Ganho_Vo * (hadc1.Instance->JDR1 - OFFSET_Vo);

	while(! (hadc2.Instance->SR & ADC_FLAG_JEOC)>>ADC_SR_JEOC_Pos ){ }
	i_inj1  = Ganho_Iinj1  * (hadc2.Instance->JDR1 - OFFSET_Iinj1);

	while(! (hadc3.Instance->SR & ADC_FLAG_JEOC)>>ADC_SR_JEOC_Pos ){ }
	Iboost = Ganho_Iboost * (hadc3.Instance->JDR1 - OFFSET_Iboost);
	Vin    = Ganho_Vin    * (hadc3.Instance->JDR2 - OFFSET_Vin   );

	if(start){
		start=0;
		Vof = Vo;
		Vinf = Vin;
		TPER = (float)(htim1.Init.Period);
		TPERd2 = TPER * 0.5f;
	}
	/*Vof = Pf_Vcc * Vof + (1.0f-Pf_Vcc) * Vo;


	Vinf = Pf_Vcc * Vinf + (1.0f-Pf_Vcc) * Vin;

	i_inj1f = Pf_Vcc * i_inj1f + (1.0f-Pf_Vcc) * i_inj1;  // Usado na Calibração
	Iboostf = Pf_Vcc * Iboostf + (1.0f-Pf_Vcc) * Iboost;  // Usado na Calibração
	*/

	// Injetor 1 ///////////////////////////////////////////////////////////////////////
	if(trigger_inj1 && !FLAG_PROT && Vcc_OK && Enable_Inv){

		Iinj1_ref = k_ref * Iinj_ref_vec[index1];

		erro_i_inj1 = Iinj1_ref - i_inj1;

		//vetor do erro
		erro_i_inj1_vec[index1] = erro_i_inj1;

		if(index1 < (N_I_REF-4))  //variáveis auxiliares e saturação de índices do controlador repetitivo
		{
			k0 = index1;

			kp1 = k0 + 1;
			if(kp1 > (N_I_REF-1)) kp1 = N_I_REF-1;

			km1 = k0 - 1;
			if(km1 < 0) km1 = 0;

			kpd = k0 + D;
			if(kpd > (N_I_REF-1)) kpd = N_I_REF-1;

			ur_inj1_ant = ur_inj1_vec[k0];

			ur_inj1_vec[k0] = K_filtro_q * ur_inj1_vec[k0] + cr * erro_i_inj1_vec[kpd];

			ur_inj1 = ur_inj1_vec[k0];
		}

	   u_inj1 = (KP_INJ * erro_i_inj1 + x_inj1 + ur_inj1);

	   u_inj1_sat = u_inj1;
	   if(u_inj1_sat<-1.0f){
		u_inj1_sat =-1.0f;
		ur_inj1_vec[k0] = 0.995f*ur_inj1_ant;
	   }else if(u_inj1_sat> 1.0f) {
		u_inj1_sat = 1.0f;
		ur_inj1_vec[k0] = 0.995f*ur_inj1_ant;
	   }
	   x_inj1 = x_inj1 + KI_INJ * erro_i_inj1 - KW_INJ * (u_inj1 - u_inj1_sat);


	   COMP1 = TPERd2 + u_inj1_sat * TPERd2;
	   COMP2 = TPERd2 - u_inj1_sat * TPERd2;

	   index1++;
	   if(index1>=N_I_REF-1)index1=N_I_REF-1;

	}else{
	   index1=0;
	   COMP1=COMP1_OFF;
	   COMP2=COMP2_OFF;

	   x_inj1=0.9f;
	   u_inj1=0.0f;
	   u_inj1_sat=0.0f;
	   ur_inj1=0.0f;
	   Iinj1_ref = 0.0f;
	}

	htim1.Instance->CCR1 = COMP1;
	htim1.Instance->CCR2 = COMP2;

	////////Controle de Tensão///////////////////////////////////////////////////
	if(Enable_boost && !FLAG_PROT){

		if(Vo_ref<VCC_REF){
			if(init_Vcc){
				init_Vcc=0;
				Vo_ref = Vo + 2.0f;
				if(Vo_ref>=VCC_REF) Vo_ref = VCC_REF;
				step_rampa_Vcc = (VCC_REF-Vo_ref)/(t_rampa_Vcc*FS);
			}

			Vo_ref = Vo_ref + step_rampa_Vcc;

			if(Vo_ref>=VCC_REF) {
				 Vo_ref = VCC_REF;
				 Vcc_OK=1;
				 Enable_Inv = 1;
			 	}
		}else{
			Vo_ref = VCC_REF;
		}

		erro_Vo = Vo_ref - Vo;
		Iboost_ref = KPv_BOOST * erro_Vo + x_Vo;

		Iboost_ref_sat = Iboost_ref;
		if(Iboost_ref_sat< IBOOST_MIN) Iboost_ref_sat = IBOOST_MIN;
		if(Iboost_ref_sat> IBOOST_MAX) Iboost_ref_sat = IBOOST_MAX;

		x_Vo = x_Vo + KIv_BOOST * erro_Vo - KWv_BOOST * (Iboost_ref - Iboost_ref_sat);


		////////Controle de Corrente///////////////////////////////////////////////////
		erro_Iboost = Iboost_ref_sat - Iboost;

		u_boost = KPi_BOOST * erro_Iboost + x_Iboost;

		u_boost_sat = u_boost;
		if(u_boost<U_BOOST_MIN) u_boost_sat = U_BOOST_MIN;
		if(u_boost>U_BOOST_MAX) u_boost_sat = U_BOOST_MAX;

		x_Iboost = x_Iboost + KIi_BOOST * erro_Iboost - KWi_BOOST * (u_boost - u_boost_sat);

		COMPb = TPER * u_boost_sat;

		if(COMPb <= COMP_BOOST_MIN) COMPb = COMP_BOOST_MIN;

	}else{
		COMPb = COMP_BOOST_MIN;
		x_Iboost = 0.0f;
		x_Vo = 0.0f;
		init_Vcc = 1;
		Enable_Inv = 0;
		Vcc_OK = 0;
		Vo_ref = 10.0f;
	}


	if(Iboost > IBOOST_PROT ) FLAG_PROT_IBOOST = 1;
	if(i_inj1 > I_INJ_PROT  ) FLAG_PROT_I_INJ  = 1;
	if(Vin    > VIN_PROT    ) FLAG_PROT_VIN    = 1;
	if(Vo     > Vo_PROT     ) FLAG_PROT_VO     = 1;
	//if(Vo     < 0.6f*VCC_REF) FLAG_PROT_VO_MIN = 1;

	FLAG_PROT = FLAG_PROT_IBOOST + FLAG_PROT_VIN + FLAG_PROT_VO + FLAG_PROT_I_INJ + FLAG_PROT_VO_MIN;

	if(FLAG_PROT){
		COMP1=COMP1_OFF;
		COMP2=COMP2_OFF;
		COMPb = COMP_BOOST_MIN;
		Enable_Inv = 0;
		Enable_boost = 0;
	}

	htim8.Instance->CCR1 = COMPb;

	HAL_GPIO_WritePin(Time_Rotina_GPIO_Port, Time_Rotina_Pin, GPIO_PIN_RESET);

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
