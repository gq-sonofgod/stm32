

#include "stm32f10x.h"
#include <stdio.h>
#include "delay.h"

void TIM1_PWM_Init(u16 arr,u16 psc)
{  
	 GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// ??TIM1??
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //??GPIO??????
	                                                                     	

   //????????????,??TIM1 CH1?PWM????
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //??????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	
	TIM_TimeBaseStructure.TIM_Period = arr; //???????????????????????????	 80K
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //??????TIMx???????????  ???
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //??????:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM??????
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //??TIM_TimeBaseInitStruct?????????TIMx???????

 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //???????:TIM????????2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //??????
	TIM_OCInitStructure.TIM_Pulse = 0; //????????????????
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //????:TIM???????
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //??TIM_OCInitStruct???????????TIMx

	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE ?????	

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1?????	 
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //??TIMx?ARR????????
	
	TIM_Cmd(TIM1, ENABLE);  //??TIM1
 
   
}


/*************************************************
函数: int main(void)
功能: main主函数
参数: 无
返回: 无
**************************************************/
int main(void)
 {	
	u16 led0pwmval=0;    
	u8 dir=1;	

	TIM1_PWM_Init(899,0);//????PWM??=72000/(899+1)=80Khz 
   	while(1)
	{
 		delay_ms(500);	 
//		if(dir)
//			led0pwmval++;
//		else 
//			led0pwmval--;	 
// 		if(led0pwmval>900)
//			dir=0;
//		if(led0pwmval==0)
//			dir=1;	   					 
		TIM_SetCompare1(TIM1,900);	   
//		delay_ms(500);	
//		TIM_SetCompare1(TIM1,900);	
	} 
}
//?? LED0_PWM_VAL ??? 0 ?? 300,???? 300 ?? 0,????,?? DS0 ????????????,?????????



