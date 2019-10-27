#include "stm32f10x.h"                  // Device header
#include "stm32f10x_dma.h"              // Keil::Device:StdPeriph Drivers:DMA
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include <stdio.h>
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include <stdlib.h>
#include "stm32f10x_dac.h"              // Keil::Device:StdPeriph Drivers:DAC

#define ADC1_DR ((uint32_t)0x4001244C) 
#define h 25

const float lpf100[h]= {-0.001358, -0.000931, 0.000000, 0.002555, 0.007892, 0.016857, 0.029658, 0.045671, 0.063434, 0.080849, 0.095555, 0.105392, 0.108852, 0.105392, 0.095555, 0.080849, 0.063434, 0.045671, 0.029658, 0.016857, 0.007892, 0.002555, 0.000000, -0.000931, -0.001358};

const 	float lpfanapnoi[h]= {0.005083, 0.006309, 0.009656, 0.015082, 0.022354, 0.031060, 0.040639, 0.050428, 0.059717, 0.067810, 0.074088, 0.078063, 0.079423, 0.078063, 0.074088, 0.067810, 0.059717, 0.050428, 0.040639, 0.031060, 0.022354, 0.015082, 0.009656, 0.006309, 0.005083};
const float NUM[13][2] = {
  {
     0.1712435335,                       - 0.1712435335
  },
  
  {
     0.1712435335,                          -0.1712435335
  },
  
  {
     0.1592428535,                         -0.1592428535
  },
  
  {
     0.1592428535,                           - 0.1592428535
  },
  
  {
     0.1369737983,                          -0.1369737983
  },
  
  {
     0.1369737983,                         -0.1369737983
  },
  
  {
     0.1059219167,                          -0.1059219167
  },
 
  {
     0.1059219167,                          -0.1059219167
  },
  
  {
     0.0684890002,                           -0.0684890002
  },
  
  {
     0.0684890002,                           -0.0684890002
  },
  
  {
    0.03022797965,                           -0.03022797965
  },
  
  {
    0.03022797965,                           -0.03022797965
  },
  
  {
     0.8912509084,                            0 
  }
};

const float DEN[13][2] = {
 
  {
                   -1.775036931,   0.9917705059 
  },
  
  {
                   -1.981937647,   0.9977175593 
  },
  
  {
                   -1.777043819,   0.9763682485 
  },
  
  {
                   -1.975842714,   0.9928928614 
  },
  
  {
                   -1.793755054,   0.9637604952 
  },
  
  {
                  -1.967325926,    0.987267077 
  },
  
  {
                  -1.820886731,   0.9555283785 
  },
  
  {
                  -1.955182195,   0.9803657532 
  },
  
  {
                   -1.853209734,   0.9528750181 
  },
  
  {
                   -1.938007832,   0.9720746279 
  },
  
  {
                  -1.885784149,   0.9559460282 
  },
  
  {
                   -1.914776802,    0.963221252 
  },
  {
                            0,              0 
  }
};
/*SEira Deigmatolhpsias.
 PC4 - 1
PC5 - 2
PC0 - 3
PC1 - 4
PC2 - 5
PC3 - 6

PB0 - 7
PB1 - 8

*/



volatile  uint16_t Values[8];

		volatile uint16_t j;
	 volatile uint16_t k;
volatile uint32_t status=0;
volatile uint8_t TxBuffer[6];
void USARTInit(void);                   
void ADCInit(void);
void DMAInit(void);
void TimInit(void);
void SendCHN(int matrix[]);
void bubbleSort(int numbers[], int array_size,int indexes[]);
void SendData(int data);
float  Filteraply(const float b[], float Buffer[],int c);
#define TxBufferSize 6
void Sendnewline(char nlc);	
void DACint(void);



float us1[14][3]={0},us2[14][3]={0},us3[14][3]={0},us4[14][3]={0}; // direct form two iir variables  gia ta 4 shmata
float ys1[15]={0}, ys2[15]={0}, ys3[15]={0},ys4[15]={0};


int main (void){
	int DC[8]={0};
	int Ene[8]={0};
	int Sum[8]={0};
	int Time=0;
  int Rvalues[8]={0};	
	int  indexes[8];
	int g;
	char f;
	int t=0;
	int u=0;
	int sumy1=0;
	int sumy2=0;
	int sumy3=0;
	int sumy4=0;
float  Buffersign1[h]={0};
float Buffersign2[h]={0};
float Buffersign3[h]={0};
float Buffersign4[h]={0};
int si=0;
float y1,y2,y3,y4,anap1,anap2,anap3,anap4,kard1,kard2,kard3,kard4;
float Buffery1[h]={0};
float Buffery2[h]={0};
float  Buffery3[h]={0};
float Buffery4[h]={0};
int Datasend[8]={0};
int iiri=0;
unsigned char c1 ;

	USARTInit();
  ADCInit();
	DMAInit();

DACint();

  DMA_Cmd(DMA1_Channel1, ENABLE);
	for ( g=0;g<10000;g++){}
	TimInit();
	
	 while(1){


	
	if (status==1){
		status=0;
	Time=Time+1;
		
	for (g=0;g<8;g++){
		Sum[g]=Sum[g]+Values[g];
		Rvalues[g]=Values[g]-DC[g];// stis 4000 prwtes epanalipseis rvalues=values dioti dc =0
	  
	}
	if (Time==4000){   
		Time=0;
		
		for (g=0;g<8;g++){    //compute DC
			DC[g]=Sum[g]/4000;
		    Sum[g]=0;      // arxikopoihsh tou Sum gia ton epomeno ypologismo tou DC
		Ene[g]= abs(Rvalues[g]);
		  indexes[g]=g;  }
		
		bubbleSort(Ene,8,indexes); //opote tha steilw ta kanalia pou einai stis 4 teleutaies theseis tou pinaka indexes
			
			
			f='c';
			Sendnewline(f);
			f='n';
			Sendnewline(f);
			f='g';
			Sendnewline(f);
			SendCHN( indexes);
			
		f='\n';
			Sendnewline(f);
			f='\r';
			Sendnewline(f);
		}
if (t==25){t=0;}


	Buffersign1[t]=Values[indexes[7]];
	Buffersign2[t]=Values[indexes[6]];
	Buffersign3[t]=Values[indexes[5]];
	Buffersign4[t]=Values[indexes[4]];
	
	
	y1=  Filteraply(lpf100, Buffersign1,t);
	y2=  Filteraply(lpf100, Buffersign2,t);   // vgazoume mia eksodo syndiazodas filtro kai buffer kathe tick tou timer
	y3=  Filteraply(lpf100, Buffersign3,t);  // y=b0*x[n] +b1*x[n-1] +b2*x[n-3] +++... n parousa xronikh stigmi
	y4=  Filteraply(lpf100, Buffersign4,t);
	 t=t+1;
	
/*sumy1= sumy1+y1;
sumy2= sumy2+y2;
sumy3= sumy3+y3;
sumy4= sumy4+y4; */
	if ((Time%5)== 0){   // synthikh wste na ypodeigmatoliptisoume sta 200 HZ, oversampling
		
		if (u==25){u=0;}
		
 Buffery1[u]=/*sum*/y1;
 Buffery2[u]=/*sum*/y2;
 Buffery3[u]=/*sum*/y3;
 Buffery4[u]=/*sum*/y4;
 
 anap1=Filteraply(lpfanapnoi,Buffery1,u);
anap2=Filteraply(lpfanapnoi,Buffery2,u);
anap3=Filteraply(lpfanapnoi,Buffery3,u);
anap4=Filteraply(lpfanapnoi,Buffery4,u);
u=u+1;
		
for(iiri=0;iiri<14;iiri++){  // shifting timwn  tou pinaka u 
			us1[iiri][2]=us1[iiri][1];
			us1[iiri][1]=us1[iiri][0]; //1 shma
			
			us2[iiri][2]=us2[iiri][1];
			us2[iiri][1]=us2[iiri][0]; //2 shma
			
			
			
			
			us3[iiri][2]=us3[iiri][1];
			us3[iiri][1]=us3[iiri][0]; //3 shma
			
			
			us4[iiri][2]=us4[iiri][1];
			us4[iiri][1]=us4[iiri][0];  //4 shma
			
		}		
		ys1[0]=y1;
		ys2[0]=y2;
		ys3[0]=y3;
		ys4[0]=y4;
		
		for (iiri=0;iiri<14;iiri++){
			
		us1[iiri][0]=ys1[iiri]-DEN[iiri][0]*us1[iiri][1] -DEN[iiri][1]*us1[iiri][2];
		ys1[iiri+1]=NUM[iiri][0]*us1[iiri][0]  +NUM[iiri][1]*us1[iiri][2];
			ys1[iiri +1]=ys1[iiri +1 ];
			
				//ys1[iiri +1]=3*ys1[iiri +1 ];
			
			us2[iiri][0]=ys2[iiri]-DEN[iiri][0]*us2[iiri][1] -DEN[iiri][1]*us2[iiri][2];
		ys2[iiri+1]=NUM[iiri][0]*us2[iiri][0]  +NUM[iiri][1]*us2[iiri][2];
			ys2[iiri +1]=3*ys2[iiri +1 ];
			
			
			
			us3[iiri][0]=ys3[iiri]-DEN[iiri][0]*us3[iiri][1] -DEN[iiri][1]*us3[iiri][2];
		ys3[iiri+1]=NUM[iiri][0]*us3[iiri][0]  +NUM[iiri][1]*us3[iiri][2];
			ys3[iiri +1]=3*ys3[iiri +1 ];
			
			us4[iiri][0]=ys4[iiri]-DEN[iiri][0]*us4[iiri][1] -DEN[iiri][1]*us4[iiri][2];
		ys4[iiri+1]=NUM[iiri][0]*us4[iiri][0]  +NUM[iiri][1]*us4[iiri][2];
			ys4[iiri +1]=3*ys4[iiri +1 ];
		}
		
		kard1=ys1[14];
kard2=ys2[14];
kard3=ys3[14];
 kard4=ys4[14];
		DAC_SetChannel1Data(DAC_Align_12b_R, 2096+kard1);

 
		/*sumy1=0;
		sumy2=0;
		sumy3=0;
		sumy4=0;*/
		
 /*SendData(anap1);
 SendData(kard1);
 SendData(anap2);
 SendData(kard2);
 SendData(anap3);
 SendData(kard3);
 SendData(anap4);
 SendData(kard4);
 */
 
 Datasend[0]=anap1;
Datasend[1]=kard1;
 Datasend[2]=anap2;
 Datasend[3]=kard2;
 Datasend[4]=anap3;
 Datasend[5]=kard3;
 Datasend[6]=anap4;
 Datasend[7]=kard4; 
 
 
 
 
for(si=0;si<8;si++){
	 c1=((Datasend[si]&0xF00)>>8) ;
	c1 = (c1	< 9 ) ? c1 +'0' : c1-10+65 ;
	Sendnewline(c1);
	c1=((Datasend[si]&0xF0)>>4 );
	c1 = (c1	< 9 ) ? c1 +'0' : c1-10+65 ;
	Sendnewline(c1);
	
	 c1=(Datasend[si]&0xF );	 
	c1 = (c1	< 9 ) ? c1 +'0' : c1-10+65 ;
	 Sendnewline(c1);


 } 
 
 
			Sendnewline('\r'); 
 Sendnewline('\n'); 
 
	}
	
	
		
	
} }
 
		
}


void ADCInit(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB, ENABLE);
GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
GPIO_InitStructure.GPIO_Pin= GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode=ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel=8;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14,1, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15,2, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10,3, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11,4, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12,5, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13,6, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8,7, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9,8, ADC_SampleTime_7Cycles5);
	
	ADC_Cmd(ADC1,ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
}

void DMAInit(void){
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitTypeDef DMA_InitSructure;
	DMA_DeInit(DMA1_Channel1);
	DMA_InitSructure.DMA_M2M=DMA_M2M_Disable;
	DMA_InitSructure.DMA_Mode=DMA_Mode_Circular;
	DMA_InitSructure.DMA_Priority=DMA_Priority_Medium;
	DMA_InitSructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
	DMA_InitSructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
	DMA_InitSructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
  DMA_InitSructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	DMA_InitSructure.DMA_DIR=DMA_DIR_PeripheralSRC;
	DMA_InitSructure.DMA_BufferSize=8;
	DMA_InitSructure.DMA_PeripheralBaseAddr= (uint32_t) ADC1_DR;
	DMA_InitSructure.DMA_MemoryBaseAddr=(uint32_t) Values;
	DMA_Init(DMA1_Channel1,&DMA_InitSructure);
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Channel1,ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel= DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;
	NVIC_Init(&NVIC_InitStructure);}
	
	
	
	void USARTInit(void){

		
		
		//enabling the clock to usart pins
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
		

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		
		///gpio confs
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
		
		//usart parametrs
		USART_InitTypeDef USART_InitStructure;
		USART_InitStructure.USART_BaudRate = 921600;
 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
		
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);
	
		
	
		
		
		
	}
	
	
	
void TimInit(void){
		  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //tim2 configuration
  
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	  
	
	
	
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1;  // 1 MHz down to 1 KHz (1 ms)
  TIM_TimeBaseStructure.TIM_Prescaler = 24 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	 TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
TIM_Cmd(TIM2, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;

NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM2 IRQ Channel
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//Preemption Priority
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //Sub Priority
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
	}
	
	
	
	
	void Sendnewline(char nlc){
		
		char s;
		s=nlc;
		USART_SendData(USART2, s);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		
	}
void SendCHN(int matrix []){
	char str[4];
for (j=4;j<8; j++){
	
		 sprintf( str, "%d ", matrix[j]);      //vazoume tis times se string kai apo to string ston buffer
			for (k=0;k<4;k++){
				TxBuffer[k]=str[k];}
	 
		uint8_t NbrOfDataToTransfer = TxBufferSize;
		uint8_t TxCounter = 0;
		
		
	
		
		
		
		 
  while(NbrOfDataToTransfer--)
  {
    USART_SendData(USART2, TxBuffer[TxCounter++]);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);      
    
  }
}
}


void SendData(int data){  
	char str[6];

	
		 sprintf( str, "%d ", data);      //vazoume tis times se string kai apo to string ston buffer
			for (k=0;k<6;k++){
				TxBuffer[k]=str[k];}
	 
		uint8_t NbrOfDataToTransfer = TxBufferSize;
		uint8_t TxCounter = 0;
		
		
	
		
		
		
		 
  while(NbrOfDataToTransfer--)
  {
    USART_SendData(USART2, TxBuffer[TxCounter++]);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);      
    
  }
}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	


void bubbleSort(int numbers[], int array_size,int indexes[]){

  int i, j, temp;
 
  for (i = (array_size - 1); i > 0; i--)
  {
    for (j = 1; j <= i; j++)
    {
      if (numbers[j-1] > numbers[j])
      {
        temp = numbers[j-1];
        numbers[j-1] = numbers[j];
        numbers[j] = temp;
				temp=indexes[j-1];    // oti alages kanei se theseis ston ene kanei kai sta indexes
				indexes[j-1]=indexes[j];
				indexes[j]=temp;
      }
    }
  }
}



float Filteraply(const float b[], float Buffer[], int c){
	
	int y=0 ;
int i=0;
	int q;
	q=c+1;
	y=b[24]*Buffer[c];
	
	if (q==25){q=0;}
while(q!=c){
		y=y+b[i]*Buffer[q];
	
	i=i+1;
	q=q+1;
if(q==25){q=0;}
	}
	return y;

}

void DACint(void){
	

  /* Once the DAC channel is enabled, the corresponding GPIO pin is automatically 
     connected to the DAC converter. In order to avoid parasitic consumption, 
     the GPIO pin should be configured in analog */
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	DAC_InitTypeDef            DAC_InitStructure;
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_4095;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	 DAC_Cmd(DAC_Channel_1, ENABLE);
	
	
}


