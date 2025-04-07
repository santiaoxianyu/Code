#include "ti_msp_dl_config.h"
#include "jy61P.h"
#include "usart3.h"


void jy61p_Init(void)
{
	NVIC_ClearPendingIRQ(UART0_INT_IRQn);
	NVIC_EnableIRQ(UART0_INT_IRQn);
	DL_UART_clearInterruptStatus(UART0,DL_UART_INTERRUPT_RX);//清除中断标志位

}

void UART0_IRQHandler(void)
{
	uint8_t RxData = DL_UART_receiveData(UART0);
	jy61p_ReceiveData(RxData);
	
}

