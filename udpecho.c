/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#include "udpecho.h"

#include "lwip/opt.h"

#if LWIP_NETCONN

#include "lwip/api.h"
#include "lwip/sys.h"

////////////////////////// INCLUIDOS PARA DAC Y PIT /////////////////////////////////////
#include "board.h"
#include "fsl_dac.h"

#include "fsl_common.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"

SemaphoreHandle_t g_led_semaphore;

#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define SOPT5_UART0TXSRC_UART_TX      0x00u   /*!< UART 0 transmit data source select: UART0_TX pin */

#define paquetes 200

#define DEMO_DAC_BASEADDR DAC0
volatile bool pitIsrFlag = false;
uint32_t dacValue;

uint16_t ping_buf[paquetes];
uint16_t pong_buf[paquetes];

uint8_t y = 0;
uint8_t error = 0;
uint32_t num = 0;
uint8_t flag = 0;

void PIT0_IRQHandler(void) //Aquí falta ponerle un campo de bits para que despierte a la tarea del dac
{
	//BaseType_t xHigherPriorityTaskWoken; //agregado
	//xHigherPriorityTaskWoken = pdFALSE;  //agregado
	//if(pong_buf[y] < 0) pong_buf[y] = 0;
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	if(flag == 0)
	{
		DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, ((ping_buf[y] >> 4)+2047));
		y++;
		}
		else
		{
		DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, ((pong_buf[y] >> 4)+2047));
		y++;
	}

   	if(y == paquetes)
   	{
   	y = 0;
		if(flag == 0)
		{
			flag = 1;
		}
		else
		{
			flag = 0;
		}
   	//PIT_StopTimer(PIT, kPIT_Chnl_0);
   	}
    //xSemaphoreGiveFromISR( g_led_semaphore, &xHigherPriorityTaskWoken ); //agregado
    //portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void DAC_PIT_INIT(void)
{
    uint8_t index;
    dac_config_t dacConfigStruct;


    CLOCK_EnableClock(kCLOCK_PortB);                           /* Port B Clock Gate Control: Clock enabled */
      CLOCK_EnableClock(kCLOCK_PortA);
      CLOCK_EnableClock(kCLOCK_PortC);
      CLOCK_EnableClock(kCLOCK_PortE);

      PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_MuxAlt3);           /* PORTB16 (pin 62) is configured as UART0_RX */
      PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_MuxAlt3);           /* PORTB17 (pin 63) is configured as UART0_TX */
      SIM->SOPT5 = ((SIM->SOPT5 &
        (~(SIM_SOPT5_UART0TXSRC_MASK)))                          /* Mask bits to zero which are setting */
          | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)       /* UART 0 transmit data source select: UART0_TX pin */
        );

    index = 0;
    dacValue = 0;

    /* CONFIGURACION DEL PIT A 1 SEGUNDO*/

	pit_config_t pitConfig;
	PIT_GetDefaultConfig(&pitConfig);
	PIT_Init(PIT, &pitConfig);
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, CLOCK_GetBusClkFreq()/50000); //44100


	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	EnableIRQ(PIT0_IRQn);
	NVIC_EnableIRQ(PIT0_IRQn);
	NVIC_SetPriority(PIT0_IRQn,5);

	//aquí soliamos iniciar el timer

	g_led_semaphore = xSemaphoreCreateBinary();

    /* Configure the DAC. */
    /*
     * dacConfigStruct.referenceVoltageSource = kDAC_ReferenceVoltageSourceVref2;
     * dacConfigStruct.enableLowPowerMode = false;
     */
    DAC_GetDefaultConfig(&dacConfigStruct);
    DAC_Init(DEMO_DAC_BASEADDR, &dacConfigStruct);
    DAC_Enable(DEMO_DAC_BASEADDR, true);             /* Enable output. */
    DAC_SetBufferReadPointer(DEMO_DAC_BASEADDR, 0U); /* Make sure the read pointer to the start. */
                                                     /*
                                                     * The buffer is not enabled, so the read pointer can not move automatically. However, the buffer's read pointer
                                                     * and itemss can be written manually by user.
                                                     */
}



///////////////////////////////////////////////////////////////////////////////////////
/*-----------------------------------------------------------------------------------*/

static void server_thread(void *arg)
{
	struct netconn *conn;
	struct netbuf *buf;

//	char *msg;

//	uint16_t len;

	//LWIP_UNUSED_ARG(arg);
	conn = netconn_new(NETCONN_UDP);
	netconn_bind(conn, IP_ADDR_ANY, 50007);
	//LWIP_ERROR("udpecho: invalid conn", (conn != NULL), return;);
	static uint8_t i = 0;
	static uint8_t ii = 0;


	while (1)
	{
		netconn_recv(conn, &buf);
		//netbuf_data(buf, (void**)&msg, &len);


		if(i == 0)
		{
		netbuf_copy(buf,ping_buf,sizeof(ping_buf));
		}
		else
		{
		netbuf_copy(buf,pong_buf,sizeof(pong_buf));
		}
		netbuf_delete(buf);
		if(ii == 0)
		{
		PIT_StartTimer(PIT, kPIT_Chnl_0);
		ii = 1;
		}


	}
}


static void server_thread1(void *arg)
{
	struct netconn *conn;
	struct netbuf *buf;

	char *msg;

	uint16_t len;

	//LWIP_UNUSED_ARG(arg);
	conn = netconn_new(NETCONN_UDP);
	netconn_bind(conn, IP_ADDR_ANY, 50008);
	//LWIP_ERROR("udpecho: invalid conn", (conn != NULL), return;);
	PRINTF("PUTO");

	while (1)
	{
		netconn_recv(conn, &buf);
		netbuf_data(buf, (void**)&msg, &len);
		netbuf_delete(buf);

	}
}

/*
static void client_thread(void *arg)
{
	ip_addr_t dst_ip;
	struct netconn *conn;
	struct netbuf *buf;

	LWIP_UNUSED_ARG(arg);
	conn = netconn_new(NETCONN_UDP);
	//LWIP_ERROR("udpecho: invalid conn", (conn != NULL), return;);

	char *msg = "ichiro";
	buf = netbuf_new();
	netbuf_ref(buf,msg,10);

	IP4_ADDR(&dst_ip, 192, 168, 1, 64);

	while (1)
	{
		netconn_sendto(conn, buf, &dst_ip, 50007);
	}
}*/


/*-----------------------------------------------------------------------------------*/


void udpecho_init(void)
{
  //sys_thread_new("udpecho_thread", udpecho_thread, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
	//sys_thread_new("client", client_thread, NULL, 300, 1);

	DAC_PIT_INIT();
    sys_thread_new("server", server_thread, NULL, 300, 2);
    sys_thread_new("server1", server_thread1, NULL, 300, 2);
 //   xTaskCreate(dac_pit, "WRITE_TASK_2", configMINIMAL_STACK_SIZE + 150, NULL, 4, NULL); //CREADA
}

#endif /* LWIP_NETCONN */
