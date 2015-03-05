
/**
  ******************************************************************************
  * File Name          : lwipopts.h
  * Date               : 08/02/2015 01:35:03
  * Description        : This file overrides LwIP stack default configuration
  *                      done in opt.h file.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LWIPOPTS__H__
#define __LWIPOPTS__H__

#include "stm32f4xx_hal.h"

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#ifdef __cplusplus
 extern "C" {
#endif

/*----- Default Value for LWIP_DHCP: 0 -----*/  
#define LWIP_DHCP   1
/*----- Default Value for WITH_RTOS:  -----*/
#define WITH_RTOS   1		
/*----- Default Value for NO_SYS: 0 -----*/ 
#define NO_SYS   0			
/*----- Default Value for MEM_ALIGNMENT: 1 -----*/ 
#define MEM_ALIGNMENT   4
/*----- Default Value for LWIP_SNMP: 0 -----*/
#define LWIP_SNMP   1		
/*----- Default Value for LWIP_NETCONN: 1 -----*/ 
#define LWIP_NETCONN   1		
/*----- Default Value for LWIP_SOCKET: 1 -----*/
#define LWIP_SOCKET   0		
/*----- Default Value for LWIP_COMPAT_SOCKETS: 1 -----*/
#define LWIP_COMPAT_SOCKETS   0		
/*----- No Default Value for LWIP_PROVIDE_ERRNO -----*/
#define LWIP_PROVIDE_ERRNO  1

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /*__ LWIPOPTS__H_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
