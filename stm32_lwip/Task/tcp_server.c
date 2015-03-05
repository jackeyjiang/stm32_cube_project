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
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx.h"
#include "tcp_server.h"
#include "crc_16.h"

#include "lwip/opt.h"

#if LWIP_NETCONN

#include "lwip/sys.h"
#include "lwip/api.h"

#define TCPECHO_THREAD_PRIO   ( tskIDLE_PRIORITY + 7 ) /*需要注意任务优先级的分配，与各任务之间的调用*/


/*sign in union*/
__attribute__ ((aligned (1)))
//union _SignIn_Union SignIn_Union;

/*TCP client send data mark  */    
u8_t tcp_client_flag;	     
u8_t tcp_client_recvbuf[TCP_CLIENT_RX_BUFSIZE];	//TCP receive buf
//u8_t *tcp_client_sendbuf="Explorer STM32F407 NETCONN TCP Client send data\r\n";	//TCP client send buf

/*TAG table Value detail*/
uint8_t  Tid[TidLen] = {0x10,0x00,0x00,0x06};
uint8_t  NomiNum[NomiNumLen]={0x10,0x00,0x01};
uint8_t  Bno[BnoLen]={0x00,0x00,0x00};
uint8_t  Brw[BrwLen]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t  DealData[DealDataLen]={0x00,0x00,0x00,0x00};
uint8_t  DealTime[DealTimeLen]={0x00,0x00,0x00};
uint8_t  DevArae[DevAraeLen]={0x17,0x03,0x02};
uint8_t  DevSite[DevSiteLen]={0x17,0x03,0x02,0x07};
uint8_t  AppVer[AppVerLen]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01};
uint8_t  ParaFileVer[ParaFileVerLen]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01};
uint8_t  BusiDatFileVer[BusiDatFileVerLen]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01};
uint8_t  CkSta[CkStaLen]={0x00,0x00};
uint8_t  CoinNum[CoinNumLen]={0x00,0x00};
uint8_t  DevStatu[DevStatuLen]={0x00,0x00};

static uint8_t decode_signin_data(uint8_t *p);
static uint8_t decode_mealcomp_data(uint8_t *p);
static uint8_t decode_statupload_data(uint8_t *p);
static uint8_t decode_getmeal_data(uint8_t *p);

/**
  * @brief  获取签到结构提的数据
  * @param  NewState: new state of the Prefetch Buffer.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
uint16_t get_signin_data(uint8_t *p)
{
  //union _SignIn_Union SignIn_Union;
  SignIn_Union   *pSignIn_Union;
  pSignIn_Union = (SignIn_Union *)malloc(sizeof(SignIn_Union));
  /*--GetSignInPacketBuff--TAG value and length*/
  pSignIn_Union->SignIn.Tid = TidChl;
  pSignIn_Union->SignIn.Tid_Len[0] = (TidLen&0xFF00)>>4;
  pSignIn_Union->SignIn.Tid_Len[1] =  TidLen&0x00FF;
  pSignIn_Union->SignIn.NomiNum = NomiNumChl;
  pSignIn_Union->SignIn.NomiNum_Len[0] = (NomiNumLen&0xFF00)>>4;
  pSignIn_Union->SignIn.NomiNum_Len[1] = NomiNumLen&0x00FF;
  pSignIn_Union->SignIn.Brw = BrwChl;
  pSignIn_Union->SignIn.Brw_Len[0] = (BrwLen&0xFF00)>>4;;
  pSignIn_Union->SignIn.Brw_Len[1] = BrwLen&0x00FF;
  pSignIn_Union->SignIn.Bno = BnoChl;
  pSignIn_Union->SignIn.Bno_Len[0] = (BnoLen&0xFF00)>>4;
  pSignIn_Union->SignIn.Bno_Len[1] = BnoLen&0x00FF;
  pSignIn_Union->SignIn.DealData = DealDataChl;
  pSignIn_Union->SignIn.DealData_Len[0] = (DealDataLen&0xFF00)>>4;
  pSignIn_Union->SignIn.DealData_Len[1] = DealDataLen&0x00FF;
  pSignIn_Union->SignIn.DealTime = DealTimeChl;
  pSignIn_Union->SignIn.DealTime_Len[0] = (DealTimeLen&0xFF00)>>4;
  pSignIn_Union->SignIn.DealTime_Len[1] = DealTimeLen&0x00FF;
  pSignIn_Union->SignIn.DevArae = DevAraeChl;
  pSignIn_Union->SignIn.DevArae_Len[0] = (DevAraeLen&0xFF00)>>4;
  pSignIn_Union->SignIn.DevArae_Len[1] = DevAraeLen&0x00FF;
  pSignIn_Union->SignIn.DevSite = DevSiteChl;
  pSignIn_Union->SignIn.DevSite_Len[0] = (DevSiteLen&0xFF00)>>4;
  pSignIn_Union->SignIn.DevSite_Len[1] = DevSiteLen&0x00FF;
  pSignIn_Union->SignIn.AppVer = AppVerChl;
  pSignIn_Union->SignIn.AppVer_Len[0] = (AppVerLen&0xFF00)>>4;
  pSignIn_Union->SignIn.AppVer_Len[1] = AppVerLen&0x00FF;
  pSignIn_Union->SignIn.ParaFileVer = ParaFileVerChl;
  pSignIn_Union->SignIn.ParaFileVer_Len[0] = (ParaFileVerLen&0xFF00)>>4;
  pSignIn_Union->SignIn.ParaFileVer_Len[1] = ParaFileVerLen&0x00FF;
  pSignIn_Union->SignIn.BusiDatFileVer = BusiDatFileVerChl;
  pSignIn_Union->SignIn.BusiDatFileVer_Len[0] = (BusiDatFileVerLen&0xFF00)>>4;
  pSignIn_Union->SignIn.BusiDatFileVer_Len[1] = BusiDatFileVerLen&0x00FF;
  pSignIn_Union->SignIn.CkSta = CkStaChl;
  pSignIn_Union->SignIn.CkSta_Len[0] = (CkStaLen&0xFF00)>>4;; 
  pSignIn_Union->SignIn.CkSta_Len[1] = CkStaLen&0x00FF;  
  /*Get the TAG detail*/
  memcpy(pSignIn_Union->SignIn.Tid_Chl,Tid,sizeof(Tid));
  memcpy(pSignIn_Union->SignIn.NomiNum_Chl,NomiNum,sizeof(NomiNum));
  memcpy(pSignIn_Union->SignIn.Bno_Chl,Bno,sizeof(Bno));
  memcpy(pSignIn_Union->SignIn.Brw_Chl,Brw,sizeof(Brw));
  memcpy(pSignIn_Union->SignIn.DealData_Chl,DealData,sizeof(DealData));
  memcpy(pSignIn_Union->SignIn.DealTime_Chl,DealTime,sizeof(DealTime));
  memcpy(pSignIn_Union->SignIn.DevArae_Chl,DevArae,sizeof(DevArae));
  memcpy(pSignIn_Union->SignIn.DevSite_Chl,DevSite,sizeof(DevSite));
  memcpy(pSignIn_Union->SignIn.AppVer_Chl,AppVer,sizeof(AppVer));
  memcpy(pSignIn_Union->SignIn.ParaFileVer_Chl,ParaFileVer,sizeof(ParaFileVer));
  memcpy(pSignIn_Union->SignIn.BusiDatFileVer_Chl,BusiDatFileVer,sizeof(BusiDatFileVer));
  memcpy(pSignIn_Union->SignIn.CkSta_Chl,CkSta,sizeof(CkSta));
  
  memcpy(p,pSignIn_Union,sizeof(SignIn_Union));
  
  free(pSignIn_Union);
  return  sizeof(SignIn_Union);
}

/**
  * @brief  获取餐品对比结构体数据
  * @param  NewState: new state of the Prefetch Buffer.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
uint16_t get_mealcompar_data(uint8_t *p)
{}
  
/**
  * @brief  获取状态上送结构体数据
  * @param  NewState: new state of the Prefetch Buffer.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
uint16_t get_statupload_data(uint8_t *p)
{
  StatuUpload_Union *pStatuUpload_Union;
  pStatuUpload_Union = (StatuUpload_Union *)malloc(sizeof(pStatuUpload_Union));
  /*--GetStatuUploadPacketBuff--TAG value and length*/
  pStatuUpload_Union->StatuUpload.Tid = TidChl;
  pStatuUpload_Union->StatuUpload.Tid_Len[0] = (TidLen&0xFF00)>>4;
  pStatuUpload_Union->StatuUpload.Tid_Len[1] =  TidLen&0x00FF;
  pStatuUpload_Union->StatuUpload.Brw = BrwChl;
  pStatuUpload_Union->StatuUpload.Brw_Len[0] = (BrwLen&0xFF00)>>4;;
  pStatuUpload_Union->StatuUpload.Brw_Len[1] = BrwLen&0x00FF;
  pStatuUpload_Union->StatuUpload.Bno = BnoChl;
  pStatuUpload_Union->StatuUpload.Bno_Len[0] = (BnoLen&0xFF00)>>4;
  pStatuUpload_Union->StatuUpload.Bno_Len[1] = BnoLen&0x00FF;  
  pStatuUpload_Union->StatuUpload.CoinNum = CoinNumChl;
  pStatuUpload_Union->StatuUpload.CoinNum_Len[0] = (CoinNumLen&0xFF00)>>4;
  pStatuUpload_Union->StatuUpload.CoinNum_Len[1] = CoinNumLen&0x00FF;  
  pStatuUpload_Union->StatuUpload.DevStatu = DevStatuChl;
  pStatuUpload_Union->StatuUpload.DevStatu_Len[0] = (DevStatuLen&0xFF00)>>4;
  pStatuUpload_Union->StatuUpload.DevStatu_Len[1] = DevStatuLen&0x00FF; 
  
  memcpy(p,pStatuUpload_Union,sizeof(StatuUpload_Union));
  free(pStatuUpload_Union);
  return  sizeof(StatuUpload_Union);
}
  
/**
  * @brief  获取取餐结构体数据
  * @param  NewState: new state of the Prefetch Buffer.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
uint16_t get_getmeal_data(uint8_t *p)
{}
  
/**
  * @brief  将签到的数据指向sendbuff,这样就可以避免复制了
  * @param  NewState: new state of the Prefetch Buffer.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */

PACKET_STRUCT pcket_struct;   //帧的封装格式
static void package_buff(uint16_t Request,uint8_t *request_buf)
{
  uint16_t crc_value;
  uint16_t contex_lenth=0;
  pcket_struct.Stx = StxChl;  //获取帧头
  pcket_struct.Etx = EtxChl;  //获取帧尾
  pcket_struct.CmdReq[0]= ((Request&0xff00)>>8);  //获取请求
  pcket_struct.CmdReq[1]=  (Request&0x00ff);        
  memcpy(request_buf,&pcket_struct.Stx,sizeof(pcket_struct.Stx)); //复制单个使用地址 ：帧头
  memcpy(request_buf+1,pcket_struct.CmdReq,sizeof(pcket_struct.CmdReq)); //复制签到请求命令 
  switch(Request)
  {
    case SignInReq:{
      contex_lenth = get_signin_data(request_buf+5); 
      //将签到用到的结构体中的数据写入sign_in_buf中，偏移5个位置，返回签到结构体的字节数 
    }break;
    case MealComparReq:{
      contex_lenth = get_mealcompar_data(request_buf+5); 
      //将签到用到的结构体中的数据写入sign_in_buf中，偏移5个位置，返回签到结构体的字节数 
    }break;
    case StatuUploadReq:{
      contex_lenth = get_statupload_data(request_buf+5); 
      //将签到用到的结构体中的数据写入sign_in_buf中，偏移5个位置，返回签到结构体的字节数     
    }break;
    case GetMealReq:{
      contex_lenth = get_getmeal_data(request_buf+5); 
      //将签到用到的结构体中的数据写入sign_in_buf中，偏移5个位置，返回签到结构体的字节数        
    }break;
    default:break;   
  }
  pcket_struct.FrameLen[0]= ((contex_lenth&0xff00)>>8);
  pcket_struct.FrameLen[1]= (contex_lenth&0x00ff);
  memcpy(request_buf+3,pcket_struct.FrameLen,sizeof(pcket_struct.FrameLen)); //复制签到请求内容长度
  memcpy(request_buf+5+contex_lenth,&pcket_struct.Etx,sizeof(pcket_struct.Etx)); //复制单个使用地址 ：帧尾
  crc_value =  crc_ccitt(request_buf+1,contex_lenth+4); //获取Crc的值，从帧头到帧尾
  pcket_struct.Crc[0]= ((crc_value&0xff00)>>8);
  pcket_struct.Crc[1]= (crc_value&0x00ff);
  memcpy(request_buf+6+contex_lenth,pcket_struct.Crc,sizeof(pcket_struct.Crc)); //复制CRC值    
}

/**
  * @brief  解码重服务器返回的数据
  * @param  NewState: new state of the Prefetch Buffer.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
uint8_t decode_host_data()
{
  uint8_t err;
  static uint16_t res_cmd = 0,res_len = 0,crc_value=0,crc_value_calcu=0;
  if(tcp_client_recvbuf[0]!= StxChl)   return 1;  //判断帧头
  res_cmd = (tcp_client_recvbuf[1]<<8) + tcp_client_recvbuf[2];  //获取响应命令
  res_len = (tcp_client_recvbuf[3]<<8) + tcp_client_recvbuf[4];  //获取长度数据
  if(tcp_client_recvbuf[res_len+5]!= EtxChl)    return 2;  //判断帧尾
  crc_value = (tcp_client_recvbuf[res_len+6]<<8) + tcp_client_recvbuf[res_len+7];  //获取CRC16的值
  crc_value_calcu = crc_ccitt(&tcp_client_recvbuf[1],res_len+4);//计算CRC的值
  //if(crc_value != crc_value_calcu) return 3;  //判断CRC,校验数据的完整性 ,校验出错，不批匹配，不知道
  switch(res_cmd)
  {
    case SignInRes:
    {
      err = decode_signin_data(&tcp_client_recvbuf[5]);
    };break;
    case MealComparRes:
    {
      err = decode_mealcomp_data(&tcp_client_recvbuf[5]);
    };break;    
    case StatuUploadRes:
    {
      err = decode_statupload_data(&tcp_client_recvbuf[5]);
    };break;
    case GetMealRes:
    {
      err = decode_getmeal_data(&tcp_client_recvbuf[5]);
    };break;  
    default:
    {
      return 4;  //返回的命令请求错误
    }
  }
  return err;
}

/**
  * @brief  解码四种有服务器返回的数据
  * @param  NewState: new state of the Prefetch Buffer.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
SIGN_IN_REQ_STRUCT sigin_req;
Meal_Union meal_union;
static uint8_t decode_signin_data(uint8_t *p)
{
  uint32_t byte_count=0;
  if(p[byte_count++]==NomiNameChl)
  {  
    sigin_req.NomiName =NomiNameChl;
    sigin_req.NomiName_Len = (p[byte_count++]<<4);
    sigin_req.NomiName_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.NomiName_Len;i++)
    {   
      sigin_req.NomiName_Chl[i] = p[byte_count++];
    }
  }
  if(p[byte_count++]==BnoChl)
  {  
    sigin_req.Bno =BrwChl;
    sigin_req.Bno_Len = (p[byte_count++]<<4);
    sigin_req.Bno_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.Bno_Len;i++)
    {   
      sigin_req.Bno_Chl[i] = p[byte_count++];
    }
  }
  if(p[byte_count++]==AppVerChl)
  {  
    sigin_req.AppVer =AppVerChl;
    sigin_req.AppVer_Len = (p[byte_count++]<<4);
    sigin_req.AppVer_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.AppVer_Len;i++)
    {   
      sigin_req.AppVer_Chl[i] = p[byte_count++];
    }
  }
  if(p[byte_count++]==ParaFileVerChl)
  {  
    sigin_req.ParaFileVer =ParaFileVerChl;
    sigin_req.ParaFileVer_Len = (p[byte_count++]<<4);
    sigin_req.ParaFileVer_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.ParaFileVer_Len;i++)
    {   
      sigin_req.ParaFileVer_Chl[i] = p[byte_count++];
    }
  }
  if(p[byte_count++]==BusiDatFileVerChl)
  {  
    sigin_req.BusiDatFileVer =BusiDatFileVerChl;
    sigin_req.BusiDatFileVer_Len = (p[byte_count++]<<4);
    sigin_req.BusiDatFileVer_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.BusiDatFileVer_Len;i++)
    {   
      sigin_req.BusiDatFileVer_Chl[i] = p[byte_count++];
    }
  }
  if(p[byte_count++]==UpDatFlagChl)
  {  
    sigin_req.UpDatFlag =UpDatFlagChl;
    sigin_req.UpDatFlag_Len = (p[byte_count++]<<4);
    sigin_req.UpDatFlag_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.UpDatFlag_Len;i++)
    {   
      sigin_req.UpDatFlag_Chl[i] = p[byte_count++];
    }
  }
  if(p[byte_count++]==MenuNumChl)
  {  
    sigin_req.MenuNum =MenuNumChl;
    sigin_req.MenuNum_Len = (p[byte_count++]<<4);
    sigin_req.MenuNum_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.MenuNum_Len;i++)
    {   
      sigin_req.MenuNum_Chl[i] = p[byte_count++];
    }
  }
  if(p[byte_count++]==MenuDetailChl)
  {  
    sigin_req.MenuDetail =MenuDetailChl;
    sigin_req.MenuDetail_Len = (p[byte_count++]<<4);
    sigin_req.MenuDetail_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.MenuDetail_Len;i++)
    {   
      //sigin_req.MenuDetail_Chl[i] = p[byte_count++];
      meal_union.MealBuf[i] = p[byte_count++];
    }
  }
  if(p[byte_count++]==AckChl)
  {  
    sigin_req.Ack =AckChl;
    sigin_req.Ack_Len = (p[byte_count++]<<4);
    sigin_req.Ack_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.Ack_Len;i++)
    {   
      sigin_req.Ack_Chl[i] = p[byte_count++];
    }
  }
  if(p[byte_count++]==CipherChl)
  {  
    sigin_req.Cipher =CipherChl;
    sigin_req.Cipher_Len = (p[byte_count++]<<4);
    sigin_req.Cipher_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.Cipher_Len;i++)
    {   
      sigin_req.Cipher_Chl[i] = p[byte_count++];
    }
  }  
  return 0;
}
static uint8_t decode_mealcomp_data(uint8_t *p)
{
  return 0;
}
static uint8_t decode_statupload_data(uint8_t *p)
{
  uint32_t byte_count=0;
  if(p[byte_count++]==AckChl)
  {  
    sigin_req.Ack =AckChl;
    sigin_req.Ack_Len = (p[byte_count++]<<4);
    sigin_req.Ack_Len += (p[byte_count++]); 
    for(uint8_t i=0;i<sigin_req.Ack_Len;i++)
    {   
      sigin_req.Ack_Chl[i] = p[byte_count++];
    }
  }
  return 0;
}
static uint8_t decode_getmeal_data(uint8_t *p)
{
  return 0;
}

/**
  * @brief  向服务器发送签到数据
  * @param  NewState: new state of the Prefetch Buffer.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
extern xQueueHandle xQueue;
/*-----------------------------------------------------------------------------------*/
static void tcpclient_thread(void *arg)
{
  struct netconn *tcp_clientconn;
  struct ip_addr ServerIPaddr;
  struct pbuf *q;
  static u32_t data_len = 0;
  static u16_t lReceivedValue;
  portBASE_TYPE xStatus;
  const portTickType xTicksToWait = 10 / portTICK_RATE_MS;
  err_t err;

  LWIP_UNUSED_ARG(arg);
  
  /* add SERVER_IP to ServerIPaddr*/
  IP4_ADDR( &ServerIPaddr, SERVER_IP_ADDR0, SERVER_IP_ADDR1, SERVER_IP_ADDR2, SERVER_IP_ADDR3 );
#if 1
  while(1)
  {
    /* Create a new connection identifier. */
      tcp_clientconn = netconn_new(NETCONN_TCP);
   
      if (tcp_clientconn!=NULL)
      {
        /*timeout to wait for new data to be received <Avoid death etc.> */
        tcp_clientconn->recv_timeout= 10;  
          /*built a connect to server*/
        err = netconn_connect(tcp_clientconn,&ServerIPaddr,SERVER_PORT);
         
        if (err == ERR_OK)
        {
           struct netbuf *recvbuf;
           while(1)
           {
              //if((tcp_client_flag & LWIP_SEND_DATA) == LWIP_SEND_DATA)  
              xStatus = xQueueReceive( xQueue,
                                       &lReceivedValue,
                                       xTicksToWait );
              if(xStatus == pdPASS)
              {
                switch(lReceivedValue)
                {
                  case SignInReq:{
                    auto uint8_t sign_in_buf[Totoal_SignIn_Lenth+ 8]={0};  //使用auto，该类具有自动存储期
                    //static uint8_t * psign_in_buf;  /*不知道为什么没有办法将结构体的数据挂上*/
                    //psign_in_buf = (uint8_t *)malloc(Totoal_SignIn_Lenth+ 8*sizeof(uint8_t));   
                    package_buff(SignInReq,sign_in_buf);   //将要发送的数据填入sendbuf        
                    err=netconn_write(tcp_clientconn,\
                    sign_in_buf,sizeof(sign_in_buf),\
                    NETCONN_MORE); 
                    if(err != ERR_OK)printf("err \r\n");      
                  };break;
                  case MealComparReq:{
                    auto uint8_t statu_upload_buf[Totoal_StatuUpload_Lenth+ 8]={0};  //使用auto，该类具有自动存储期
                    package_buff(MealComparReq,statu_upload_buf);   //将要发送的数据填入sendbuf        
                    err=netconn_write(tcp_clientconn,\
                    statu_upload_buf,sizeof(statu_upload_buf),\
                    NETCONN_MORE); 
                    if(err != ERR_OK)printf("err \r\n");                        
                  };break;
                  case StatuUploadReq:{
                    
                  };break;
                  case GetMealReq:{
                  };break;
                  default:break;
                }   
              }
              if(netconn_recv(tcp_clientconn, &recvbuf) == ERR_OK) 
              {         
                  memset(tcp_client_recvbuf,0,TCP_CLIENT_RX_BUFSIZE);       
                  for(q=recvbuf->p;q!=NULL;q=q->next)     
                  {           
                      if(q->len>(TCP_CLIENT_RX_BUFSIZE-data_len))memcpy(\
                          tcp_client_recvbuf+data_len,q->payload,\
                          (TCP_CLIENT_RX_BUFSIZE-data_len));     
                      else memcpy(tcp_client_recvbuf+data_len,q->payload,q->len);
                      data_len += q->len;           
                      if(data_len > TCP_CLIENT_RX_BUFSIZE) break;         
                  }      
                  data_len=0;
                  //int i;
                  //for(i=0;i<TCP_CLIENT_RX_BUFSIZE;i++)                  
                  printf("ACK 0x%02X \r\n",tcp_client_recvbuf[0]);
                  //vPrintString(tcp_client_recvbuf,TCP_CLIENT_RX_BUFSIZE);                  
                  netbuf_delete(recvbuf);
                  /*解码重服务器返回的数据*/
                  decode_host_data();                  
              }
           }
        }
     }     
  }
#endif
}

/*-----------------------------------------------------------------------------------*/

void tcpclient_init(void)
{
  sys_thread_new("tcpecho_thread", tcpclient_thread, NULL, DEFAULT_THREAD_STACKSIZE, TCPECHO_THREAD_PRIO);
}
/*-----------------------------------------------------------------------------------*/

#endif /* LWIP_NETCONN */


