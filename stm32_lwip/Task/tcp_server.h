#include "stdint.h"

__attribute__ ((aligned (1)))
typedef struct _COMMUNICATE_PACKET_STRUCT
{
	uint8_t Tid;             /*终端TID码*/
	uint8_t Tid_Len[2];      
	uint8_t Brw;             /*批次号*/
	uint8_t Brw_Len[2];
	uint8_t Bno;						 /*交易流水线*/
	uint8_t Bno_Len[2];
	uint8_t DevArae;         /*终端所在区域编号*/
	uint8_t DevArae_Len[2];
	uint8_t DevAreaNum;      /*终端所在地域编号*/
	uint8_t DevAreaNum_Len[2];
	uint8_t DevStu;          /*设备状态*/
	uint8_t DevStu_Len[2];   
	uint8_t DealData;        /*交易日期*/
	uint8_t DealData_Len[2];
	uint8_t DealTime;        /*交易时间*/
	uint8_t DealTime_Len[2];
	uint8_t Mac;             /*物理地址*/
	uint8_t Mac_Len[2];	
	uint8_t MealId;          /*餐品ID*/
	uint8_t MealId_Len[2];	 
  uint8_t MealQty;         /*餐品购买数量*/
	uint8_t MealQty_Len[2];
	uint8_t MealName;        /*餐品名字*/
	uint8_t MealName_Len[2];
	uint8_t MealPrice;       /*餐品价格*/
	uint8_t MealPrice_Len[2];
	uint8_t PayType;         /*支付方式*/
	uint8_t PayType_Len[2];
	uint8_t Change;          /*找零金额*/
	uint8_t Change_Len[2];
	uint8_t RmnMealQty;      /*剩余餐品数量*/
	uint8_t RmnMealQty_Len[2];
	uint8_t NomiName;        /*商户名*/
	uint8_t NomiName_Len[2];
	uint8_t NomiNum;         /*商户号*/
	uint8_t NomiNum_Len[2];
	uint8_t AppVer;          /*应用程序版本*/
	uint8_t AppVer_Len[2]; 
	uint8_t ParaFileVer;     /*参数文件版本*/ 
	uint8_t ParaFileVer_Leng[2];
	uint8_t BusiDatFileVer;  /*业务数据文件版本*/
	uint8_t BusiDatFileVer_Len[2];
	uint8_t CkSta;           /*自检状态*/
	uint8_t CkSta_Len[2];
	uint8_t TkMealFlag;      /*取餐标志*/
	uint8_t TkMealFlag_Len[2];
	uint8_t PosDevNum;       /*刷卡器终端号*/
	uint8_t PosDevNum_Len[2];
	uint8_t PosTenantNum;    /*刷卡器商户号*/
	uint8_t PosTenantNum_Len[2];
	uint8_t PosBatchNum;     /*刷卡器交易流水号*/
	uint8_t PosBatchNum_Len[2];
	uint8_t PosUserNum;      /*用户银行卡号*/ 
	uint8_t PosUserNum_Len[2];
	uint8_t UpDatFlag;       /*更新标识*/
	uint8_t UpDatFlag_Len[2];
	uint8_t MenuNum;         /*菜单编号*/ 
	uint8_t MenuNum_Len[2];
	uint8_t Ack;             /*应答码*/
	uint8_t Ack_Len[2];
	uint8_t Cipher;          /*密钥*/
	uint8_t Cipher_Len[2];
}COMMUNICATE_PACKET_STRUCT,communicate_packet; //与服务器交换数据的TAG值表


#define StxChl         0x02    /*帧头：0x02*/
#define EtxChl         0x03    /*帧尾：0x03*/
#define SignInReq      0x0100  /*签到请求*/
#define SignInRes      0x0110  /*签到响应*/
#define MealComparReq  0x0200  /*餐品对比请求*/
#define MealComparRes  0x0210  /*餐品对比响应*/
#define StatuUploadReq 0x0400  /*状态上送请求*/
#define StatuUploadRes 0x0410  /*状态上送响应*/
#define GetMealReq     0x0800  /*取餐数据上传请求*/
#define GetMealRes     0x0810  /*取餐数据上传响应*/

typedef struct _PACKET_STRUCT
{
	uint8_t  Stx;          						/*帧头：0x02*/
	uint8_t  CmdReq[2];   		    				/*命令码*/
	uint8_t  FrameLen[2];        				/*帧长度*/
	uint8_t  Etx;             				/*帧尾：0x03*/
	uint8_t  Crc[2];          				/*CRC校验码*/
}PACKET_STRUCT;  /*帧的格式*/
extern PACKET_STRUCT pcket_struct;


/*SignInReq*/
#define TidChl              0xA2
#define TidLen              4
#define NomiNumChl          0xA3
#define NomiNumLen          3
#define BrwChl              0xA6
#define BrwLen              7
#define BnoChl              0xA7
#define BnoLen              3
#define DealDataChl         0xA9
#define DealDataLen         4
#define DealTimeChl         0xAA
#define DealTimeLen         3
#define DevAraeChl          0xAC 
#define DevAraeLen          3
#define DevSiteChl          0xAD
#define DevSiteLen          4
#define AppVerChl           0xC2 
#define AppVerLen           8
#define ParaFileVerChl      0xC3 
#define ParaFileVerLen      8 
#define BusiDatFileVerChl   0xC4
#define BusiDatFileVerLen   8
#define CkStaChl            0xAB
#define CkStaLen            2

#define Totoal_SignIn_Lenth TidLen+NomiNumLen+BnoLen+BrwLen+DealDataLen+DealTimeLen+DevAraeLen\
                            +DevSiteLen+AppVerLen+ParaFileVerLen+BusiDatFileVerLen+CkStaLen+12*3

__attribute__ ((aligned (1))) /*异常，大小端与堆栈，不用uint16_t*/
typedef struct _SIGN_IN_STRUCT
{
	uint8_t  Tid;             /*终端TID码*/
	uint8_t  Tid_Len[2]; 
	uint8_t  Tid_Chl[TidLen]; 
	uint8_t  NomiNum;         /*商户号*/
	uint8_t  NomiNum_Len[2];	
	uint8_t  NomiNum_Chl[NomiNumLen];		
	uint8_t  Bno;						 /*交易流水线*/
	uint8_t  Bno_Len[2];
	uint8_t  Bno_Chl[BnoLen];
	uint8_t  Brw;             /*批次号*/
	uint8_t  Brw_Len[2];
	uint8_t  Brw_Chl[BrwLen];
	uint8_t  DealData;        /*交易日期*/
	uint8_t  DealData_Len[2];
	uint8_t  DealData_Chl[DealDataLen];
	uint8_t  DealTime;        /*交易时间*/
	uint8_t  DealTime_Len[2];
	uint8_t  DealTime_Chl[DealTimeLen];
	uint8_t  DevArae;         /*终端所在区域编号*/ 
	uint8_t  DevArae_Len[2];
	uint8_t  DevArae_Chl[DevAraeLen];
	uint8_t  DevSite;         /*终端所在区域地点编号*/
	uint8_t  DevSite_Len[2];
	uint8_t  DevSite_Chl[DevSiteLen]; 
	uint8_t  AppVer;          /*应用程序版本*/
	uint8_t  AppVer_Len[2];
	uint8_t  AppVer_Chl[AppVerLen]; 	
	uint8_t  ParaFileVer;     /*参数文件版本*/ 
	uint8_t  ParaFileVer_Len[2];
	uint8_t  ParaFileVer_Chl[ParaFileVerLen];
	uint8_t  BusiDatFileVer;  /*业务数据文件版本*/
	uint8_t  BusiDatFileVer_Len[2];
	uint8_t  BusiDatFileVer_Chl[BusiDatFileVerLen];
	uint8_t  CkSta;           /*自检状态*/
	uint8_t  CkSta_Len[2];	
	uint8_t  CkSta_Chl[CkStaLen];
}SIGN_IN_STRUCT; /*请求签到帧的数据格式*/


typedef union _SignIn_Union
{
  SIGN_IN_STRUCT SignIn;
  uint8_t SignInBuf[Totoal_SignIn_Lenth];
}SignIn_Union;
//extern union _SignIn_Union SignIn_Union;

/*SignInRes*/
#define NomiNameChl         0xA4
#define NomiNameLen         20
#define AppVerChl           0xC2
#define AppVerLen           8               
#define ParaFileVerChl      0xC3
#define ParaFileVerLen      8
#define BusiDatFileVerChl   0xC4
#define BusiDatFileVerLen   8
#define UpDatFlagChl        0xC1
#define UpDatFlagLen        1
#define MenuNumChl          0xDD
#define MenuNumLen          1    /*变长*/
#define MenuDetailChl       0xDF
#define MenuDetailLen       4*47 /*变长*/ 
#define AckChl              0xC0
#define AckLen              2
#define CipherChl           0xC7
#define CipherLen           8

typedef struct _SIGN_IN_REQ_STRUCT
{
	uint8_t  NomiName;              /*商户名*/
	uint16_t NomiName_Len; 
  uint8_t  NomiName_Chl[NomiNameLen]; 
	uint8_t  Bno;                   /*批次号*/
	uint16_t Bno_Len;
  uint8_t  Bno_Chl[BnoLen];
	uint8_t  AppVer;                /*应用程序版本*/
	uint16_t AppVer_Len; 
  uint8_t  AppVer_Chl[AppVerLen]; 
	uint8_t  ParaFileVer;           /*参数文件版本*/ 
	uint16_t ParaFileVer_Len; 
  uint8_t  ParaFileVer_Chl[ParaFileVerLen];
	uint8_t  BusiDatFileVer;        /*业务数据文件版本*/
	uint16_t BusiDatFileVer_Len;
  uint8_t  BusiDatFileVer_Chl[BusiDatFileVerLen];
	uint8_t  UpDatFlag;             /*更新标识*/
	uint16_t UpDatFlag_Len; 
  uint8_t  UpDatFlag_Chl[UpDatFlagLen];  
  uint8_t  MenuNum;               /*菜单编号*/
  uint16_t MenuNum_Len;
  uint8_t  MenuNum_Chl[MenuNumLen];
  uint8_t  MenuDetail;               /*菜单编号*/
  uint16_t MenuDetail_Len;
  uint8_t  MenuDetail_Chl[MenuDetailLen];
	uint8_t  Ack;                   /*应答码*/
	uint16_t Ack_Len;
  uint8_t  Ack_Chl[AckLen];
	uint8_t  Cipher;                /*密钥*/
	uint16_t Cipher_Len;
  uint8_t  Cipher_Chl[CipherLen];
}SIGN_IN_REQ_STRUCT;


/*******************************************
下发餐品详细的结构体

********************************/
#define FloorMealNum    15				   //一共15层
#define MealKindTotoal  4            // 有四种餐品

typedef struct _Meal_struction
{
	uint8_t MealID[4];     
	uint8_t MealName[20];  
  uint8_t MealCnt[2];    
	uint8_t MealPrice[6]; 
	uint8_t MealCut[2];   
  uint8_t VipCut[2];    
  uint8_t SztCut[2];   
  uint8_t GbocCut[2];    
  uint8_t VchatCut[2];  
  uint8_t MealOrder[1];  
	uint8_t MealType[4];   
}Meal_struction;  /*餐品的详细信息*/

typedef union _Meal_Union
{
  Meal_struction Meal[MealKindTotoal];
  uint8_t MealBuf[MealKindTotoal*47];
}Meal_Union;   /*四种餐品的数据*/


#define TradvolChl          0xB0
#define TradvolLen          6
#define MealIdChl           0xB1
#define MealIdLen           6
#define MealQtyChl          0xB2
#define MealQtyLen          6
#define MealNameChl         0xB3
#define MealNameLen         20
#define MealPriceChl        0xBD
#define MealPriceLen        6
#define PayTypeChl          0xBF
#define PayTypeLen          1
#define ChangeChl           0xD7
#define ChangeLen           6
#define RmnMealQtyChl       0xD8
#define RmnMealQtyLen       2
#define TkMealFalgChl       0xDB
#define TkMealFalgLen       1
#define MacChl              0xC9
#define MacLen              1


typedef struct _TAKE_MEAL_STRUCT
{
	uint8_t Tid;             /*终端TID码*/
	uint8_t Tid_Len[2]; 
	uint8_t Tid_Chl[TidLen]; 	
	uint8_t Bno;						 /*交易流水线*/
	uint8_t Bno_Len[2];
	uint8_t Bno_Chl[BnoLen];
	uint8_t Brw;             /*批次号*/
	uint8_t Brw_Len[2];
	uint8_t Brw_Chl[BrwLen];
	uint8_t DealData;        /*交易日期*/
	uint8_t DealData_Len[2];
	uint8_t DealData_Chl[DealDataLen];
	uint8_t DealTime;        /*交易时间*/
	uint8_t DealTime_Len[2];
	uint8_t DealTime_Chl[DealTimeLen];
	uint8_t DevArae;         /*终端所在区域编号*/ 
	uint8_t DevArae_Len[2];
	uint8_t DevArae_Chl[DevAraeLen];
	uint8_t DevSite;         /*终端所在区域地点编号*/
	uint8_t DevSite_Len[2];
	uint8_t DevSite_Chl[DevSiteLen]; 
	uint8_t Tradvol;         /*交易金额*/
	uint8_t Tradvol_Len[2];
	uint8_t Tradvol_Chl[TradvolLen];
	uint8_t MealId;          /*餐品ID*/
	uint8_t MealId_Len[2];
	uint8_t MealId_Chl[MealIdLen];
	uint8_t MealQty;         /*餐品数量*/
	uint8_t MealQty_Len[2];
	uint8_t MealQty_Chl[MealQtyLen];
	uint8_t MealName;        /*餐品名称*/
	uint8_t MealName_Len[2];
	uint8_t MealName_Chl[MealNameLen];	
	uint8_t MealPrice;       /*餐品价格*/
	uint8_t MealPrice_Len[2];
	uint8_t MealPrice_Chl[MealPriceLen];
	uint8_t PayType;         /*支付方式*/
	uint8_t PayType_Len[2];
	uint8_t PayType_Chl[PayTypeLen];	
	uint8_t Change;          /*找零金额*/
	uint8_t Change_Len[2];
	uint8_t Change_Chl[ChangeLen];
	uint8_t RmnMealQty;      /*剩余餐品数量*/
	uint8_t RmnMealQty_Len[2];
	uint8_t RmnMealQty_Chl[RmnMealQtyLen];	
	uint8_t Mac;             /*MAC地址*/
	uint8_t Mac_Len[2];
	uint8_t Mac_Chl[MacLen];	
}TAKE_MEAL_STRUCT;


#define CoinNumChl       0xDE
#define CoinNumLen       2
#define DevStatuChl      0xAE
#define DevStatuLen      2

#define Totoal_StatuUpload_Lenth  TidLen+BrwLen+BnoLen+CoinNumLen+DevStatuLen+5*3

typedef struct _STATU_UPLOAD_STRUCT
{
	uint8_t Tid;             /*终端TID码*/
	uint8_t Tid_Len[2]; 
	uint8_t Tid_Chl[TidLen]; 	
  uint8_t Brw;             /*交易流水号*/
	uint8_t Brw_Len[2];
	uint8_t Brw_Chl[BrwLen];	  
	uint8_t Bno;						 /*交易批次号*/
	uint8_t Bno_Len[2];
	uint8_t Bno_Chl[BnoLen];
	uint8_t CoinNum;				 /*机内硬币数*/
	uint8_t CoinNum_Len[2];
	uint8_t CoinNum_Chl[CoinNumLen];
	uint8_t DevStatu;				 /*设备状态*/
	uint8_t DevStatu_Len[2];
	uint8_t DevStatu_Chl[DevStatuLen];  
}STATU_UPLOAD_STRUCT;

typedef union _StatuUpload_Union
{
  STATU_UPLOAD_STRUCT StatuUpload;
  uint8_t StatuUploafBuf[Totoal_StatuUpload_Lenth];
}StatuUpload_Union;

/*TCP Server Address*/
//#define Test

#ifdef Test
  #define SERVER_IP_ADDR0  192
  #define SERVER_IP_ADDR1  168
  #define SERVER_IP_ADDR2  1
  #define SERVER_IP_ADDR3  7
#else
  #define SERVER_IP_ADDR0  211
  #define SERVER_IP_ADDR1  162
  #define SERVER_IP_ADDR2  73
  #define SERVER_IP_ADDR3  85
#endif

/*TCP Server Port*/
#define SERVER_PORT     9013

/*Define bit have some data to send*/
#define LWIP_SEND_DATA			0X80    

/*Define the tcp_client_recvbuf size*/
#define TCP_CLIENT_RX_BUFSIZE	  512

extern uint8_t tcp_client_flag;

extern struct netif xnetif;

void tcpclient_init(void);

