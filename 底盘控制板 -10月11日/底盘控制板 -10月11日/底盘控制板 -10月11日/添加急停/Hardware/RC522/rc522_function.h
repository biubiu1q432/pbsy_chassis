#ifndef __RC522_FUNCTION_H
#define	__RC522_FUNCTION_H


#include "stm32f10x_it.h"
/////////////////////////////////////////////////////////////////////
//和MF522通讯时返回的错误代码
/////////////////////////////////////////////////////////////////////


#define          macDummy_Data              0x00

void SPI1_Init(void);
void RC522_Init(void);
char PcdReset(void);
void WriteRawRC(unsigned char Address, unsigned char value);
unsigned char ReadRawRC(unsigned char Address);  
void PcdAntennaOn(void);
void PcdAntennaOff(void);
char PcdRequest(unsigned char req_code,unsigned char *pTagType);
char PcdAnticoll(unsigned char *pSnr);
char PcdSelect(unsigned char *pSnr);
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr);
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue);
char PcdRead(unsigned char addr,unsigned char *pData);
char PcdHalt(void);
char PcdBakValue(unsigned char sourceaddr, unsigned char goaladdr);
char PcdWrite(unsigned char addr,unsigned char *pData);
#endif /* __RC522_FUNCTION_H */

