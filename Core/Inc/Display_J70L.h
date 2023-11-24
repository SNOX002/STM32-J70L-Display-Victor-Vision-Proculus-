#include <stdio.h>      /* printf, scanf, NULL */
#include <stdlib.h>     /* malloc, free, rand */
#include <string.h>
#include <Register_J70L.h>
//#include <Perfilógrafo.h>

//UART_HandleTypeDef *huart = &huart3;
#define VP_LED 20
#define VP_STRING 35

const uint8_t FrameHead[2] = { 0x5A, 0xA5 }; /*<Framehead> <ByteCount>*/

int1 DSP_CMD;
uint8_t  DSP_Buffer[40];
uint8_t RXBuffer[40];

const uint8_t *PtrVP = &DSP_Buffer[0]; 	// Aponta para a memória que fica o número do VP recebido do display

const uint8_t *PtrByteCount = &RXBuffer[2];	//Memória onde está o ByteCount do cabeçalho
const uint8_t *PtrDSP_Buffer = &DSP_Buffer[1];	//Aponta para o byte mais significativo dos VP (que é 16 bits)

uint8_t *PtrString = &DSP_Buffer[4];	//Local de memória onde começa as letras das strings
char Read_Text[20];
char Save_str[20];


void Get_string(char* str_VP) {
    // Package -->
    // <FrameHeadAlto> <FHB> <ByteCount> <0x82> <VPH> <VPL> <FisrtByte String>;
    // PtrString = DSP_Buffer[4]

    uint8_t Size = *PtrByteCount - 4;
    uint8_t i = 0;

    uint8_t* currentPtr = PtrString;  // Apontador para o início do buffer
    uint8_t* endPtr = PtrString + Size;  // Apontador para o final do buffer

    for (i = 0; currentPtr <= endPtr; i++) {
        if (*currentPtr == 0xFF) {
            break;
        }
        str_VP[i] = *currentPtr;
        currentPtr++;
    }

    str_VP[i] = 0;  // Adicionar o caractere nulo
}

uint16_t Get_VP(const uint8_t *DSP_send) {
	//O ponteiro passado já esta apontando para o byte correto do VP
/* Package --> <FrameHeadAlto><FHB> <ByteCount> <0x82> <VPH> <VPL> <ValueHIGH> <ValueLOW>; dados de 16bits (2 bytes)*/

	uint16_t VP = (uint16_t)(*DSP_send << 8);  	// O primeiro byte é deslocado 8 bits para a esquerda
    DSP_send++;  								// Avança para o próximo byte no array
    VP |= (uint16_t)(*DSP_send);  				// O segundo byte é 'ou' (bitwise OR) com o valor existente
    return VP;
    // Agora, VP contém o valor de 16 bits formado pelos dois bytes consecutivos
}

/*		Exemplo de como fazer uso desse código:
 *  chamar DSP_GetCMD() a partir de DSP_MSG();
 */

void DSP_GetCMD(){
	memset(RXBuffer, 0, 4);

		if(DSP_Buffer[0] == Read_REG_VP) //Substituir por Switch
		{
			/*Quem é DSP_Buffer?	Cabeçalho de bytes:
			 * <0x82> <VPH> <VPL> <LEN> <ValueHIGH> <ValueLOW>; dados de 16bits (2 bytes)
			 */
			uint16_t VP = Get_VP(PtrDSP_Buffer);

			switch (VP) {
				case VP_LED:
						int1 x = DSP_Buffer[5];
						LED(x);
						memset(DSP_Buffer, 0, 9);
					break;
				case VP_STRING:
						Get_string(Read_Text);
//						reply();
					break;
			}
		}
		HAL_UART_Receive_DMA(huart, RXBuffer, 60);
}

//void DSP_MSG()
//{
//	if(*PtrByteCount == 0)
//		return;
//
//	HAL_Delay(5);	//1 caracter no BaudRate(115200) demora 50Us
//	HAL_UART_DMAStop(&huart3);
//	memcpy(DSP_Buffer, RXBuffer+3, *PtrByteCount);
//	/* ******************************************
//	 * Funções portada da antiga DSP_GetCMD()
//	 */
//	memset(RXBuffer, 0, 4);
//	uint16_t VP = Get_VP(PtrDSP_Buffer);
//	/******************************************/
//	DSP_GetCMD();
//}

void DSP_Light(uint16_t Percent) {
    if (Percent >= 0 && Percent <= 100) {
        uint8_t scaledValue = (uint8_t)((Percent * 63) / 100);
        uint8_t command[] = {0x5A, 0xA5, 0x03, 0x80, 0x01, scaledValue};

        HAL_UART_Transmit_DMA(huart, command, 6);
    }
    HAL_Delay(1000);
}

void DSP_Read(uint8_t REG, uint8_t *PData, uint8_t N_Bytes_Read) {

	uint8_t Byte_count = 3;							// W/R + REG + Dados
	uint8_t Size_Package = 5;
	uint8_t *Package = (uint8_t*) malloc(Size_Package);

	memcpy(Package, FrameHead, 2);
	Package[2] = Byte_count;
	Package[3] = Read_REG;
	Package[4] = REG;

	HAL_UART_Transmit_DMA(huart, Package, Size_Package);

	uint8_t Package_Received = Size_Package + N_Bytes_Read;
	HAL_UART_Receive_DMA(huart, PData, Package_Received);
	/* Package --> <FrameHeadAlto><FHB> Bytecount = sizeof (<Leitura ou escrita> <Registrador a ser lido> ) */
}

void DSP_Write(uint8_t REG, uint8_t *PData, uint8_t N_Bytes) {

	uint8_t Byte_count = N_Bytes + 2;				// <0x5A> <0xA5>	W/R + REG + Dados

	/* Package --> <FrameHeadAlto><FHB> Bytecount = sizeof (<Leitura ou escrita> <Registrador> <dados...>) */
	uint8_t Size_Package = Byte_count + 3;

	uint8_t *Package = (uint8_t*) malloc(Size_Package);
	memcpy(Package, FrameHead, 2);
	Package[2]= Byte_count;
	Package[3] = Write_REG;
	Package[4] = REG;

	uint8_t *pDataPtr = PData;
	for (uint8_t i = 0; i < N_Bytes; i++) {
		Package[5 + i] = *pDataPtr++;
	}

	HAL_UART_Transmit_DMA(huart, Package, Size_Package);
	free(Package);
}

void DSP_Read_VPs(uint8_t REG, uint8_t *PData, uint8_t N_Bytes_Read) {

	uint8_t Byte_count = 3;							// W/R + REG + Dados
	uint8_t Size_Package = 5;
	uint8_t *Package = (uint8_t*) malloc(Size_Package);

	memcpy(Package, FrameHead, 2);
	Package[2] = Byte_count;
	Package[3] = Read_REG;
	Package[4] = REG;

	HAL_UART_Transmit_DMA(huart, Package, Size_Package);

	uint8_t Package_Received = Size_Package + N_Bytes_Read;
	HAL_UART_Receive_DMA(huart, PData, Package_Received);
	/* Package --> <FrameHeadAlto><FHB> Bytecount = sizeof (<Leitura ou escrita> <Registrador a ser lido> )
	 * <FHH> <FHL> <BC> 83 <VP><VP> <LEN> <VL1><VL1> [<VL2><VL2> <VL3><VL3> ...]
	 */
}

void DSP_Write_VPs(uint16_t VP, uint16_t Value) {
    uint8_t Byte_count = 6;             // <0x82> <VPH> <VPL> <ValueHIGH> <ValueLOW>    W/R + VP + Dados
    uint8_t Package[8];

    memcpy(Package, FrameHead, 2);
    Package[2] = Byte_count;
    Package[3] = Write_REG_VP;
    Package[4] = (uint8_t)((VP >> 8) & 0xFF);  // Byte de ordem superior (VPH)
    Package[5] = (uint8_t)(VP & 0xFF);         // Byte de menor ordem (VPL)
    Package[6] = (uint8_t)((Value >> 8) & 0xFF);  // Byte de ordem superior (ValueHIGH)
    Package[7] = (uint8_t)(Value & 0xFF);         // Byte de menor ordem (ValueLOW)

    /* Package --> <FrameHeadAlto><FHB> <ByteCount> <0x82> <VPH> <VPL> <ValueHIGH> <ValueLOW>; dados de 16bits (2 bytes)*/

    HAL_UART_Transmit_DMA(huart, Package, 8);
}


void DSP_Clean_S(uint16_t VP, uint8_t Size)
{

	uint8_t Byte_count = Size + 2 + 2;				// <0x5A> <0xA5>	W/R + REG + Dados E FF FF
	uint8_t Size_Package = Byte_count + 3;
	uint8_t VP_High = (VP >> 8) & 0xFF; // Byte mais significativo (bits 8-15)
	uint8_t VP_Low = VP & 0xFF;        // Byte menos significativo (bits 0-7)
	uint8_t Pack[100];
	memset(Pack, 0, 100);

	memcpy(Pack, FrameHead, 2);
		Pack[2]= Byte_count;
		Pack[3] = Write_REG_VP;
		Pack[4] = VP_High;
		Pack[5] = VP_Low;

	HAL_UART_Transmit_DMA(huart, Pack, Size_Package);
	HAL_Delay(1);
}

void DSP_PutS(char* Str_Write, uint16_t VP)
{
	uint8_t Size = strlen(Str_Write) + 1;
	uint8_t Byte_count = Size + 2 + 2;				// <0x5A> <0xA5>	W/R + REG + Dados E FF FF
	uint8_t Size_Package = Byte_count + 3;
	uint8_t VP_High = (VP >> 8) & 0xFF; // Byte mais significativo (bits 8-15)
	uint8_t VP_Low = VP & 0xFF;        // Byte menos significativo (bits 0-7)

	/* Package --> <FrameHeadAlto><FHB> Bytecount = sizeof (<Leitura ou escrita> <Registrador> <dados...>) */
	uint8_t *Package = (uint8_t*) malloc(Size_Package);

	memcpy(Package, FrameHead, 2);
	Package[2]= Byte_count;
	Package[3] = Write_REG_VP;
	Package[4] = VP_High;
	Package[5] = VP_Low;

	uint8_t* Pack_Write = &Package[6];
	char *Pstr = &Str_Write[0];
	char* Str_End = &Str_Write[Size];

	for (; Pstr < Str_End; Pstr++, Pack_Write++) {
		*Pack_Write = *Pstr;
	}


	HAL_UART_Transmit_DMA(huart, Package, Size_Package);
	free(Package);
	HAL_Delay(1);
}


void UART_setrate(uint32_t baudrate, UART_HandleTypeDef* huart_x) {
	// Desinicializar a USART
	HAL_UART_DeInit(huart_x);

	// Configurar nova velocidade
	huart_x->Init.BaudRate = baudrate;

	// Reconfigurar a USART com a nova velocidade
	HAL_UART_Init(huart_x);
}

//void reply() {
//    if (Read_Text[0] == '\0') {
//        return;
//    }
//
//    if (Read_Text[0] == '#') {
//        strcpy(Save_str, Read_Text + 1);
//    }
//
//    if (Read_Text[0] == '-') {
//        DSP_PutS(Save_str, 35);
//    }
//
//    int Flag = 0;
//    Flag = (strcmp(Read_Text, "reply") == 0);
//
//    if (Flag) {
//    	HAL_Delay(1000);
//    	DSP_PutS("Ok... Sua resposta é:", 35);
//    	HAL_Delay(1000);
//        DSP_PutS("Reposta 123", 35);
//    }
//}


void Clear_Curve_buffer(uint8_t Channel)
{
	//Escrita no Ctrl_REG 0xEB
//	uint8_t command = 0x56 | Channel;

}
