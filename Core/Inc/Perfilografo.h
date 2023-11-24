/*
 * Perfilógrafo.h
 *
 *  Created on: Sep 1, 2023
 *      Author: Teknikao
 */

#ifndef INC_PERFILÓGRAFO_H_
#define INC_PERFILÓGRAFO_H_
#include "Lisa.h"
/*
 * Protótipos
 */
void DSP_PutS(char *Str_Write, uint16_t VP);
void medida_ini();
void calibração_4mm_X();
void calibração_4mm_Y();
void reset();
int16_t read_accelX();
int16_t read_accelY();

#define change_pg 		0x0A		//VP: 10

#define btn_reset 		0x00		//VP: 0

#define btn_Inicial 	0x01		//VP: 1
#define btn_calço_X 	0x02		//VP: 2
#define btn_calço_Y 	0x03		//VP: 3
#define btn_X_180		0x04		//VP: 4
#define btn_Y_180		0x05		//VP: 5

/*
 * VPS de texto
 * Medida inicial X e Y*/
#define Text_init_X 	0x64
#define Text_init_Y 	0x96
/*
 * VP de calços de 4 Milimetros X e Y
 */
#define Text_calço_X 	0xC8
#define Text_calço_Y 	0xFA
/*
 * VP de medida a 180º
 */
#define Text_180_X		0x12C
#define Text_180_Y		0x15E
/*
 * VP de medindo...
 */
#define Text_Med_X		0x190
#define Text_Med_Y		0x1C2


void perfilografo(uint16_t VP) {
	switch (VP) {
	case btn_Inicial:
//			DSP_Clean_S(Text_init_X, 10);
		medida_ini();
		break;
	case btn_calço_X:
		calibração_4mm_X();
		break;
	case btn_calço_Y:
		calibração_4mm_Y();
		break;
	case btn_X_180:
		break;
	case btn_Y_180:
		break;
	case btn_reset:
		reset();
		break;

		/*
		 * Envio de texto
		 */

		case change_pg:
					break;
//		case btn_Y_180:
//					break;
	}
	HAL_UART_Receive_DMA(huart, RXBuffer, 60);
}

void medida_ini() {

	media_XY(&Ref_ZeroX, &Ref_ZeroY);

//	float accX = calc_aceleration(mediaX, 2);
//	float accY = calc_aceleration(mediaY, 2);

	char Acel_data_X[10];
	char Acel_data_Y[10];

	sprintf(Acel_data_X, "%.2f", Ref_ZeroX);
	sprintf(Acel_data_Y, "%.2f", Ref_ZeroY);

	DSP_PutS(Acel_data_X, Text_init_X);
	DSP_PutS(Acel_data_Y, Text_init_Y);
	HAL_Delay(1);
}

void calibração_4mm_X() {
	if(mediaX == 0)
		medida_ini();

	media_X(&Ref_4mmX);
	FatorX = Ref_4mmX - Ref_ZeroX;
	FatorX = FatorX/4;
	char Acel_str[10];
	sprintf(Acel_str, "%.2f", Ref_4mmX);
	DSP_PutS(Acel_str, Text_calço_X);
}

void calibração_4mm_Y() {
	if(mediaY == 0)
		medida_ini();

	media_Y(&Ref_4mmY);
	FatorY = Ref_4mmY - Ref_ZeroY;
	FatorY = FatorY/4;

	char Acel_str[10];
	sprintf(Acel_str, "%.2f", Ref_4mmY);
	DSP_PutS(Acel_str, Text_calço_Y);
}

void medindo()
{
	if(FatorX ==0 && FatorY ==0)
		return;

	float readX,readY;
	media_X(&readX);
	media_Y(&readY);

	readX= readX/FatorX; 	//Resposta em MM
	readY= readY/FatorY;

	char Acel_str[10];
	sprintf(Acel_str, "%.2f", readX);
	DSP_PutS(Acel_str, Text_calço_X);
}

void reset()
{
	uint16_t labels[] = {Text_180_X, Text_180_Y, Text_Med_X, Text_Med_Y,Text_calço_X, Text_calço_Y, Text_init_X,Text_init_Y};
	for (int i=0; i<(sizeof(labels)); i++)
	{
		DSP_Clean_S(labels[i], 10);
	}
	mediaX=0; Ref_ZeroX=0; Ref_4mmX=0; Ref_180X=0;
	mediaY=0; Ref_ZeroY=0; Ref_4mmY=0; Ref_180X=0;
	Angulo=0; He=0; FatorX=0; FatorY=0;
}
#endif /* INC_PERFILÓGRAFO_H_ */
