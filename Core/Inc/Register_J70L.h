/*
 * Registradores J70L.h
 *
 *  Created on: Aug 7, 2023
 *      Author: Teknikao
 */

#ifndef INC_REGISTER_J70L_H_
#define INC_REGISTER_J70L_H_

#define Read_REG 			0x81
#define Write_REG 			0x80
#define Write_REG_VP		0x82
#define Read_REG_VP			0x83
#define Write_TrendCurve	0x84

#define	REG_R1_BaudRate				0x07
#define	REG_R_
#define	REG_R1_
#define	REG_R1_
#define	REG_R1_
/*Registradores BaudRate: R1, R5, 59*/
//R1 Value
#define BaudRate_115200 0x07
/*Registrador R2 Controla: CRC, Backlight, Autosend, RAM_INIT, Ciclo de operação*/
//7 0x80 Undefined Write 0.
//6 0x40 Undefined Write 0.
//5 0x20 BACKLIGHT 0 = Disable Backlight Automatic Control.
//1 = Enabled Backlight Automatic Control.
//4 0x10 CRC_CTRL 0 = Disable CRC16 verification on serial port.
//1 = Enable CRC16 verification on serial port.
//3 0x08 AUTOSEND 0 = Disable Control Auto-Send.
//1 = Enable Control Auto-Send.
//2 0x04 RAM_INIT 0 = Clear RAM at power-up.
//1 = Initialize RAM at power-up according to initial values.
//0:1 0x02:0x01 OP_CYCLE
//Operation Cycle Period:
//00 = 200ms
//01 = 160ms
//10 = 120ms
//11 = 80ms
//Obs.: Padrão é 120ms


/*Registrador R4: Rotação de tela*/
//0x00 0°
//0x01 90° Clockwise
//0x02 180° Clockwise
//0x03 270° Clockwise
/*Parametros Pré definidos*/
#define Brilho_MAX 0x3F

#endif /* INC_REGISTER_J70L_H_ */
