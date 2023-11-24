//extern SPI_HandleTypeDef *hspi = &hspi3;

#define LIS3DSH_READ 0x80
#define LIS3DSH_WRITE 0x00
#define LIS3DSH_REG_OUT_X_ADDR		((uint8_t) 0x28) /*Base address of OUT_X (H and L)*/
#define LIS3DSH_REG_OUT_Y_ADDR		((uint8_t) 0x2A) /*Base address of OUT_Y (H and L)*/
#define LIS3DSH_REG_OUT_Z_ADDR		((uint8_t) 0x2C) /*Base address of OUT_Z (H and L)*/

/* Who I am values */
#define LIS302DL_ID							0x3B
#define LIS3DSH_ID							0x3F

/* Common registers */
#define REG_WHO_I_AM		(uint8_t)0x0F

#define CTRL_REG3 0x23
#define CTRL_REG4 0x20
#define CTRL_REG5 0x24
#define CTRL_REG6 0x25

#define Defalt_REG3 0x00 //Sem interrupções
#define Defalt_REG4 0x67 // Taxa de 100hz, atualização de bloco, eixos X,Y,Z
#define Defalt_REG5 0x00  // Sem filtro e sensibilidade em +/-2G
#define Defalt_REG6 0xD0

/* CTRL_REG4 register */
#define LIS3DSH_CTRL_REG4_ODR_OFF	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG4_ODR_3_125	((uint8_t) 0x10)
#define LIS3DSH_CTRL_REG4_ODR_6_25	((uint8_t) 0x20)
#define LIS3DSH_CTRL_REG4_ODR_12_5	((uint8_t) 0x30)
#define LIS3DSH_CTRL_REG4_ODR_25	((uint8_t) 0x40)
#define LIS3DSH_CTRL_REG4_ODR_50	((uint8_t) 0x50)
#define LIS3DSH_CTRL_REG4_ODR_100	((uint8_t) 0x60)
#define LIS3DSH_CTRL_REG4_ODR_400	((uint8_t) 0x70)
#define LIS3DSH_CTRL_REG4_ODR_800	((uint8_t) 0x80)
#define LIS3DSH_CTRL_REG4_ODR_1600	((uint8_t) 0x90)

#define Lis_Faqu_1600	((uint8_t) 0x90)
#define Lis_Faqu_800	((uint8_t) 0x80)
#define Lis_Scale_2 ((uint8_t) 0x00)
#define Lis_Scale_16 ((uint8_t) 0x20)

#define LIS3DSH_CTRL_REG4_BDU_CONT	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG4_BDU_EN	((uint8_t) 0x08)

#define LIS3DSH_CTRL_REG4_ZEN_EN	((uint8_t) 0x04)
#define LIS3DSH_CTRL_REG4_YEN_EN	((uint8_t) 0x02)
#define LIS3DSH_CTRL_REG4_XEN_EN	((uint8_t) 0x01)

/* CTRL_REG3 register */
#define LIS3DSH_CTRL_REG3_DR_EN		((uint8_t) 0x80)

#define LIS3DSH_CTRL_REG3_IEA_LOW	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG3_IEA_HIGH	((uint8_t) 0x40)

#define LIS3DSH_CTRL_REG3_IEL_LATCH	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG3_IEL_PULSE	((uint8_t) 0x20)

#define LIS3DSH_CTRL_REG3_INT2_EN	((uint8_t) 0x10)
#define LIS3DSH_CTRL_REG3_INT1_EN	((uint8_t) 0x08)

/* CTRL_REG5 register */
#define LIS3DSH_CTRL_REG5_FSCALE_2	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG5_FSCALE_4	((uint8_t) 0x08)
#define LIS3DSH_CTRL_REG5_FSCALE_6	((uint8_t) 0x10)
#define LIS3DSH_CTRL_REG5_FSCALE_8	((uint8_t) 0x18)
#define LIS3DSH_CTRL_REG5_FSCALE_16	((uint8_t) 0x20)

#define LIS3DSH_CTRL_REG5_SIM_4WIRE	((uint8_t) 0x00)
#define LIS3DSH_CTRL_REG5_SIM_3WIRE	((uint8_t) 0x01)

#define LIS3DSH_GRAVIDADE		((float) -9.81f)

typedef enum {
	LIS3DSH_OK,
	LIS3DSH_INIT_ERROR,
	LIS3DSH_SCALE_ERROR,
	LIS3DSH_READ_ERROR,
	LIS3DSH_WRITE_ERROR,
	LIS3DSH_GET_AXIS_X_ERROR,
	LIS3DSH_GET_AXIS_Y_ERROR,
	LIS3DSH_GET_AXIS_Z_ERROR,
} Lis_error;

int16_t read_accelX();
int16_t read_accelY();

/* Váriaveis */
float mediaX=0, Ref_ZeroX, Ref_4mmX, Ref_180X;
float mediaY=0, Ref_ZeroY, Ref_4mmY, Ref_180X;
float Angulo, He, FatorX, FatorY;


float calc_aceleration(float measure, uint8_t scale_G) {
	float Aceleration = 0, Temp = 0;


	Temp = 32768 / scale_G;
	Temp = (float) measure / Temp;
	Aceleration = LIS3DSH_GRAVIDADE * Temp;
	return Aceleration;
}

void media_X(float * Ref_X){
	uint16_t NUM_AMOSTRAS = 300, NUM_MEDIAS = 20;
	int32_t somaX = 0;
	*Ref_X =0;

	for (int media_media = 0; media_media < NUM_MEDIAS; media_media++)
		{
			for (int media_acel = 0; media_acel < NUM_AMOSTRAS; media_acel++)
				somaX += read_accelX();

			// Calcula a média dos valores
			mediaX = (float) somaX / NUM_AMOSTRAS;
			somaX=0;

			if(media_media == 0)
			{
			*Ref_X = mediaX;
			}

			*Ref_X = (*Ref_X * 0.7  + 0.3 * mediaX) ;
		}

}
void media_Y(float * Ref_Y){
	uint16_t NUM_AMOSTRAS = 300, NUM_MEDIAS = 20;
	int32_t somaY = 0;
	*Ref_Y =0;

	for (int media_media = 0; media_media < NUM_MEDIAS; media_media++)
		{
			for (int media_acel = 0; media_acel < NUM_AMOSTRAS; media_acel++)
				somaY += read_accelY();

			// Calcula a média dos valores
			mediaY = (float) somaY / NUM_AMOSTRAS;
			somaY=0;

			if(media_media == 0)
			{
			*Ref_Y = mediaY;
			}

			*Ref_Y = (*Ref_Y * 0.7  + 0.3 * mediaY) ;
		}

}
void media_XY(float * Ref_X, float* Ref_Y ){

	uint16_t NUM_AMOSTRAS = 300, NUM_MEDIAS = 20;
	int32_t somaX = 0, somaY = 0;
	*Ref_X =0;
	*Ref_Y =0;

for (int media_media = 0; media_media < NUM_MEDIAS; media_media++)
	{

		for (int media_acel = 0; media_acel < NUM_AMOSTRAS; media_acel++) {

			// Acumula os valores dos eixos Y e Y
			somaX += read_accelX();
			somaY += read_accelY();

			// Tempo para controlar a taxa de leitura
	//				HAL_Delay(2); // Aproximadamente 100 Hz de leitura
		}

		// Calcula a média dos valores
		mediaX = (float) somaX / NUM_AMOSTRAS;
		mediaY = (float) somaY / NUM_AMOSTRAS;
		somaX=0;
		somaY=0;

		if(media_media == 0)
		{
		*Ref_X = (int16_t)mediaX;
		*Ref_Y = (int16_t)mediaY;
		}

		*Ref_X = (*Ref_X * 0.7  + 0.3 * mediaX) ;
		*Ref_Y = (*Ref_Y * 0.7  + 0.3 * mediaY);
	}

}

//==========================================================

uint8_t Default_REG4 = LIS3DSH_CTRL_REG4_BDU_CONT | LIS3DSH_CTRL_REG4_ZEN_EN
		| LIS3DSH_CTRL_REG4_YEN_EN | LIS3DSH_CTRL_REG4_XEN_EN;
//
//void Select_LIS3DH(void)
//{
//	CS_Acel(0);
//}
//void Deselect_LIS3DH(void)
//{
//	CS_Acel(1);
//}
//
//void SPI_Transmit(uint8_t *byte, uint16_t size)
//{
//	HAL_SPI_Transmit(hspi, byte, size, HAL_MAX_DELAY);
//}
//
//void SPI_Receive(uint8_t *byte, uint16_t size)
//{
//	HAL_SPI_Receive(hspi, byte, size, HAL_MAX_DELAY);
//}
//
//uint8_t Detect_LIS3D()
//{
//	uint8_t LIS3DSHTR = 0; 		//neutro
//	uint8_t Temp[2];
//
//	Select_LIS3DH();
//
//	Temp[0] = 0;
//	Temp[1] = 0x80 | REG_WHO_I_AM;
//	HAL_SPI_Receive(hspi, Temp, 2, 100);
//
//
//	if (Temp[1] == LIS3DSH_ID)
//	{
//		LIS3DSHTR = 1;		//Funcionamento
//	}
//	Deselect_LIS3DH();
//	return LIS3DSHTR;
//}
//
//uint8_t LIS3DH_Read_reg(uint8_t Read_Reg, uint8_t *PData, uint8_t Size_Pdata)
//{
//	//Bit mais significativo é 1
//	uint8_t Command[2];
//	Command[1]= 0;
//	Command[0] = Read_Reg | LIS3DSH_READ;
//	if(Size_Pdata == 1)
//		Size_Pdata = 2;
//
//	Select_LIS3DH();
//	HAL_SPI_TransmitReceive(hspi, Command, PData, Size_Pdata, 100);
//
//	Deselect_LIS3DH();
//	return LIS3DSH_OK;
//}
//
//uint8_t LIS_Read_reg_N(uint8_t reg_addr, uint8_t *dataR, uint8_t size)
//{
//	uint8_t Ok = 0;
//	dataR[0] = 0x80 | reg_addr;
//
//	Select_LIS3DH();
//	Ok = HAL_SPI_Receive(hspi, dataR, size, 10) == HAL_OK;
//	Deselect_LIS3DH();
//	if(!Ok)
//	{
//		return LIS3DSH_READ_ERROR;
//	}
//	return LIS3DSH_OK;
//}
//
//uint8_t LIS3DH_Write_reg(uint8_t WriteAddr, uint8_t Data)
//{
//	//Bit mais significativo é 0 que configura a escrita no acelerômetro
//	uint8_t command[2];
//	Select_LIS3DH();
//	// Enviar o comando de escrita no registrador especificado
//	command[0] = WriteAddr | LIS3DSH_WRITE;
//	command[1] = Data;
//
//	HAL_SPI_Transmit(&hspi3, command, 2, HAL_MAX_DELAY);
//
//	Deselect_LIS3DH();
//	HAL_Delay(50);
//	command[0] = WriteAddr | LIS3DSH_READ;
//	HAL_SPI_Receive(&hspi3, &command[0], 1, HAL_MAX_DELAY);
//	if(command[0] != Data)
//	{
//		command[1] = 0;
//	}else
//		command[1] = 1;
//
//	return command[1];
//}
//
//// Função de inicialização do LIS3DSH
//
//void LIS3DSH_Init()
//{
//	Deselect_LIS3DH();
////    uint8_t ctrlReg4Data = 0x08; // 2g de faixa, taxa de amostragem de 3.125 kHz
//
//	LIS3DH_Write_reg(CTRL_REG3, Defalt_REG3);
//	LIS3DH_Write_reg(CTRL_REG4, Lis_Faqu_1600 | Default_REG4);
//	LIS3DH_Write_reg(CTRL_REG5, Lis_Scale_16);
//	LIS3DH_Write_reg(CTRL_REG6, Defalt_REG6);
//
//
//	uint8_t check;
//	LIS3DH_Read_reg(CTRL_REG4, &check, 1);
//	if (check != (Lis_Faqu_1600 | Default_REG4))
//	{
//		while (1);
//	}
//
//	LIS3DH_Read_reg(CTRL_REG5, &check, 1);
//	if (check != Lis_Scale_16)
//	{		//Registrador não fica em 0
//		while (1);
//	}
//
//	//LIS3DH_Read_reg(CTRL_REG6, &check, 1);
//	//if (check != Defalt_REG6)
//	//{
//	//	while (1);
//	//}
//}
//
//uint8_t LIS3DSH_Get_axis(int16_t *axis)
//{
//	uint8_t dataR[3] = { 0x00, 0x00, 0x00 };
//
//	if (LIS_Read_reg_N(LIS3DSH_REG_OUT_X_ADDR, dataR, 3) != LIS3DSH_OK)
//	{
//		return 0;	//Deu erro
//	}
//	axis[0] = dataR[1] | dataR[2] << 8;
//
//	if (LIS_Read_reg_N(LIS3DSH_REG_OUT_Y_ADDR, dataR, 3) != LIS3DSH_OK)
//	{
//		return 0;	//Deu erro
//	}
//	axis[1] = dataR[1] | dataR[2] << 8;
//
//	if (LIS_Read_reg_N(LIS3DSH_REG_OUT_Z_ADDR, dataR, 3) != LIS3DSH_OK)
//	{
//		return 0;	//Deu erro
//	}
//	axis[2] = dataR[1] | dataR[2] << 8;
//
//	return LIS3DSH_OK;
//}
//
//uint8_t LIS3DSH_Get_accelerations(float *accelerations)
//{
//	int16_t l_a_axis[3] = { 0x00, 0x00, 0x00 };
//	uint8_t l_e_error;
//	uint8_t aScaleFactor = 2;
//
//	l_e_error = LIS3DSH_Get_axis(l_a_axis);
//
//	if (l_e_error != LIS3DSH_OK)
//	{
//		return l_e_error;
//	}
//
//	for (uint8_t i = 0; i < 3; i++)
//	{
//		accelerations[i] = LIS3DSH_GRAVIDADE * ((float) l_a_axis[i] / (32768 / aScaleFactor));
//	}
//
//	return l_e_error;
//}

/* Protocolo I2C*/

#define Read 		0x3D
#define Write 		0x3C

#define STATUS		0x27
#define OUT_X		0x28
#define OUT_Y		0x2A
#define OUT_Z		0x2C

#define CTRL_REG1	0x21
#define CTRL_REG2	0x22
#define CTRL_REG3	0x23
#define CTRL_REG4	0x20
#define CTRL_REG5	0x24	// 16 bits tem que estar em 0 = 2g, 800hz anti-aliasing e SPI 4 fios
#define CTRL_REG6 	0x25/* Padrão com Rate max de 1600Hz e BDU em 0 0x97*/

void I2C_Mode() {
	HAL_GPIO_WritePin(GPIOD, CS_I2C_Pin, 1); // Enquato CS em 1, I2C Habilitado
	HAL_GPIO_WritePin(GPIOC, SEL_I2C_Pin, 0); // Enquato CS em 1, I2C Habilitado
}

void I2C_Write(uint16_t REG, uint8_t Data) {
	uint8_t pData[2];
	pData[0] = Data;
	pData[1] = Data;
	HAL_I2C_Mem_Write(&hi2c3, Write, REG, 1, pData, 2, HAL_MAX_DELAY);
}

void I2C_Read(uint16_t REG, uint8_t *Pdata, uint8_t Size) {
	HAL_I2C_Mem_Read(&hi2c3, Read, REG, 1, Pdata, Size, HAL_MAX_DELAY);
}
uint8_t TEST_REG(uint8_t REG, uint8_t value) {
	uint8_t test[2];

	I2C_Read(REG, test, 2);

	if (test[0] != value && test[1] != value) {
		I2C_Write(REG, value);
		I2C_Read(REG, test, 2);
		if (test[0] != value && test[1] != value) {
			return 0;
		}
	}
	return 1;
}
void init_LIS() {
	uint8_t Erro = -1;
	I2C_Mode();

	Erro = TEST_REG(CTRL_REG1, 0);
	Erro = TEST_REG(CTRL_REG2, 0);
	Erro = TEST_REG(CTRL_REG3, 0x0);	//Sem INT =0, com INT = C8
	Erro = TEST_REG(CTRL_REG4, 0x87); // Enable axis e ODR
	Erro = TEST_REG(CTRL_REG6, 0x10);

	if(Erro != 1)
		while(1);
}

int16_t read_accelX() {
	int16_t Data = 0;
	uint8_t rawData[2];
	I2C_Read(OUT_X, rawData, 2);
	Data = (int16_t) rawData[1] << 8 | rawData[0];
	return Data;
}

int16_t read_accelY() {
	int16_t Data = 0;
	uint8_t rawData[2];
	I2C_Read(OUT_Y, rawData, 2);
	Data = (int16_t) rawData[1] << 8 | rawData[0];
	return Data;
}

void Read_Data(int16_t *destination) {
	uint8_t rawData[6];  // Dados brutos do acelerômetro serão armazenados aqui

	// Lê os dados brutos dos três eixos (X, Y, Z) do acelerômetro LIS3DSH
	if (HAL_I2C_Mem_Read(&hi2c3, Read, OUT_X, 1, rawData, 6, HAL_MAX_DELAY)
			== HAL_OK) {
		// Os dados são lidos como palavras de 16 bits no LIS3DSH; combine os bytes corretamente
		destination[0] = ((int16_t) rawData[1] << 8) | rawData[0];
		destination[1] = ((int16_t) rawData[3] << 8) | rawData[2];
		destination[2] = ((int16_t) rawData[5] << 8) | rawData[4];
	} else {
		// Lida com o erro de leitura, se houver
		// Por exemplo, você pode definir os valores de destino para zero ou outra coisa apropriada
		destination[0] = 0;
		destination[1] = 0;
		destination[2] = 0;
	}
}
