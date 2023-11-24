# STM32-J70L-Display-Victor-Vision-Proculus-
Código base para utilizar Display Da Victor Vision (J70L) - Linguagem C
🎲 Display Proculus com STM32 (CubeIDE)
Esse código foi escrito com o chip STM32F303RE – Núcleo
Passos para utilizar este código:
$ Os arquivos necessários estão dentro da pasta Core/Inc e são 2 arquivos:
Display_J70L.h - 	Contém as funções de gerenciamento do display.
Register_J70L.h - 	Contém endereços e registradores importantes
Coisas importantes a saber sobre o funcionamento do Proculus (J70L)
$ Funciona através de comunicação serial e possui um cabeçalho de comunicação padrão. (Esse cabeçalho padrão pode ser alterado no software UnicViwer nas configurações)  Cabeçalho: <FHH><FHL>  Frame Head High e Frame Head Low
$ 
Dependências
$ É preciso adicionar uma rotina chamada DSP_MSG no seu main.c.
Essa função é própria de cada projeto, já que a ideia é que essa função faça a interação entre o Display e seu STM32, ela recebe um código via Serial, e verifica se, é o display “falando”. Os 2 primeiros bytes que serão o cabeçalho <5A><A5> confirmam se é o display.
Configurar suas variáveis de ambiente
•	1° Crie função DSP_MSG()  Tem o exemplo e forma de uso dentro do arquivo Display_J70L.h. Copie essa função, tire o comentário e adapte como desejar.
•	2° Cadastre as variáveis de ambiente que nesse caso, é generalizar o endereço de UART que você está utilizando, por exemplo:     	UART_HandleTypeDef *huart = &huart3;   Criei um apontador e coloquei dentro dele o endereço da UART que estou utilizando, você pode mudar para o de sua preferência. ( = &huart1 ou &huart2 e etc.)
•	3° Escrevi as funções usando DMA(Direct Memory Acess) na UART, você pode reescrever usando transmit comum, é só acrescentar o parâmetro de delay.


 
O arquivo executável para ver minhas configurações está no arquivo (DSP_ATT_F3.ioc) mas é preciso copiar o repositório inteiro para executar na sua máquina.
________________________________________
🛠 Tecnologias
Os seguintes Integrados foram usadas na construção do projeto:
•	STM32 F303RE
•	Lisa (LIS3DSH)
•	Display J70L 800x480px ________________________________________
⚒️ Dependências e Bibliotecas
•	Display_J70L.h
•	Register_J70L.h
