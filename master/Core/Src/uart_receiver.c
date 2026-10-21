#include "uart_receiver.h"
#include "lcd.h"
#include <stdio.h>
#include <string.h>

/* ✅ EXTERN handles created by CubeMX (in usart.c / dma.c) */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart4_rx;

/* ✅ EXTERN your buffers/vars (you already have them somewhere) */
extern uint8_t  uart1_rx_dma[];
extern uint8_t  uart3_rx_dma[];
extern uint8_t  uart4_rx_dma[];

extern char     lineBuf1[];
extern char     lineBuf3[];
extern char     lineBuf4[];

extern volatile uint16_t lineIndex1;
extern volatile uint16_t lineIndex3;
extern volatile uint16_t lineIndex4;

extern volatile uint8_t voltage_updated;


/* Your settings struct */

/* Your flags/vars used in code */
extern volatile uint8_t Recceived, uart1_ready, Fliptestcorrect, menureceiveflag;
extern volatile uint8_t RRecceived, Ruart1_ready, RFliptestcorrect, Rmenureceiveflag;
extern volatile uint8_t YRecceived, Yuart2_ready, YFliptestcorrect, Ymenureceiveflag;
extern volatile uint8_t BRecceived, Buart3_ready, BFliptestcorrect, Bmenureceiveflag;

extern volatile uint8_t RreceiverInrush;
extern volatile uint8_t YreceiverInrush;
extern volatile uint8_t BreceiverInrush;

/* Phase vars */
extern char phaseChar;
extern int  phaseInput, phaseOutput, phaseCont, pHeatFlag;
extern char currentPhase;
extern int  phaseRinput, phaseRoutput, phaseRcont, phaseRheat;

extern char RreceiverChar;
extern int  Rreceiverinput, RreceiverOutput, RreceiverCont, Rreceiverheat;
extern char RphaseChar;
extern int  Rphaseinput, Rphaseoutput, Rphasecont, Rphaseheat;

/* Y */
extern char YreceiverChar;
extern int  Yreceiverinput, YreceiverOutput, YreceiverCont, Yreceiverheat;
extern char YphaseChar;
extern int  Yphaseinput, Yphaseoutput, Yphasecont, Yphaseheat;

/* B */
extern char BreceiverChar;
extern int  Breceiverinput, BreceiverOutput, BreceiverCont, Breceiverheat;
extern char BphaseChar;
extern int  Bphaseinput, Bphaseoutput, Bphasecont, Bphaseheat;

/* Buffer sizes */
#ifndef RX_BUF_SIZE
#define RX_BUF_SIZE 128
#endif

#ifndef LINE_BUF_SIZE
#define LINE_BUF_SIZE 128
#endif

extern  volatile uint8_t Rrunningtimeflag;
extern  volatile uint8_t Yrunningtimeflag;
extern  volatile uint8_t Brunningtimeflag;
extern  volatile uint8_t Srunningtimeflag;
static uint8_t phaseCont_stable_active = 0;
static uint32_t phaseCont_start_time = 0;
#define PHASE_CONT_STABLE_TIME_MS    250U

static uint8_t Rphasecont_stable_active = 0;
static uint32_t Rphasecont_start_time = 0;

static uint8_t Yphasecont_stable_active = 0;
static uint32_t Yphasecont_start_time = 0;

static uint8_t Bphasecont_stable_active = 0;
static uint32_t Bphasecont_start_time = 0;


extern  volatile uint8_t Rinrushjustdisplayflag;
extern  volatile uint8_t Yinrushjustdisplayflag;
extern  volatile uint8_t Binrushjustdisplayflag;
extern  volatile uint8_t Sinrushjustdisplayflag;

/* ================= Local static handlers ================= */
static void UART1_SinglePhase_RxHandler(uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++)
    {
        char c = (char)uart1_rx_dma[i];
        if (c == '\r') continue;

        if (c == '\n')
        {
            lineBuf1[lineIndex1] = '\0';

            if (lineBuf1[0] == '[' && lineBuf1[2] == ']')
            {
                if      (lineBuf1[1] == 'P') { Recceived = 1; }
                else if (lineBuf1[1] == 'D') { uart1_ready = 1; }
                else if (lineBuf1[1] == 'K') { Fliptestcorrect = 1; }
                else if (lineBuf1[1] == 'W') { Fliptestcorrect = 2; }
                else if (lineBuf1[1] == 'A' && uart1_ready) { menureceiveflag = 1; }
                else if (lineBuf1[1] == 'V')
                {
                    int vin, vout;
                    if (sscanf(lineBuf1, "[V]%d,%d", &vin, &vout) == 2)
                    {
                        working_settings.input_v_r  = (float)vin;
                        working_settings.output_v_r = (float)vout;
                    }
                    Srunningtimeflag = 1;
                }
#if 0
                else if (lineBuf1[1] == 'R')
                {
                    int singleInrush = 0;

                    if (sscanf(lineBuf1, "[%c]%d,%d,%d,%d,%d",
                               &phaseChar,
                               &phaseInput,
                               &phaseOutput,
                               &phaseCont,
                               &pHeatFlag,
                               &singleInrush) == 6)
                    {
                        currentPhase  = phaseChar;
                        phaseRinput   = phaseInput;
                        phaseRoutput  = phaseOutput;
                        phaseRcont    = phaseCont;
                        phaseRheat    = pHeatFlag;
                        RreceiverInrush = (uint8_t)singleInrush;   // NEW
                        voltage_updated = 1;
                    }
                }
#endif
                else if (lineBuf1[1] == 'R')
                {
                    int singleInrush = 0;

                    if (sscanf(lineBuf1, "[%c]%d,%d,%d,%d,%d",
                               &phaseChar,
                               &phaseInput,
                               &phaseOutput,
                               &phaseCont,
                               &pHeatFlag,
                               &singleInrush) == 6)
                    {
                        currentPhase = phaseChar;
                        phaseRinput = phaseInput;
                        phaseRoutput = phaseOutput;
                        if (phaseCont == 1)
                        {
                            if (phaseCont_stable_active == 0)
                            {
                                phaseCont_stable_active = 1;
                                phaseCont_start_time = HAL_GetTick();
                            }
                            else
                            {
                                if ((HAL_GetTick() - phaseCont_start_time) >= PHASE_CONT_STABLE_TIME_MS)
                                {
                                    phaseRcont = 1;
                                }
                            }
                        }
                        else
                        {
                            phaseCont_stable_active = 0;
                            phaseCont_start_time = 0;
                            phaseRcont = 0;
                        }

                        phaseRheat = pHeatFlag;
   //                     RreceiverInrush = (uint8_t)singleInrush;
                        Sinrushjustdisplayflag = (uint8_t)singleInrush;
                        voltage_updated = 1;
                    }
                }
            }

            lineIndex1 = 0;
        }
        else if (lineIndex1 < LINE_BUF_SIZE - 1)
        {
            lineBuf1[lineIndex1++] = c;
        }
        else
        {
            lineIndex1 = 0;
        }
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

static void UART1_ThreePhase_RxHandler(uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++)
    {
        char c = (char)uart1_rx_dma[i];
        if (c == '\r') continue;

        if (c == '\n')
        {
            lineBuf1[lineIndex1] = '\0';

            if (lineBuf1[0] == '[' && lineBuf1[2] == ']')
            {
                switch (lineBuf1[1])
                {
					case 'P': RRecceived = 1; break;
					case 'D': Ruart1_ready = 1;break;
					case 'A': Rmenureceiveflag = 1; break;
                    case 'K': RFliptestcorrect = 1; break;
                    case 'W': RFliptestcorrect = 2; break;

                    case 'V':
                    {
                        int Rvin, Rvout;
                        if (sscanf(lineBuf1, "[V]%d,%d", &Rvin, &Rvout) == 2)
                        {
                            working_settings.input_v_r  = Rvin;
                            working_settings.output_v_r = Rvout;
                        }
                        Rrunningtimeflag = 1;
                    } break;
#if 0
                    case 'R':
                    {
                     int rinrush = 0;

                     if (sscanf(lineBuf1, "[%c]%d,%d,%d,%d,%d",
                                &RreceiverChar,
                                &Rreceiverinput,
                                &RreceiverOutput,
                                &RreceiverCont,
                                &Rreceiverheat,
                                &rinrush) == 6)
                     {
                         RphaseChar = RreceiverChar;
                         Rphaseinput = Rreceiverinput;
                         Rphaseoutput = RreceiverOutput;
                         Rphasecont = RreceiverCont;
                         Rphaseheat = Rreceiverheat;
                         RreceiverInrush = (uint8_t)rinrush;
                         voltage_updated = 1;
                     }
                    } break;
#endif
                    case 'R':
                    {
                        int rinrush = 0;

                        if (sscanf(lineBuf1, "[%c]%d,%d,%d,%d,%d",
                                   &RreceiverChar,
                                   &Rreceiverinput,
                                   &RreceiverOutput,
                                   &RreceiverCont,
                                   &Rreceiverheat,
                                   &rinrush) == 6)
                        {
                            RphaseChar   = RreceiverChar;
                            Rphaseinput  = Rreceiverinput;
                            Rphaseoutput = RreceiverOutput;
                            Rphaseheat   = Rreceiverheat;
                            if (RreceiverCont == 1)
                            {
                                if (Rphasecont_stable_active == 0)
                                {
                                    Rphasecont_stable_active = 1;
                                    Rphasecont_start_time = HAL_GetTick();
                                }
                                else
                                {
                                    if ((HAL_GetTick() - Rphasecont_start_time) >=
                                        PHASE_CONT_STABLE_TIME_MS)
                                    {
                                        Rphasecont = 1;
                                    }
                                }
                            }
                            else
                            {
                                Rphasecont_stable_active = 0;
                                Rphasecont_start_time = 0;
                                Rphasecont = 0;
                            }
               //             RreceiverInrush = (uint8_t)rinrush;
                            Rinrushjustdisplayflag = (uint8_t)rinrush;
                            voltage_updated = 1;
                        }
                    }
                    break;
                }
            }

            lineIndex1 = 0;
        }
        else if (lineIndex1 < LINE_BUF_SIZE - 1) lineBuf1[lineIndex1++] = c;
        else lineIndex1 = 0;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

static void UART4_ThreePhase_RxHandler(uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++)
    {
        char c = (char)uart4_rx_dma[i];
        if (c == '\r') continue;

        if (c == '\n')
        {
            lineBuf4[lineIndex4] = '\0';

            if (lineBuf4[0] == '[' && lineBuf4[2] == ']')
            {
                switch (lineBuf4[1])
                {
					case 'P':YRecceived=1;break;
					case 'D':Yuart2_ready=1;break;
					case 'K':YFliptestcorrect=1; break;
					case 'W':YFliptestcorrect=2;break;
					case 'A':Ymenureceiveflag=1;break;
                    case 'V':
                    {
                        int Yvin, Yvout;
                        if (sscanf(lineBuf4, "[V]%d,%d", &Yvin, &Yvout) == 2)
                        {
                            working_settings.input_v_y  = Yvin;
                            working_settings.output_v_y = Yvout;
                        }
                        Yrunningtimeflag = 1;
                    } break;
#if 0
                    case 'Y':
                    {
                     int yinrush = 0;

                     if (sscanf(lineBuf4, "[%c]%d,%d,%d,%d,%d",
                                &YreceiverChar,
                                &Yreceiverinput,
                                &YreceiverOutput,
                                &YreceiverCont,
                                &Yreceiverheat,
                                &yinrush) == 6)
                     {
                         YphaseChar = YreceiverChar;
                         Yphaseinput = Yreceiverinput;
                         Yphaseoutput = YreceiverOutput;
                         Yphasecont = YreceiverCont;
                         Yphaseheat = Yreceiverheat;
                         YreceiverInrush = (uint8_t)yinrush;
                     }
                    } break;
#endif
                    case 'Y':
                    {
                        int yinrush = 0;
                        if (sscanf(lineBuf4, "[%c]%d,%d,%d,%d,%d",
                                   &YreceiverChar,
                                   &Yreceiverinput,
                                   &YreceiverOutput,
                                   &YreceiverCont,
                                   &Yreceiverheat,
                                   &yinrush) == 6)
                        {
                            YphaseChar   = YreceiverChar;
                            Yphaseinput  = Yreceiverinput;
                            Yphaseoutput = YreceiverOutput;
                            Yphaseheat   = Yreceiverheat;
                            if (YreceiverCont == 1)
                            {
                                if (Yphasecont_stable_active == 0)
                                {
                                    Yphasecont_stable_active = 1;
                                    Yphasecont_start_time = HAL_GetTick();
                                }
                                else
                                {
                                    if ((HAL_GetTick() - Yphasecont_start_time) >=
                                        PHASE_CONT_STABLE_TIME_MS)
                                    {
                                        Yphasecont = 1;
                                    }
                                }
                            }
                            else
                            {
                                Yphasecont_stable_active = 0;
                                Yphasecont_start_time = 0;
                                Yphasecont = 0;
                            }

//                            YreceiverInrush = (uint8_t)yinrush;
                            Yinrushjustdisplayflag = (uint8_t)yinrush;
                        }
                    }
                    break;
                }
            }

            lineIndex4 = 0;
        }
        else if (lineIndex4 < LINE_BUF_SIZE - 1) lineBuf4[lineIndex4++] = c;
        else lineIndex4 = 0;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart4_rx, DMA_IT_HT);
}


static void UART3_ThreePhase_RxHandler(uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++)
    {
        char c = (char)uart3_rx_dma[i];
        if (c == '\r') continue;

        if (c == '\n')
        {
            lineBuf3[lineIndex3] = '\0';

            if (lineBuf3[0] == '[' && lineBuf3[2] == ']')
            {
                switch (lineBuf3[1])
                {
					case 'P':BRecceived=1;break;
					case 'D':Buart3_ready=1;break;
					case 'K':BFliptestcorrect=1;break;
					case 'W':BFliptestcorrect=2;break;
					case 'A':Bmenureceiveflag=1;break;
                    case 'V':
                    {
                        int Bvin, Bvout;
                        if (sscanf(lineBuf3, "[V]%d,%d", &Bvin, &Bvout) == 2)
                        {
                            working_settings.input_v_b  = Bvin;
                            working_settings.output_v_b = Bvout;
                        }
                        Brunningtimeflag = 1;
                    } break;
#if 0
                    case 'B':
                    {
                     int binrush = 0;

                     if (sscanf(lineBuf3, "[%c]%d,%d,%d,%d,%d",
                                &BreceiverChar,
                                &Breceiverinput,
                                &BreceiverOutput,
                                &BreceiverCont,
                                &Breceiverheat,
                                &binrush) == 6)
                     {
                         BphaseChar = BreceiverChar;
                         Bphaseinput = Breceiverinput;
                         Bphaseoutput = BreceiverOutput;
                         Bphasecont = BreceiverCont;
                         Bphaseheat = Breceiverheat;
                         BreceiverInrush = (uint8_t)binrush;
                     }
                    } break;
#endif
                    case 'B':
                    {
                        int binrush = 0;

                        if (sscanf(lineBuf3, "[%c]%d,%d,%d,%d,%d",
                                   &BreceiverChar,
                                   &Breceiverinput,
                                   &BreceiverOutput,
                                   &BreceiverCont,
                                   &Breceiverheat,
                                   &binrush) == 6)
                        {
                            BphaseChar   = BreceiverChar;
                            Bphaseinput  = Breceiverinput;
                            Bphaseoutput = BreceiverOutput;
                            Bphaseheat   = Breceiverheat;
                            if (BreceiverCont == 1)
                            {
                                if (Bphasecont_stable_active == 0)
                                {
                                    Bphasecont_stable_active = 1;
                                    Bphasecont_start_time = HAL_GetTick();
                                }
                                else
                                {
                                    if ((HAL_GetTick() - Bphasecont_start_time) >=
                                        PHASE_CONT_STABLE_TIME_MS)
                                    {
                                        Bphasecont = 1;
                                    }
                                }
                            }
                            else
                            {
                                Bphasecont_stable_active = 0;
                                Bphasecont_start_time = 0;
                                Bphasecont = 0;
                            }
   //                         BreceiverInrush = (uint8_t)binrush;
                            Binrushjustdisplayflag = (uint8_t)binrush;
                        }
                    }
                    break;
                }
            }

            lineIndex3 = 0;
        }
        else if (lineIndex3 < LINE_BUF_SIZE - 1) lineBuf3[lineIndex3++] = c;
        else lineIndex3 = 0;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_dma, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}


/* ================= Public API ================= */
void UARTAPP_RxStart_ByMode(void)
{
    if (GetMode() == MODE_SINGLE)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        HAL_UART_AbortReceive(&huart3);
        HAL_UART_AbortReceive(&huart4);
    }
    else
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart4_rx, DMA_IT_HT);
    }
}

void UARTAPP_OnModeChanged(void)
{
    /* call this whenever pcd_id changes */
    lineIndex1 = lineIndex3 = lineIndex4 = 0;
    UARTAPP_RxStart_ByMode();
}


/* ================= HAL Callbacks (ONLY ONCE) ================= */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_OREF | UART_CLEAR_PEF);
    HAL_UART_AbortReceive(huart);

    if (GetMode() == MODE_SINGLE)
    {
        if (huart->Instance != USART1) return;

        lineIndex1 = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        return;
    }

    if (huart->Instance == USART1)
    {
        lineIndex1 = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
    else if (huart->Instance == USART4)
    {
        lineIndex4 = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart4_rx, DMA_IT_HT);
    }
    else if (huart->Instance == USART3)
    {
        lineIndex3 = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (GetMode() == MODE_SINGLE)
    {
        if (huart->Instance == USART1)
            UART1_SinglePhase_RxHandler(Size);
        return;
    }

    if (huart->Instance == USART1)
        UART1_ThreePhase_RxHandler(Size);
    else if (huart->Instance == USART4)
        UART4_ThreePhase_RxHandler(Size);
    else if (huart->Instance == USART3)
        UART3_ThreePhase_RxHandler(Size);
}
