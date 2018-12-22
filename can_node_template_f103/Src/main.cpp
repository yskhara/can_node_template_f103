/*
 * main.cpp
 * by yskhara
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include "can.h"
#include "main.h"
#include "led.h"

#include <array>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

#define CAN_MTU 8

template<typename T>
union _Encapsulator
{
    T data;
    uint64_t i;
};

template<typename T>
static void can_unpack(const uint8_t (&buf)[CAN_MTU], T &data);
template<typename T>
static void can_pack(uint8_t (&buf)[CAN_MTU], const T data);

static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    //MX_CAN_Init();

    // CANを初期化する．
    can_init();
    // CANの通信速度を設定する．2018は500kbpsで通信した．
    can_set_bitrate(CAN_BITRATE_500K);

    // turn on green LED
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // blink red LED for test
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    // 初期化しただけでは，まだCANで送受信できない．
    // そこで，CANを有効化してやる．これで送受信できるようになる．
    can_enable();

    // CANで送受信するデータ（フレーム）を格納しておくための構造体変数．
    CanRxMsgTypeDef rx_msg;
    CanTxMsgTypeDef tx_msg;
    uint32_t status;

    // 送受信したいメッセージIDを定数として定義しておく．
    // constexprは，C++99とかにはない機能．
    static constexpr uint16_t id_handStatus = 0x300;
    static constexpr uint16_t id_handCmd = 0x301;

    uint32_t last_ctrl_time = HAL_GetTick();
    const uint32_t ctrl_interval = 1000 / 10;

    uint32_t last_stat_time = HAL_GetTick();
    const uint32_t stat_interval = 1000 / 2;


    while (1)
    {
        if (HAL_GetTick() - last_ctrl_time > ctrl_interval)
        {
            // タイマ割り込みを使わないタイムベース．精度はゴミでしょう．
            //carrierNode->Control();
            last_ctrl_time = HAL_GetTick();
        }

        if (HAL_GetTick() - last_stat_time > stat_interval)
        {
            // CANによるデータ送信の例
            // stat_intervalの周期で，他のノードに対してこのノードの状態を報告する．そんなケースを考えよう．
            //const CarrierStatus status = carrierNode->GetStatus();
            const uint16_t status = 0x0012;
            // can_packで，送信したいデータを，CANで送信できる形式に変換する．
            can_pack(tx_msg.Data, static_cast<uint16_t>(status));
            // メッセージIDを設定する．
            tx_msg.StdId = id_handStatus;
            // CAN_RTR_DATAを指定すると送信．
            tx_msg.RTR = CAN_RTR_DATA;
            // CAN_ID_STDを指定すると標準形式のフォーマットで通信する．
            tx_msg.IDE = CAN_ID_STD;
            // データ長を指定する．ここでは16ビット=2バイトのデータを送ろうとしているので，2を設定する．
            tx_msg.DLC = 2;
            // 送信ン〜
            // 第一引数に送信するデータを，第二引数にタイムアウトを指定する．タイムアウトはテキトーに3を指定した．
            can_tx(&tx_msg, 3);
            // 送信手続き終了！（ただし，送信が完了したかはわからない）

            last_stat_time = HAL_GetTick();
        }

        // 次に，他のノードから送られてきたデータを逐次処理していくようなケースを考える．
        // まず，受信バッファに未処理の受信データがあるか確認する．
        if (is_can_msg_pending (CAN_FIFO0))
        {
            // can_rxで，データを受信ン〜
            // 第一引数には，受信データを格納するrx_msgへのポインタを指定し，第二引数にはタイムアウトを指定する．タイムアウトはテキ（ｒｙ
            status = can_rx(&rx_msg, 3);
            // can_rxの戻り値がHAL_OKなら，正常に受信できたといってよいだろう．
            if (status == HAL_OK)
            {
                // 特定のメッセージIDをもつフレームにのみ興味がある．
                // （自分宛てのデータかどうか？というのをメッセージIDによって判断する）
                // received can frame
                if (rx_msg.StdId == id_handCmd)
                {
                    // 受信データを格納する変数
                    uint16_t cmd;
                    // 受信したデータを，欲しいデータ型に変換．（ここではuint16_tだね）
                    can_unpack(rx_msg.Data, cmd);
                    // 受信したデータを使ってやりたいようにやる
                    //carrierNode->SetCommand(static_cast<CarrierCommands>(cmd));
                    //carrierNode->Control();
                    last_ctrl_time = HAL_GetTick();
                }
                // この先に，さらにif文を追加すると，他のIDのフレームも取れる．switch構文のほうが綺麗かもね．

            }

            // データを受信したので，CANの動作確認用のLEDを光らせる．
            led_on();
        }

        // LEDの処理．詳しくはled.c参照．
        led_process();
    }
}

// おまけ．
// このファイルはCPPファイルなので，Cから直接アクセスできる関数を定義するためには，extern "C"が必要．
extern "C" void EXTI9_5_IRQHandler(void)
{
    // 割り込みフラグを確認．
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
    {
        // 割り込みフラグが立っていたら，フラグを戻して，割り込み処理．
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
        //carrierNode->OnRightChuckSensorEXTInt();
    }

    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
        //carrierNode->OnLeftChuckSensorEXTInt();
    }

    // この2行は必要ない．HALの割り込みフレームワークは遅いだろうし，割り込みにHALを使うメリットはあまりない．
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

// unpacks can payload
template<typename T>
void can_unpack(const uint8_t (&buf)[CAN_MTU], T &data)
{
    _Encapsulator<T> _e;

    for (int i = 0; i < sizeof(T); i++)
    {
        _e.i = (_e.i << 8) | (uint64_t) (buf[i]);
    }

    data = _e.data;
}

// packs can payload
template<typename T>
void can_pack(uint8_t (&buf)[CAN_MTU], const T data)
{
    _Encapsulator<T> _e;
    _e.data = data;

    for (int i = sizeof(T); i > 0;)
    {
        i--;
        buf[i] = _e.i & 0xff;
        _e.i >>= 8;
    }
}

// CANのハードウェアを初期化する．
/* CAN init function */
static void MX_CAN_Init(void)
{
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 16;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SJW = CAN_SJW_1TQ;
    hcan.Init.BS1 = CAN_BS1_1TQ;
    hcan.Init.BS2 = CAN_BS2_1TQ;
    hcan.Init.TTCM = DISABLE;
    hcan.Init.ABOM = DISABLE;
    hcan.Init.AWUM = DISABLE;
    hcan.Init.NART = DISABLE;
    hcan.Init.RFLM = DISABLE;
    hcan.Init.TXFP = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 *
 * CANで必要なGPIOピンは，stm32f1xx_hal_msp.cで初期化されるので，この関数は好きに書き換えて構わない．
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE()
    ;
    __HAL_RCC_GPIOD_CLK_ENABLE()
    ;
    __HAL_RCC_GPIOB_CLK_ENABLE()
    ;
    __HAL_RCC_GPIOA_CLK_ENABLE()
    ;

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pins : PB0 PB1 PB12 PB13
     PB14 PB15 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PA8 PA9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PB3 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/*****END OF FILE****/
