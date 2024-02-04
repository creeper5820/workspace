---
title: can-tutorial
date: 2024-01-25 03:32:17
tags:
---


## 引子

现在是2024年1月18号晚上零点半，电路工数等困难科目已经考完，只是剩一门马原

临近寒假的这一段时间颇为闲暇，于是在工作室寻得一些 **M2006无刷电机**和 **C610电调** ，加上手头上的 **C板**，试着组一台个人未来比赛用的四驱底盘

依据大疆资料来看，电调需要使用CAN通信来控制，正中知识盲区，于是放下手中的马原教材（其实根本没有拿起来过），学习一下CAN

## 环境准备

### 前置知识
- STM32CubeMX的使用
- 一定的C语言使用经验

### 软件环境

- 代码生成 `STM32CubeMX` （以HAL库为基础）
- 编译工具 `arm-none-eabi工具链`（使用其他编译器亦可）
- 编写环境 `VSCode`+`Embedded IDE` （Keil和CubeIDE亦可）
- 调试工具 `Ozone`（本篇仅以此方法调试）

### 硬件环境

- 主控芯片 `大疆C板-STM32F407IGH6`
- 烧录工具 `JLink`
- 通讯目标 `C610电调`

### 设备文档

- [C610电调使用说明](https://rm-static.djicdn.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf)

- [RoboMaster 开发板C型](https://rm-static.djicdn.com/tem/35228/RoboMaster%20%20%E5%BC%80%E5%8F%91%E6%9D%BF%20C%20%E5%9E%8B%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf)

- [C板原理图&位号图](https://rm-static.djicdn.com/tem/35228/RoboMaster%20%E5%BC%80%E5%8F%91%E6%9D%BFC%E5%9E%8B%E5%8E%9F%E7%90%86%E5%9B%BE&%E4%BD%8D%E5%8F%B7%E5%9B%BE.zip)

## CAN的印象

### 何为CAN？

在查阅了很多资料后，我提取了几个关键词：`总线结构`，`串行通讯`，`标准协议`，只需要两条线，即可解决沿途中设备的通信需求，例如，使用一块主控板加上CAN总线就可以很轻松的控制多个电机，极大缓解了布线带给我们的`焦虑`

{% asset_img slug %}
<center>
<img src="https://img-blog.csdnimg.cn/direct/0334eb74241d49979182c2ec0562302b.jpeg" 
    width=80% />
</center>
<center>
布线地狱
</center>
<br>

至于书面，准确，乃至于繁缛的官方定义，我便不写入文章里，百度看看就好

### CAN的硬件组成

我们可以称一个通讯单元为**节点**，一个节点一般有三个部分：**微控制器**， **CAN控制器**，**CAN收发器**，总线两端须串上120Ω的电阻，以模拟无限远传输线的特性阻抗，通过开关等手段来选择是否使用这个**电阻**

<center>
<img src=https://img-blog.csdnimg.cn/direct/fc40ed4d25ce4e14b155bb0b385efd73.png
    width=80% 
    />
</center>
<center>
CAN总线结构
</center>
<br>

STM32芯片会自带CAN外设拓展，名为**bxCAN** `(Basic Extended CAN  - 基本拓展CAN)`，详细内容此处不展开

要注意，一般的STM32开发板是不带有CAN收发器的，需要自己另外购买，大疆C板是自带CAN收发器的，所以可以直接使用

## CAN的回环测试

姑且暂停理论部分的讲解，**繁杂的原理**总是令人头大，使人望而却步，我们先**启动**开发软件，走通一个通讯的流程，再来细细分析其中的缘由，或者跳过理论，只掌握软件层的流程也是可以的

基本步骤：`配置STM32CubeMX` > `配置CAN过滤器` > `发送接收报文`

### 配置STM32CubeMX

启动CubeMX，选好芯片类型创建项目，首先把**常规设置**搞定

关于C板的一些**注意点**

- 注意C板的晶振是**12MHz**，要把输入频率调整成12MHz
- C板的外设电源和swd输入的**电源不在一条线路**，不能通过**swd口供电**，需要插上**24v电源**或者**usb口供电**，否则CAN的收发器将不工作，无法正常收发数据，当然，**回环模式**还是可以收到的，因为回环的数据不经过CAN收发器
- 一对一连接C板和电调时，需要将电调上的电阻打开，一对多时，把最远端的电阻打开即可，保持CAN总线两端串着电阻

<center>
<img src=https://img-blog.csdnimg.cn/direct/5c952ce9d3e74729bef9badd88baf38f.png
    width=80% 
    />
</center>
<center>
RCC设置
</center>
<br>

<center>
<img src=https://img-blog.csdnimg.cn/direct/14c833c93c1a4343911d8267e5a1948c.png
    width=80% 
    />
</center>
<center>
SWD设置
</center>
<br>

<center>
<img src=https://img-blog.csdnimg.cn/direct/0d19eb74c01c46c780a29f194f143da4.png
    width=80% 
    />
</center>
<center>
时钟设置
</center>
<br>

<center>
<img src=https://img-blog.csdnimg.cn/direct/5ce51f07fb40420098f0e63340972fea.png
    width=80% 
    />
</center>
<center>
.c文件和.h文件分开生成
</center>
<br>

项目管理类型之类的根据**自己使用的开发环境**来设置即可

简单写一个点灯测试一下

这是板载灯的连线

- `TIM5_CH1` - `LED_BLUE`
- `TIM5_CH2` - `LED_GREEN`
- `TIM5_CH3` - `LED_RED`

```cpp
void breath_led()
{
    for (int i = 0; i < 100; i++) {
        HAL_Delay(10);
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 20000 * i / 100);
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 20000 * i / 100);
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 20000 * i / 100);
    }

    for (int i = 100; i > 0; i--) {
        HAL_Delay(10);
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 20000 * i / 100);
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 20000 * i / 100);
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 20000 * i / 100);
    }
}
```
将其放入主循环中运行，理所应当地成功了
<center>
<img src=https://img-blog.csdnimg.cn/direct/1d9fc725662a4a28b4bf8b925585b0fc.jpeg
    width=80% 
    />
</center>
<center>
呼吸灯测试
</center>
<br>

现在开始配置**CAN通信**

CubeMX界面中，在`CAN1`的**Parameter Settings**我们可以看到

- **Bit Timings Parameters** - 配置传输速度
    - **Prescaler (for Time Quantum)** - 分频，调整TQ（Time Quantum）大小
    - Time Quantum - 最小时间单位
    - **Time Quanta in Bit Segment 1** - 相位缓冲段1段占几个TQ
    - **Time Quanta in Bit Segment 2** - 相位缓冲段2段占几个TQ
    - Time for one Bit
    - Baud Rate - 波特率
    - **ReSynchronization Jump Width** - 再同步补偿宽度
- **Basic Parameters** - 基本参数
    - Time Triggered Communication Mode - 时间触发模式
    - Automatic Bus-off Management - 自动离线管理
    - Automatic Wake-Up Mode - 自动唤醒
    - Automatic Retransmission - 自动重传
    - Receive Fifo Locked Mode - 锁定模式
    - Transmit Fifo Priority - 报文发送优先级
- **Advanced Parameters** - 高级参数
    - **Operating Mode** -*运行模式：`正常模式` `静默模式` `回环模式` `回环静默模式`

而 **NVIC Interrupt Table** 中有

- CAN1 TX interrupts
- CAN1 RX0 interrupts
- CAN1 RX1 interrupt
- CAN1 SCE interrupt

这是我们初期需要关注的配置列表

**1. 设置波特率**

以我的需求为例，查阅大疆官方资料可以得知

> 将 CAN 信号线连接到控制板接收 CAN 控制指令，CAN 总线比特率为 1Mbps 

所以我们需要将CAN通讯的比特率 `baud rate` 设置为 `1000000 bit/s`

根据波特率计算公式 BaudRate = TQ * ( Sync + TBS1 + TBS2) , 我们得到如下设置

<center>
<img src=https://img-blog.csdnimg.cn/direct/bac1aba73d1040cfa061ecc028c35bfa.png
    width=80% 
    />
</center>
<center>
TQ * ( 1 + 11 + 2 ) = 1000ns
</center>

根据实际情况计算一下即可，也可以多选几个选项，把正确的波特率尝试出来，**灰色的选项**就是CubeMX帮我们计算好的数值

**2. 打开中断**

处理电调发送的电机信息，需要中断来调用回调函数，于是打开**接收中断**

在接收和发送信息前，我们会遇到一个CAN通信的抽象概念 —— **邮箱**

这里我使用的单片机中，CAN外设具有两个用于接收信息的**邮箱**，我们命其为 `FIFO0`和`FIFO1`，每个邮箱都有**一个过滤器**，用于筛选报文，可以存放**三条报文**，在**中断设置**中对应 `CAN1 RX0 interrupt`和`CAN1 RX1 interrupt`，我们打开需要使用的那一个就可以

<center>
<img src=https://img-blog.csdnimg.cn/direct/f3a40ac4efe94be79b876c875cedbbf1.png
    width=80% 
    />
</center>

既然存在接收邮箱，相应的，就有**发送邮箱**，我们现在只要知道发送邮箱存在**发送优先级**且每个邮箱只能存放**一条报文**

现在，我们已经在CubeMX中配置好了CAN，下一步就是要配置**CAN过滤器**

### 配置CAN过滤器

前面我们说到，STM32上有两个**邮箱**用于接收报文，为了接收我们想要的报文，我们需要配置一下过滤器，把不想接受的报文过滤掉，只放行想要的报文

配置过滤器需要我们自己手写，并未提前生成，但HAL库提供了过滤器配置参数的结构体类型，我们只需要**给这个结构体赋值**，然后**调用HAL提供的初始化函数**即可完成配置

```cpp
// Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal_can.h

// 过滤器结构体
typedef struct
{
  uint32_t FilterIdHigh;
  uint32_t FilterIdLow;
  uint32_t FilterMaskIdHigh;
  uint32_t FilterMaskIdLow; 
  uint32_t FilterFIFOAssignment; 
  uint32_t FilterBank;        
  uint32_t FilterMode;
  uint32_t FilterScale;
  uint32_t FilterActivation; 
  uint32_t SlaveStartFilterBank; 
} CAN_FilterTypeDef;

// 配置函数
HAL_StatusTypeDef HAL_CAN_ConfigFilter(
    CAN_HandleTypeDef *hcan, 
    CAN_FilterTypeDef *sFilterConfig
    );
```

具体的使用和结构体的定义随后再讲，我们只需要对这个结构体和函数有一个**大概的印象**即可

### 发送接收报文

首先是**发送**

我预期使用一块主控与四个电机通信，那么在发送报文时，就需要指定**发送给哪一个电机**，以及**其他一些信息**，比如发送`信息的长度`，`信息的类型`，`信息ID类型`等等，HAL把这些发送需要的信息定义成了一个结构体 `CAN_TxHeaderTypeDef`,我们只需要为每一个电机声明一个 `CAN_TxHeaderTypeDef` 结构体，再确定好发送的数据内容，就可以将数据发送到指定的电机中

我们回想一下，在设置接收中断时，是不是提到了**邮箱**的概念？**STM32F407IGHx**为我们提供了三个**发送邮箱**，在发送时，HAL库会自动选择空闲的邮箱，然后将**实际使用的邮箱**返回给我们，这也解释了我们传入函数的是指向邮箱的指针，而非一个邮箱编号的常量

HAL库理所应当地帮我们写好了发送的函数，只要传入`can的句柄`，`报文头结构体`，`数据信息`和`邮箱`即可

```cpp
// Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal_can.h

// 发送函数的声明
HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    CAN_HandleTypeDef *hcan, 
    CAN_TxHeaderTypeDef *pHeader, 
    uint8_t aData[], 
    uint32_t *pTxMailbox
    )；

// 邮箱编号的定义
#define CAN_TX_MAILBOX0             (0x00000001U)  /*!< Tx Mailbox 0  */
#define CAN_TX_MAILBOX1             (0x00000002U)  /*!< Tx Mailbox 1  */
#define CAN_TX_MAILBOX2             (0x00000004U)  /*!< Tx Mailbox 2  */
```

然后是**接收**

总线上的报文在经过了我们设置的**过滤器**后，正确的报文会**触发**我们设置的**中断**，我们便可以在中断的**回调函数**中对收到的数据进行处理了

我们只需要找到HAL库为我们提供的中断函数，对其进行覆写即可

```cpp
// 这是一种使用情况

// 回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance ==CAN1)
	{
	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, date_CAN1); 
	  return ;
	}
}

// 这个函数可以从报文中分离出我们想要的信息
HAL_StatusTypeDef HAL_CAN_GetRxMessage(
    CAN_HandleTypeDef *hcan,            // can句柄
    uint32_t RxFifo,                    // 接收邮箱编号
    CAN_RxHeaderTypeDef *pHeader,       // 接收报文头
    uint8_t aData[]                     // 数据
    )；

// 接收邮箱编号的定义
#define CAN_RX_FIFO0                (0x00000000U)  /*!< CAN receive FIFO 0 */
#define CAN_RX_FIFO1                (0x00000001U)  /*!< CAN receive FIFO 1 */
```

现在我们配置过滤器和发送接收这两个流程应该是有了一个**大概的认知**，来做一个简单的测试吧

将运行模式设置为**回环发送**，我们就可以收到自己发送的报文，前提是能通过邮箱过滤，其他配置依照上文即可

<center>
<img src=https://img-blog.csdnimg.cn/direct/9662f70acc644e3897a03f64c5d34d2b.png
    width=80% 
    />
</center>

记得重新生成代码

然后我们写一个过滤器的配置

```cpp
void can_filter_init()
{
    CAN_FilterTypeDef config;

    // 报文头结构体的赋值
    // 此处配置为接收全部报文，以便于测试
    config.FilterActivation     = ENABLE;                // 启用过滤器
    config.FilterBank           = 0;                     // 将要初始化的过滤器组
    config.SlaveStartFilterBank = 0;                     // 从模式下的过滤器组
    config.FilterMode           = CAN_FILTERMODE_IDMASK; // 掩码模式
    config.FilterScale          = CAN_FILTERSCALE_32BIT; // 32位宽
    config.FilterFIFOAssignment = CAN_FILTER_FIFO0;      // 配置邮箱0
    config.FilterIdHigh         = 0x0000;                // 高位0
    config.FilterIdLow          = 0x0000;                // 低位0
    config.FilterMaskIdHigh     = 0x0000;                // 掩码高位不检测
    config.FilterMaskIdLow      = 0x0000;                // 掩码低位不检测

    // 将配置加载进CAN 1中
    HAL_CAN_ConfigFilter(&hcan1, &config);
}
```

初始化CAN

上面过滤器的配置中启用了邮箱0 `CAN_FILTER_FIFO0`，所以在初始化时，我们要打开邮箱0的中断

```cpp
void can_init()
{
    can_filter_init();                                                  // 过滤器
    HAL_CAN_Start(&hcan1);                                              // 开启CAN通讯
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // 开启接收中断
}
```

```cpp
// Hal库中的邮箱接收中断类型
CAN_IT_RX_FIFO0_MSG_PENDING     /*!< FIFO 0 message pending interrupt */
CAN_IT_RX_FIFO0_FULL            /*!< FIFO 0 full interrupt            */
CAN_IT_RX_FIFO0_OVERRUN         /*!< FIFO 0 overrun interrupt         */
CAN_IT_RX_FIFO1_MSG_PENDING     /*!< FIFO 1 message pending interrupt */
CAN_IT_RX_FIFO1_FULL            /*!< FIFO 1 full interrupt            */
CAN_IT_RX_FIFO1_OVERRUN         /*!< FIFO 1 overrun interrupt 
```

声明一些必要的变量

```cpp
uint8_t can_1_rx[8];    // 接收数据
uint8_t can_1_tx[8];    // 发送数据

CAN_RxHeaderTypeDef can_1_rx_header;    // 接收报文头
CAN_TxHeaderTypeDef can_1_tx_header;    // 发送报文头

uint32_t mail_tx = CAN_TX_MAILBOX0;     // 发送邮箱编号
```

初始化一些参数

```cpp
// 随便创建一种发送报文头结构体
can_1_tx_header.StdId              = 0x00000000;
can_1_tx_header.ExtId              = 0x12345000;
can_1_tx_header.IDE                = CAN_ID_EXT;
can_1_tx_header.RTR                = CAN_RTR_DATA;
can_1_tx_header.DLC                = 8;
can_1_tx_header.TransmitGlobalTime = DISABLE;

// 初始化一些发送的数据
can_1_tx[0] = 1;

// 要使用的灯记得开启，根据自己的板子写即可
HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
// 刚才写的初始化函数用上
can_init();
```

回调函数的覆写

```cpp
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_1_rx_header, can_1_rx);

        // 简单根据接收数据内容做一个反馈，点亮或熄灭板载灯
        // 根据自己的板子替换一下点灯的函数
        if(can_1_rx[0] == 0)
        {
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 20000);
        }
        else if(can_1_rx[0] == 1)
        {
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);
        }

        return;
    }
}
```

在主循环中不断发送报文

```cpp
HAL_CAN_AddTxMessage(&hcan1, &can_1_tx_header, can_1_tx, &mail_tx);
```

这是 main.c ，注意根据自己使用的板子情况**进行修改**

```cpp
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t can_1_rx[8]; // 接收数据
uint8_t can_1_tx[8]; // 发送数据

CAN_RxHeaderTypeDef can_1_rx_header; // 接收保报文头
CAN_TxHeaderTypeDef can_1_tx_header; // 发送报文头

uint32_t mail_tx = CAN_TX_MAILBOX0; // 发送邮箱编号

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void can_filter_init();
void can_init();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void can_filter_init()
{
    CAN_FilterTypeDef config;

    // 报文头结构体的赋值
    // 此处配置为接收全部报文，以便于测试
    config.FilterActivation     = ENABLE;                // 启用过滤器
    config.FilterBank           = 0;                     // 将要初始化的过滤器组
    config.SlaveStartFilterBank = 0;                     // 从模式下的过滤器组
    config.FilterMode           = CAN_FILTERMODE_IDMASK; // 掩码模式
    config.FilterScale          = CAN_FILTERSCALE_32BIT; // 32位宽
    config.FilterFIFOAssignment = CAN_FILTER_FIFO0;      // 配置邮箱0
    config.FilterIdHigh         = 0x0000;                // 高位0
    config.FilterIdLow          = 0x0000;                // 低位0
    config.FilterMaskIdHigh     = 0x0000;                // 掩码高位不检测
    config.FilterMaskIdLow      = 0x0000;                // 掩码低位不检测

    // 将配置加载进CAN 1中
    HAL_CAN_ConfigFilter(&hcan1, &config);
}

void can_init()
{
    can_filter_init();                                                 // 过滤器
    HAL_CAN_Start(&hcan1);                                             // 开启CAN通讯
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 开启接收中断
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_1_rx_header, can_1_rx);

        // 简单根据接收数据内容做一个反馈，点亮或熄灭板载灯
        // 根据自己的板子替换一下点灯的函数
        if (can_1_rx[0] == 0) {
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 20000);
        } else if (can_1_rx[0] == 1) {
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);
        }

        return;
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_CAN1_Init();
    /* USER CODE BEGIN 2 */

    // 随便创建一种发送报文头结构体
    can_1_tx_header.StdId              = 0x00000000;
    can_1_tx_header.ExtId              = 0x12345000;
    can_1_tx_header.IDE                = CAN_ID_EXT;
    can_1_tx_header.RTR                = CAN_RTR_DATA;
    can_1_tx_header.DLC                = 8;
    can_1_tx_header.TransmitGlobalTime = DISABLE;

    can_1_tx[0] = 1;

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    can_init();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {

        HAL_CAN_AddTxMessage(&hcan1, &can_1_tx_header, can_1_tx, &mail_tx);
        HAL_Delay(10);

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 168;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

```

在ozone中查看参数，并实时修改**发送数据**的数值，发现**接收数据**也会实时修改，板载灯反馈正常

若不能使用ozone，也可以在代码中修改发送的数值，重新烧录，查看板载灯的情况

<center>
<img src=https://img-blog.csdnimg.cn/direct/b9f8c4761d6f417087f7213b0a7d41ca.png
    width=60% 
    />
</center>

回环测试正常，我们可以进行下一步的了解

## CAN与电调通讯

在囫囵吞枣地走通过一遍流程后，我们遇到很很多**复杂的模式和结构体**，这些需要根据实际情况来酌情配置

接下来我们尝试使用C板来与C610通讯，接收信息并发送信息来控制电机

### 接收电机的回馈消息

为了保持代码的可读性，我们将CAN相关代码分离开来

大致结构是底层依赖`base-can`，在其基础上写一层`module-m2006`，在进程中调用这两个部分文件

为了降低理解的难度，暂时不考虑使用其他设备的可能，只针对一个电调和一个电机的情况先写一份控制代码

首先是base-can部分，它负责与底层的交互，直接使用HAL库提供的函数，将底层与应用层隔离

```cpp
// base-can.h

#pragma once
#include "can.h"

void init_can(CAN_HandleTypeDef *hcan);

void set_can_tx_header(CAN_TxHeaderTypeDef *header);
```

```cpp
// base-can.c

#include "base-can.h"

// 仍然使用最简单的配置，接收所有报文
// 为了统一初始化函数的调用模式，我们将HAL_CAN_ConfigFilter函数的返回值返回
static HAL_StatusTypeDef configure_can_filter(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef config;

    config.FilterActivation     = CAN_FILTER_ENABLE;
    config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    config.FilterMode           = CAN_FILTERMODE_IDMASK;
    config.FilterScale          = CAN_FILTERSCALE_32BIT;
    config.FilterIdHigh         = 0x00;
    config.FilterIdLow          = 0x00;
    config.FilterMaskIdHigh     = 0x00;
    config.FilterMaskIdLow      = 0x00;
    config.FilterBank           = 0;
    config.SlaveStartFilterBank = 0;

    return HAL_CAN_ConfigFilter(hcan, &config);
}

// 集中对CAN进行初始化
void init_can(CAN_HandleTypeDef *hcan)
{
    if (configure_can_filter(hcan))
        Error_Handler();

    if (HAL_CAN_Start(hcan))
        Error_Handler();

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING))
        Error_Handler();
}

// 根据C610手册，设置发送报文头的内容
void set_can_tx_header(CAN_TxHeaderTypeDef *header)
{
    header->StdId              = 0x200;
    header->IDE                = CAN_ID_STD;
    header->RTR                = CAN_RTR_DATA;
    header->DLC                = 8;
    header->TransmitGlobalTime = DISABLE;
}

```

上面代码写的十分地局限，没有考虑后期**可能不断变化的需求**，但对于现阶段来说，我们的首要目标是先以最简单的方式，驱动目标电机，现在接着往下写

### 包装与电调通讯的代码

为了方便我们 `get` 和 `control` 电机的状态，可以把对外暴露的api，即电机返回的状态值，用结构体包装，而对于控制电机需要使用到的 `句柄` ，或者说一些必要的 `上下文信息` ,我们也使用结构体将其包装起来，最后将两个结构体包装为电机完整的结构体，这样可以极大地方便函数的调用

```cpp
// module-m2006.h

#pragma once
#include "base-can.h"

typedef struct MotorStatus {
    int16_t angle;
    int16_t speed;
    int16_t torque;
} MotorStatus;

typedef struct MotorHandle {
    CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef header_rx;
    CAN_TxHeaderTypeDef header_tx;
    uint32_t mail;
} MotorHandle;

typedef struct Motor {
    MotorHandle handle;
    MotorStatus status;
} Motor;

void init_motor(Motor *motor);

void get_motor_status(const uint8_t data[8], Motor *motor);

void set_motor_current(const int16_t current, Motor *motor);
```

```cpp
// module-m2006.c

#include "module-m2006.h"

// 初始化关于电机的所有信息
void init_motor(Motor *motor)
{
    motor->handle.hcan = &hcan1;

    init_can(motor->handle.hcan);
    set_can_tx_header(&motor->handle.header_tx);
}

// 用于从回报文中提取状态信息
// 按照C610用户手册上给出的高低位数据变换即可
void get_motor_status(const uint8_t data[8], Motor *motor)
{
    motor->status.angle  = (data[0] << 8) | data[1];
    motor->status.speed  = (data[2] << 8) | data[3];
    motor->status.torque = (data[4] << 8) | data[5];
}

// 用于设置电机输出电流值
// 由于目前只使用一个电机，为了方便，直接设置电机电调的ID为1
// 那么第一位和第二位uint8_t数据就是第一个电机的电流值高低位
// 依照文档，电流值的范围为 -10000 到 10000
void set_motor_current(const int16_t current, Motor *motor)
{
    uint8_t data[8];

    data[0] = current >> 8;
    data[1] = current | 0xff00;

    HAL_CAN_AddTxMessage(
        motor->handle.hcan, &motor->handle.header_tx,
        data, &motor->handle.mail);
}

```
在做完前面的工作后，我们只需要调用最上层的`module-motor`提供的初始化函数，就可以对整个系统初始化，然后使用`get_motor_status`获取电机状态，使用`set_motor_current`控制电机电流值

### 主要逻辑和CAN接收回调函数

最后，为了与HAL库生成的文件分离地更彻底一点，我们在CubeMX生成的main.c中，加入我们自定义的进程入口

```cpp
// 前面代码省略


/* USER CODE BEGIN PFP */
extern void entrypoint();
/* USER CODE END PFP */


// 中间代码省略


/* USER CODE BEGIN 2 */
entrypoint();
/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */


// 后面代码省略
```

使用extern声明一个外部函数，接着在CubeMX提供的主循环前插入该函数，我们便可以创建一个entrypoint.c函数来实现`void entrypoint()`

同时我们也可以把回调函数写在这里，保证变量作用域的统一，这样以后对代码增删改查都可以避免直接接触CubeMX直接生成的代码，贯彻了代码**高内聚低耦合**的原则（doge）


```cpp
// entrypoint.c

#include "main.h"
#include "tim.h"

#include "module-m2006.h"

// entrypoint function from main.h
void entrypoint();

static Motor motor;
static int16_t current;

// @brief the main loop function
// @note control the current of motor every 1 ms
void entrypoint()
{
    init_motor(&motor);
    current = 0;

    while (1) {
        HAL_Delay(1);
        set_motor_current(current, &motor);
    }
}

// @brief callback when receiving message from motor
// @note convert the uint8_t data to motor status
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) {
        uint8_t data[8];

        HAL_CAN_GetRxMessage(
            &hcan1, CAN_RX_FIFO0,
            &motor.handle.header_rx, data);

        get_motor_status(data, &motor);

        return;
    }
}
```

这是代码结构，`activity`中是主要的进程，里面包含了程序自定义入口，也可以在使用实时系统的情况下将任务写在该文件夹下，`dependency`是包装后的库函数，供`activity`调用，`base`是与底层直接接触的库，一般会用到大量的HAL库，负责实现一些通讯协议，而`module`是更高级一层的，一般抽象成为某个外设的驱动库


```
// 注意HAL库生成的代码并没有被写出
// 请将这些文件结构包含进项目目录中，以保证entrypoint函数能够被顺利链接
-activity
  entrypoint.c
-dependency
  -base
    base-can.c
    base-can.h
  -module
    module-m2006.c
    module-m2006.h
```

### 连接设备与测试

临时画一个M2006的电机架测试用，放上一张整体的连线图

<center>
<img src=https://img-blog.csdnimg.cn/direct/b2ab2d1d35e64f748716595eadc33d68.png
    width=80% 
    />
</center>

<center>
<img src=https://img-blog.csdnimg.cn/direct/e7b7c967147d44a68894373f82a300b7.jpeg
    width=80% 
    />
</center>

修改`entrypoint.c`中`current`的值，主循环中就会以1000Hz的频率调整电机的电流值，同时可以查看电调发回来的三个状态信息，使用Ozone可以查看`motor`和`current`两个变量的情况

<center>
<img src=https://img-blog.csdnimg.cn/direct/32f4e14701794797baaabf72cf58f805.png
    width=80% 
    />
</center>

使用绘图工具将`angle`的返回值绘制出来，旋转电机，可以发现角度值会随着旋转流畅地改变

<center>
<img src=https://img-blog.csdnimg.cn/direct/13a2924d64694173a87a0e51b07fd959.png
    width=80% 
    />
</center>

调整`current`的值，就可以控制电机的正反转动，由于没有写**控制算法**，运动总是**迟滞**于电流值的改变的

<center>
<img src=https://img-blog.csdnimg.cn/direct/8a430c3d44ac4c9d9a745f213bcd3486.jpeg
    width=80% 
    />
</center>

倘若无法使用ozone，可以使用**串口**传递将需要查看的变量和需要修改的变量，或者修改`current`后查看电机转动状态亦可，各个参数的范围请参照官方提供的开发文档

## CAN的各种配置与模式

### HAL库关于CAN的说明

想要深入了解Hal库提供的接口，最好的办法是直接查看**源码上的注释**

这是 `stm32f4xx_hal_can.c` 中的注释，我翻译了一下

---
#### 如何使用该驱动

- 通过执行 `HAL_CAN_MspInit()` 初始化 `CAN` 的底层资源
    - 使用 `__HAL_RCC_CANx_CLK_ENABLE()` 启用 `CAN` 接口时钟
    - 配置 `CAN` 引脚
        - 启用 `CAN GPIOs` 时钟
        - 将 `CAN` 引脚配置为可选的开漏型
    - 如果使用中断（例如 `HAL_CAN_ActivateNotification()`）
        - 使用 `HAL_NVIC_SetPriority()` 配置 `CAN` 中断优先级
        - 使用 `HAL_NVIC_EnableIRQ()` 启用 `CAN IRQ handler`
        - 在 `CAN IRQ handler` 中，调用 `HAL_CAN_IRQHandler()`
- 使用 `HAL_CAN_Init()` 函数初始化 `CAN` 外设 该函数委托 `HAL_CAN_MspInit()` 进行初步的初始化
- 使用以下函数配置接收过滤器
    - `HAL_CAN_ConfigFilter()`
- 使用 `HAL_CAN_Start()` 函数启动 CAN 模块 此后该节点在总线上便处于活动状态：接收报文，并能发送报文 
- 为管理报文传输，可使用以下 Tx 控制函数
    - `HAL_CAN_AddTxMessage()` 用于请求传输新的报文信息 
    - `HAL_CAN_AbortTxRequest()` 用于中止待处理报文的传输 
    - `HAL_CAN_GetTxMailboxesFreeLevel()` 用来获取空闲的 Tx 邮箱的数量 
    - `HAL_CAN_IsTxMessagePending()` 用于检查 Tx 邮箱中是否有待处理的信息 
    - `HAL_CAN_GetTxTimestamp()` 当 `Time triggered communication mode` 开启时，用来获取发送的 Tx 消息的时间戳 
- 当 `CAN Rx FIFO` 收到报文时，可以使用 `HAL_CAN_GetRxMessage()` 获取，`HAL_CAN_GetRxFifoFillLevel()` 可以获取 `Rx FIFO` 中存储的报文数量
- 调用 `HAL_CAN_Stop()` 函数可停止 CAN 模块 
- 通过 `HAL_CAN_DeInit()` 函数实现去初始化 

#### 轮询模式操作

- 接收
    - 使用 `HAL_CAN_GetRxFifoFillLevel()` 监控信息接收情况，至少收到一条信息后停止监控
    - 然后使用 `HAL_CAN_GetRxMessage()` 获取信息
- 传输
    - 使用 `HAL_CAN_GetTxMailboxesFreeLevel()` 监控发送信箱的是否空闲，至少有一个发送信箱空闲后停止
    - 然后使用 `HAL_CAN_AddTxMessage()` 请求发送

#### 中断模式操作

- 使用 `HAL_CAN_ActivateNotification()` 激活通知，然后可以使用`HAL_CAN_xxxCallback()`来控制接收消息通知，该回调也是使用 `HAL_CAN_GetRxMessage()` 和 `HAL_CAN_AddTxMessage()`来实现的
- 可以使用 `HAL_CAN_DeactivateNotification()` 函数停用通知 

- 应特别注意 `CAN_IT_RX_FIFO0_MSG_PENDING` 和 `CAN_IT_RX_FIFO1_MSG_PENDING` ，这些通知会触发回调`HAL_CAN_RxFIFO0MsgPendingCallback()` 和 `HAL_CAN_RxFIFO1MsgPendingCallback()` 用户有两种可选项
    - 使用 `HAL_CAN_GetRxMessage()`在回调中直接获取 Rx 消息
    - 或者在回调中停用通知，而不获取 Rx 消息，使用 `HAL_CAN_GetRxMessage()` 获取 Rx 消息后再次激活通知 
---

上面这一部分注释详细介绍了Hal库CAN通信的使用流程，而在**接收过滤器**初始化之前的步骤，均由CubeMX工具替我们完成，我们只需调用Hal提供的api来完成剩下的步骤

同时，针对各个CAN通讯的时期，Hal库都提供了完备的回调函数，只要使用 `HAL_CAN_RegisterCallback()` 来注册对应的中断，然后覆写下面的回调函数即可，这些回调函数都会以 `__weak` 的类型声明，前面会加上 `HAL_CAN_`，他们都可以在`stm32f4xx_hal_can.c`中找到

```cpp
  The compilation define  USE_HAL_CAN_REGISTER_CALLBACKS when set to 1
  allows the user to configure dynamically the driver callbacks.
  Use Function HAL_CAN_RegisterCallback() to register an interrupt callback.

  Function HAL_CAN_RegisterCallback() allows to register following callbacks:
    (+) TxMailbox0CompleteCallback   : Tx Mailbox 0 Complete Callback.
    (+) TxMailbox1CompleteCallback   : Tx Mailbox 1 Complete Callback.
    (+) TxMailbox2CompleteCallback   : Tx Mailbox 2 Complete Callback.
    (+) TxMailbox0AbortCallback      : Tx Mailbox 0 Abort Callback.
    (+) TxMailbox1AbortCallback      : Tx Mailbox 1 Abort Callback.
    (+) TxMailbox2AbortCallback      : Tx Mailbox 2 Abort Callback.
    (+) RxFifo0MsgPendingCallback    : Rx Fifo 0 Message Pending Callback.
    (+) RxFifo0FullCallback          : Rx Fifo 0 Full Callback.
    (+) RxFifo1MsgPendingCallback    : Rx Fifo 1 Message Pending Callback.
    (+) RxFifo1FullCallback          : Rx Fifo 1 Full Callback.
    (+) SleepCallback                : Sleep Callback.
    (+) WakeUpFromRxMsgCallback      : Wake Up From Rx Message Callback.
    (+) ErrorCallback                : Error Callback.
    (+) MspInitCallback              : CAN MspInit.
    (+) MspDeInitCallback            : CAN MspDeInit.
  This function takes as parameters the HAL peripheral handle, the Callback ID
  and a pointer to the user callback function.
```

需要注意的是，`HAL_CAN_ActivateNotification()` 本质是开启硬件中断，而 `HAL_CAN_RegisterCallback()` 本质是软件层面的回调，这在源代码中可以很清楚地看出

```cpp
// in HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)

__HAL_CAN_ENABLE_IT(hcan, ActiveITs);

// in HAL_CAN_RegisterCallback(
//      CAN_HandleTypeDef *hcan, 
//      HAL_CAN_CallbackIDTypeDef CallbackID, 
//      void (* pCallback)(CAN_HandleTypeDef *_hcan)
//      )

case HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID :
        hcan->TxMailbox0CompleteCallback = pCallback;
        break;

```

上面我忽略了对于休眠模式的内容，感兴趣的可以去浏览器搜索相关api的使用

### CubeMX 配置与过滤器

囿于篇幅，这两个方面就不写在这篇文章，只是简单总结一下

配置要点是波特率的设置，正常的收发模式可以满足大部分的场景，上面配置波特率我只是一笔带过，这里在重新讲一下

波特率的计算参数主要有三个

1. `Prescalar`
2. `Time Quanta in Bit Segment 1`
3. `Time Quanta in Bit Segment 2`

记住几个常用的组合足以应对大部分需求，注意CubeMX会**判断**各个传播相位是否过小，倘若遇到无法设置的问题时，可以试试将另一个参数先调大，将之前的参数调到目标值后再调回另一个参数

而关于过滤器，要想参透原理又是需要不短的篇幅，倘若CAN设备数量不多，先用着全开放的过滤器吧，在回调函数**判断回报文头结构体的ID**就可以实现分别处理，`C610`设置ID的方法在文档中有很详细的介绍

## 文末

这篇文章写了好几天，`电路`和`马原`成绩还没有出来，`工程数学`挂了，`大学物理`及格，一众`水课`平安度过，希望能过个好年吧

<br>
<br>
<br>
<p align="right">2023年1月24日凌晨2点半结文</p>