// 头文件包含
#include <stm32f10x.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_gpio.h>
#include <math.h>
#include <stdio.h>


#include "C:\Users\13162\Desktop\嵌入式系统\3\lcd_hd44780.h"

// 定义引脚连接
#define LCD_DATA_PORT GPIOA
#define LCD_CONTROL_PORT GPIOB
#define MCP4921_PORT GPIOC

// 定义引脚位
#define LCD_DATA_PIN_OFFSET 0
#define LCD_CONTROL_PIN_OFFSET 0
#define MCP4921_SCK_PIN GPIO_Pin_0
#define MCP4921_CS_PIN GPIO_Pin_1
#define MCP4921_SDI_PIN GPIO_Pin_2
#define MCP4921_LDAC_PIN GPIO_Pin_3

#define LCD_GPIO_CLK    RCC_APB2Periph_GPIOA  // LCD引脚的时钟
#define LCD_RS_PIN      GPIO_Pin_0            // LCD RS引脚
#define LCD_EN_PIN      GPIO_Pin_1            // LCD EN引脚
#define LCD_D4_PIN      GPIO_Pin_2            // LCD D4引脚
#define LCD_D5_PIN      GPIO_Pin_3            // LCD D5引脚
#define LCD_D6_PIN      GPIO_Pin_4            // LCD D6引脚
#define LCD_D7_PIN      GPIO_Pin_5            // LCD D7引脚
#define LCD_GPIO_PORT   GPIOA                 // LCD引脚所在的GPIO端口

// 定义信号类型
typedef enum {
    SineWave,
    SquareWave,
    TriangleWave
} SignalType;

// 按钮定义
typedef enum {
    ButtonK0,
    ButtonK1,
    ButtonK2,
    ButtonK3
} Button;

// 定义频率和时间变量
uint32_t frequency = 1000;  // 初始频率为1kHz
uint16_t time = 0;          // 初始时间为0ms
SignalType signalType = SineWave;  // 全局变量定义，并初始化为正弦波

// 初始化GPIO和SPI配置
void GPIO_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    // 使能GPIO和SPI时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_SPI1, ENABLE);

		// 配置LCD数据引脚
		GPIO_InitStructure.GPIO_Pin = ((uint16_t)0xFF << LCD_DATA_PIN_OFFSET);
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(LCD_DATA_PORT, &GPIO_InitStructure);

		// 配置LCD控制引脚
		GPIO_InitStructure.GPIO_Pin = ((uint16_t)0xF << LCD_CONTROL_PIN_OFFSET);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(LCD_CONTROL_PORT, &GPIO_InitStructure);

    // 配置MCP4921引脚
    GPIO_InitStructure.GPIO_Pin = MCP4921_SCK_PIN | MCP4921_CS_PIN | MCP4921_SDI_PIN | MCP4921_LDAC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MCP4921_PORT, &GPIO_InitStructure);

    // 配置SPI1
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);
}

// 生成正弦波输出值
uint16_t GenerateSineWaveOutput(uint16_t amplitude, float frequency, uint16_t time) {
    float angle = 2 * 3.14159 * frequency * time / 1000.0;  // 角度（根据时间和频率计算）
    return amplitude * (sinf(angle) + 1.0) / 2.0;  // 返回0到amplitude之间的值
}

// 生成方波输出值
uint16_t GenerateSquareWaveOutput(uint16_t amplitude, float frequency, uint16_t time) {
    float period = 1000.0 / frequency;  // 周期（根据频率计算）
    if ((time % (uint16_t)period) < (uint16_t)(period / 2)) {
        return amplitude;  // 上半周期输出最大值
    } else {
        return 0;  // 下半周期输出最小值
    }
}

// 生成三角波输出值
uint16_t GenerateTriangleWaveOutput(uint16_t amplitude, float frequency, uint16_t time) {
    float period = 1000.0 / frequency;  // 周期（根据频率计算）
    float value = (time % (uint16_t)period) / period;  // 周期内的相对位置（0到1之间）
    if (value < 0.5) {
        return amplitude * (value * 2.0);  // 上半周期线性增加
    } else {
        return amplitude * (2.0 - value * 2.0);  // 下半周期线性减小
    }
}

// 设置DAC输出信号
void SetDACOutput(SignalType signalType) {
    uint16_t outputValue = 0;  // DAC输出值

    // 根据信号类型生成输出值
    switch (signalType) {
        case SineWave:
            // 生成正弦波输出值
            outputValue = GenerateSineWaveOutput(2047, frequency, time);
            break;
        case SquareWave:
            // 生成方波输出值
            outputValue = GenerateSquareWaveOutput(2047, frequency, time);
            break;
        case TriangleWave:
            // 生成三角波输出值
            outputValue = GenerateTriangleWaveOutput(2047, frequency, time);
            break;
        default:
            break;
    }

    // 将输出值写入MCP4921芯片
    GPIO_ResetBits(MCP4921_PORT, MCP4921_CS_PIN);  // 选择MCP4921
    SPI_I2S_SendData(SPI1, outputValue);           // 发送输出值
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET)
        ;                                          // 等待传输完成
    GPIO_SetBits(MCP4921_PORT, MCP4921_CS_PIN);     // 取消选择MCP4921
    GPIO_SetBits(MCP4921_PORT, MCP4921_LDAC_PIN);   // 更新LDAC引脚以更新输出信号
}

// 监测按钮按下事件
Button CheckButtonPressed(void) {
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == RESET) {
        return ButtonK0;
    } else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == RESET) {
        return ButtonK1;
    } else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == RESET) {
        return ButtonK2;
    } else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == RESET) {
        return ButtonK3;
    } else {
        return -1;
    }
}

// 处理按钮按下事件
void HandleButtonPress(Button button) {
    static SignalType signalType = SineWave;  // 初始信号类型为正弦波
    static uint8_t selectedBit = 0;  // 初始选择第0位
    switch (button) {
        case ButtonK0:
            // 对选中的位加1，更新频率值
            frequency += 100;  // 假设每次增加100Hz
            SetDACOutput(signalType);  // 更新DAC输出信号
            break;
        case ButtonK1:
            // 对选中的位减1，更新频率值
            if (frequency >= 100) {  // 避免频率为负值
                frequency -= 100;  // 假设每次减少100Hz
                SetDACOutput(signalType);  // 更新DAC输出信号
            }
            break;
        case ButtonK2:
						// 循环选择输出频率的某个位
						selectedBit = (selectedBit + 1) % 8;  // 循环选择下一位
						// 更新频率值，假设只更新选择位为0或1
						frequency = (frequency & ~(1 << selectedBit)) | (selectedBit < 2 ? (1 << selectedBit) : 0);
						SetDACOutput(signalType);  // 更新DAC输出信号
						break;
        case ButtonK3:
            // 切换输出信号类型
            signalType = (signalType + 1) % 3;  // 循环切换信号类型
            SetDACOutput(signalType);  // 更新DAC输出信号
            break;
        default:
            break;
    }
}


// 初始化LCD
void LCD_Initialization(void) {
    // 设置LCD引脚为输出模式
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(LCD_GPIO_CLK, ENABLE);  // 使能LCD引脚的时钟

    // 设置RS引脚
    GPIO_InitStructure.GPIO_Pin = LCD_RS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LCD_GPIO_PORT, &GPIO_InitStructure);

    // 设置EN引脚
    GPIO_InitStructure.GPIO_Pin = LCD_EN_PIN;
    GPIO_Init(LCD_GPIO_PORT, &GPIO_InitStructure);

    // 设置D4-D7引脚
    GPIO_InitStructure.GPIO_Pin = LCD_D4_PIN | LCD_D5_PIN | LCD_D6_PIN | LCD_D7_PIN;
    GPIO_Init(LCD_GPIO_PORT, &GPIO_InitStructure);

    // 初始化LCD控制器
    LCD_SendCommand(0x33);  // 初始化指令1
    LCD_SendCommand(0x32);  // 初始化指令2
    LCD_SendCommand(0x28);  // 设置4位数据总线、2行显示、字符5x8
    LCD_SendCommand(0x0C);  // 显示开、光标关闭、光标闪烁关闭
    LCD_SendCommand(0x06);  // 文本写入后光标右移，显示不移动
    LCD_SendCommand(0x01);  // 清除显示，光标复位
    Delay_ms(2);            // 等待清屏完成
}


// 显示频率值和波形
void DisplayFrequencyAndWaveform(uint32_t frequency) {
    char buffer[16];  // 用于存储频率字符串的缓冲区

    // 将频率值转换为字符串
    sprintf(buffer, "Frequency: %u Hz", frequency);

    // 在LCD上显示频率值
    LCD_SetCursor(0, 0);         // 设置光标位置为第一行第一列
    LCD_SendString(buffer);      // 发送频率字符串

    // 显示波形图标
    LCD_SetCursor(1, 0);         // 设置光标位置为第二行第一列
    LCD_SendString("Waveform:"); // 发送波形标签

    // 根据当前信号类型显示相应的波形图标
    LCD_SetCursor(1, 10);  // 设置光标位置为第二行第十一列
    switch (signalType) {
        case SineWave:
            LCD_SendString("Sin");  // 发送正弦波标签
            break;
        case SquareWave:
            LCD_SendString("Sqr");  // 发送方波标签
            break;
        case TriangleWave:
            LCD_SendString("Tri");  // 发送三角波标签
            break;
        default:
            LCD_SendString("N/A");  // 发送不可用标签
            break;
    }
}


int main(void) {
    // 初始化GPIO和SPI配置
    GPIO_Configuration();

    // 初始化LCD
    LCD_Initialization();

    // 初始频率值和时间
    uint32_t frequency = 1000;  // 初始频率为1kHz
    uint16_t time = 0;          // 初始时间为0ms

    while (1) {
        // 设置DAC输出信号
        SetDACOutput(signalType);

        // 监测按钮按下事件
        Button button = CheckButtonPressed();

        // 处理按钮按下事件
        if (button != (button)-1) {
            HandleButtonPress(button);
        }

        // 更新时间
        time++;

        // 显示频率值和波形
        DisplayFrequencyAndWaveform(frequency);
    }
}
