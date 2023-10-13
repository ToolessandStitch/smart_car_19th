/******头文件的声明******/
#include "zf_common_headfile.h"
#include "electrical_machine.h"
#include "mt9v03x.h"
#include "efficient.h"
#include "isr.h"
#include "pid.h"
#include "ps2.h"
// **************************** 代码区域 ****************************
int main (void)
{
		//各种初始化部分
    clock_init(SYSTEM_CLOCK_600M);                                              // 初始化芯片时钟 工作频率为 600MHz
    debug_init();                                                               // 初始化默认 debug uart
    // 此处编写用户代码 例如外设初始化代码等
		overall_init();	
		system_delay_ms(300);           //等待主板其他外设上电完成

		pit_ms_init(PIT_CH0, 10);
		pit_ms_init(PIT_CH1, 10);
		pit_ms_init(PIT_CH2, 10);

    while(1)
    {				
				
    }
}
// **************************** 代码区域 ****************************
