/******ͷ�ļ�������******/
#include "zf_common_headfile.h"
#include "electrical_machine.h"
#include "mt9v03x.h"
#include "efficient.h"
#include "isr.h"
#include "pid.h"
#include "ps2.h"
// **************************** �������� ****************************
int main (void)
{
		//���ֳ�ʼ������
    clock_init(SYSTEM_CLOCK_600M);                                              // ��ʼ��оƬʱ�� ����Ƶ��Ϊ 600MHz
    debug_init();                                                               // ��ʼ��Ĭ�� debug uart
    // �˴���д�û����� ���������ʼ�������
		overall_init();	
		system_delay_ms(300);           //�ȴ��������������ϵ����

		pit_ms_init(PIT_CH0, 10);
		pit_ms_init(PIT_CH1, 10);
		pit_ms_init(PIT_CH2, 10);

    while(1)
    {				
				
    }
}
// **************************** �������� ****************************
