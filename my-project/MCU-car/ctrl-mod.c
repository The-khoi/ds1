#include "head.h"

//����ģʽ�л�
//0x05�ֶ� 0x06�Զ�
void ctrlModSwitch(uint mod)
{
    switch (mod)
    {
    case 0x05://�ֶ�
        blutoothInit();  //ͨѶ���ڳ�ʼ�����ã�ʹ�ü�����1��

        break;
    case 0x06://�Զ�
        speedTimerInit(); //AI�ҵ���ʱ����ʼ����ʹ�ü�����1��

        break;
    default:
        break;
    }
}