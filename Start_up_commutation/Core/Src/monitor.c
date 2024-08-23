//-------------------------------------------------------------------------
// File name : monitor.c
// <Abstract>
//
// STM32F407�� Full speed USB�� �̿��� monitoring�� �����ϱ� ���� �ʿ��� �Լ� library
//
// Programmed by Prof. Lee on 7 April 2023
//-------------------------------------------------------------------------
#include "monitor.h"
#include "usbd_cdc_if.h"

ToHost toSimulink; //
extern uint8_t UserTxBufferFS[]; // usbd_cdc_if.c�� ����Ǿ� �ִ�.
extern USBD_HandleTypeDef hUsbDeviceFS; // usb_device.c�� ����

//-----------------------------------------------------------
// PC�� ������ data�� ToHost structure�� �������ش�.
// data pointer �����ϴ� �Ͱ� data�� ��, channel number �����
// ������ �ش�.
//------------------------------------------------------------
void connectData(ToHost *toHostVar, char type, char channel, void *ptr)
{
	toHostVar->NumOfToHostBlock++;  // ToHostBlock�� ���� �ϳ��� ���� 
	if(type==SS_DOUBLE) // ���� double�� data �̸�
	{
		toHostVar->NumOfDoubleData++;  // Double�� data�� ���� 1�� ����
	}
	toHostVar->TypeAndChannel[channel] = (type<<4)|(channel &0x0F);
	toHostVar->DataPtr[channel] = ptr;  // pointer ���� 
	//toHostVar->ModNumber = 510/(10*(toHostVar->NumOfToHostBlock-toHostVar->NumOfDoubleData) + 20*toHostVar->NumOfDoubleData + 2);
}


void putcToBuffer(ToHost *toHostVar, char c)
{
	toHostVar->data[toHostVar->indx++] = c;
	return;
}


void outhex8_Buffer(ToHost *toHostVar, char n)
{
	char c;
	c = (n >> 4) & 0x0000000f;
	if (c > 9)  toHostVar->data[toHostVar->indx++] = c - 10 + 'a';
	else toHostVar->data[toHostVar->indx++] = c+'0';

	c = n & 0x0000000f;
	if (c > 9) toHostVar->data[toHostVar->indx++] = c - 10 + 'a';
	else toHostVar->data[toHostVar->indx++] = c+'0';

	return;
}

void outhex32_Buffer(ToHost *toHostVar, unsigned long n)
{
	outhex8_Buffer(toHostVar, (char) (n >> 24) );
	outhex8_Buffer(toHostVar, (char) (n >> 16) );	
	outhex8_Buffer(toHostVar, (char) (n >> 8) );	
	outhex8_Buffer(toHostVar, (char) n);
}

//------------------------------------------------------------------
// EZUSB�� ������� ������ polling�� ������� �ʰ� �׳� data�� EZUSB���ٰ�
// write �� �� �־��µ� STM32F407�� Full speed USB�� Virtual COM���� �����
// ��쿡�� ������ write �� ���� ����. ���� ���� �Ǿ ���ư��� PC ��
// application�� ���ư��� ���� ��쿡�� lock �ɸ��Ƿ� USB buffer�� �ٷ�
// ���� ���� ������ software buffer���ٰ� ������ data�� ��Ƹ� ����. �׷�����
// �����ؾ� �� ������ �Ǹ� USB�� tx_buffer�� Ȯ���Ǿ� �ִ����� Ȯ���ϰ�
// ���� Ȯ���� �Ǿ��ٸ� tx_buffer���ٰ� write �ϸ� �ǰ� ���� tx_buffer��
// Ȯ���� ���� �ʾҴٸ� �׳� �ƹ��ϵ� ���ϰ� �Ѿ�� �ȴ�. �׸��� 
// tx_buffer�� Ȯ���ɶ����� software buffer�� ���̻� ä������ �ȴ´�.
// �Ʒ��� �Լ��� PC������ �����ؾ� �� data�� ���� ���ŵǾ��� ��� 
// �����ؾ� �� ������ data�� ����� ���� ���� software buffer���ٰ�
// ä���ְ� �����ؾ� �� ������ �Ǿ��� �� USB�� tx_buffer���ٰ� 
// write �ϴ� ������ �ϴ� �Լ��̴�.
// Data�� hex encoding�� ����ϰ� Ư�� ����Ÿ�Ӷ��� data�� ����
// ����Ÿ�Ӷ��� data�� �����ϱ� ���� �����ڷν� 'T'�� 'Q'�� ����Ѵ�.
// �׸��� �����ؾ� �� data�� ���������� 'Z'�� ����Ѵ�. ���� 
// ������ �� data�� 2��, Modulus number�� 2�� ��� data�� ���´� 
// ������ ����.
// T_XXXXXXXXXX_XXXXXXXXXX_Q_T_XXXXXXXXXX_XXXXXXXXXX_Q_Z
// ���� ���� data�� ���� n, modulus number�� mod ��� ���� ��
// �ѹ��� �����ϴ� data�� ���Ե� byte�� ���� ������ ���� ���ȴ�.
// 
// # of bytes = (n*10+2)*mod+1
//
// ���⼭ 10�� �ϳ��� data�� hex encoding �ǰ� ������ byte ��
// 2�� 'T'�� 'Q'�� ���� byte ��
// �������� 1�� 'Z'�� ���� byte ��
//
// ���� Full speed USB�� ��� Buffer size�� 512�� ������Ƿ�
//
// ���� ���Ŀ� ���� ���� �� byte ���� 512���� 1���� 511���� ����
// ���� �����Բ� mod�� ����ؾ� �Ѵ�. ���� �̰��� �������� �����
// ���� 
// 
//  mod = 510/(n*10+2) <== ���������̹Ƿ� ���� ������ ���´�.
//
// ���� ��� n=2�� ��� 
// 
// mod = 510/(2*10+2) = 23.18 => mod = 23
//------------------------------------------------------------------
void ProcessToHostBlock()
{
	unsigned int i, n;
	signed long temp_int32;
	char type;
	static int flag = 0;
	
	n = toSimulink.NumOfToHostBlock;

	if(toSimulink.UpdateDoneFlag)
	{
		if(toSimulink.SendHoldFlag==0)
		{
			putcToBuffer(&toSimulink, 'T');  // 
			for(i=0;i<n;i++)
			{
				//---------------------------------------------------------
				// For-loop�� ���鼭 �ش� block�� �Է��� ������ �Ǿ�
				// �ִ� ���¶�� Host������ �����Ѵ�.
				// ������ �Ǿ������� ���δ� UpdateDoneFlag[]�� �˻��Ѵ�.
				//---------------------------------------------------------
				//TypeAndChannel[i]�� ����
				// ���� 4-bit�� channel number, ���� 4-bit�� data type.
				// data type�� ���� ������ �Ʒ��� �־��� �ִ�.
				//---------------------------------------------------------

				if(GetBit(toSimulink.UpdateDoneFlag,i))  // ������ �� ���
				{
					outhex8_Buffer(&toSimulink, toSimulink.TypeAndChannel[i]); // channel number ����

					type = (toSimulink.TypeAndChannel[i] >> 4) & 0x0F;  // data type�� �˾Ƴ���.

					//--------------------------------------------
					// data type
					//--------------------------------------------
					// SS_DOUBLE  =  0,    64-bit double �� data 
					// SS_SINGLE  =  1,    /* real32_T  */
					// SS_INT8    =  2,    /* int8_T    */
					// SS_UINT8   =  3,    /* uint8_T   */
					// SS_INT16   =  4,    /* int16_T   */
					// SS_UINT16  =  5,    /* uint16_T  */
					// SS_INT32   =  6,    /* int32_T   */
					// SS_UINT32  =  7,    /* uint32_T  */
					// SS_BOOLEAN =  8     /* boolean_T */
					//--------------------------------------------
					switch(type)
					{
						case 0:  // double
							outhex32_Buffer(&toSimulink, *(unsigned long*)(toSimulink.DataPtr[i])); // double �� data�� ���� 4-byte ���� ����
							
							//---------------------------------------------------------------------------
							// ���� 4 byte�� ������ ���� ���� 4 byte�� ������ ���� �ٽ� type�� channel 
							// ������ �����Ѵ�. ���⼭ type�� double�� data�� ��Ÿ���� 0�� �ƴ϶� 9�� 
							// ����ϱ�� �Ѵ�. ���⼭ 9�� �ǹ��ϴ� ���� double�� data�� �ι�° data��� 
							// ���� �ǹ��Ѵ�. ������������ �� �����κ��� data�� �̾�ٿ� ��� �Ǵ� ���� 
							// �� �� �ְų� Ȥ���� type �� 0 �̿��µ� �� ������ 9�� ������ ������ 
							// ���� ó���� �� �ֵ��� �Ѵ�.
							//-------------------------------------------------------------------------
							type = toSimulink.TypeAndChannel[i];
							type = type & 0x0F;  // ���� 4 bit�� clear ��Ű�� ���� 4 bit(channel ����)�� �״�� ����. 
							type = type | 0x90;  // ���� 4 bit�� 9�� ��ġ��Ų��. double �� data�� 10 byte 2���� ������ ������
							                     // �ϹǷ� ���� 4 byte�� 9��� ���� double �� data�� 2��° data�� �ǹ�
							outhex8_Buffer(&toSimulink, type);  // 2��° data�� �ǹ�
							outhex32_Buffer(&toSimulink, *((unsigned long*)(toSimulink.DataPtr[i])+1)); // double �� data�� ���� 4-byte ����
							break;
						case 1:  // signle data
						case 6:  // int 32
						case 7:  // uinit 32
							outhex32_Buffer(&toSimulink, *(unsigned long*)(toSimulink.DataPtr[i])); // channel data ����
							break;
						case 2:  // int 8
						case 4:  // int 16
							temp_int32 = (signed long)(*(signed int *)(toSimulink.DataPtr[i]));
							outhex32_Buffer(&toSimulink,*(unsigned long*)&temp_int32);
							break;
						case 3:  // uint 8
						case 5:  // uint 16
						case 8:  // bool
							temp_int32 = (signed long)(*(unsigned int *)(toSimulink.DataPtr[i]));
							outhex32_Buffer(&toSimulink, *(unsigned long*)&temp_int32);
							break;
						default:
							break;
					}

					//--------------------------------------------
     				// �����ϰ� ���� Flag resetting. �̷��� �ؾ���
					// ������ �� �����ϰ� ���� Flag=1�� �����ؼ�
					// ���ſ��θ� �Ǵ��� �� �����ϰ� �Ǵ� ���̴�.
					//--------------------------------------------
					ClearBit(toSimulink.UpdateDoneFlag,i);
				}
			}

			putcToBuffer(&toSimulink,'Q');  // Frame End
		
			toSimulink.ModCntr++;
			toSimulink.ModCntr%=toSimulink.ModNumber;
			if(toSimulink.ModCntr==0)
			{
				putcToBuffer(&toSimulink,'Z');
			}
		}
		
		//--------------------------------------------------------------------------
		// ModCntr=0 �̸� data�� �����ؾ� �� ������ �ǹ��Ѵ�. ���� �� ������
		// tx_buffer�� Ȯ���� �ȵǾ��ִٸ� data�� ������ �ʰ� SendHoldFlag�� true��
		// �����ؼ� buffer�� Ȯ���ɶ������� ���̻� toSimulink�� ä���ִ�
		// ���� ���� �ʵ��� �Ѵ�. PC���� Simulink�� �������� ������ PC ������
		// data�� �о�� �����Ƿ� ���⼭ �����ϴ� ������ �߻��Ѵ�. 
		// PC�� Simulink�� �������� �ʴ��� �̷� ���� u-controller ����
		// application�� Lock �Ǵ� ��찡 �߻��ϸ� �ȵǹǷ� �̸� �ذ��ϱ� ����
		// ��� �Ʒ� �κ��� �߰��ȴ�. �ϴ� SendHoldFlag=true�� �Ǹ� ���̻� 
		// ModCntr�� �������� �����Ƿ� ����ؼ� 0���� ���� �ְ� �ȴ�. ����
		// ���� iteration ���� ����� ������ �ǰ� ���� tx_buffer�� Ȯ���Ǹ�
		// data�� ������ �ȴ�. �׷��� �Ǹ� �ڿ������� SendHoldFlag �� �ٽ� false
		// �� �ǹǷ� �������� ���۵����� ����� �� �ִ�. 
		// SendHoldFlag �� true��� ���� Sending ������ Hold �ϰ� �ִ� ���¸�
		// �ǹ��Ѵ�
		//--------------------------------------------------------------------------
		if(toSimulink.ModCntr==0)  // <-- �����ؾ� �� ������ �ǹ� 
		{

			USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
			if (hcdc->TxState != 0){
				toSimulink.SendHoldFlag = 1; // �غ� �ȵ� SendHoldFlag�� 1�� ����
			}
			else
			{

				memcpy(UserTxBufferFS, toSimulink.data, toSimulink.indx);
				CDC_Transmit_FS(UserTxBufferFS, toSimulink.indx);
				toSimulink.indx = 0;  // ������ ���̹Ƿ� toSimulink���� data�� 0���� ������ �� ���´�.
				toSimulink.SendHoldFlag = 0; // ���� ������ ���̹Ƿ� SendHoldFlag�� 0�� ����
			}
		}
	}
	return;
}
