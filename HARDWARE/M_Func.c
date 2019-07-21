/* Includes ------------------------------------------------------------------*/
#include "MyTypeDef.h"
#include "lcd.h"

/* Private typedef -----------------------------------------------------------*/
//typedef void(* func)(void);

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u8 KeyBoard_State = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Show_Keyboard(void)
{
	u8 i;
	
	LCD_Clear(WHITE);
	POINT_COLOR=BLACK;
	if (KeyBoard_State == 0)
	{
		LCD_DrawLine(0, lcddev.height-1-48, lcddev.width-1, lcddev.height-1-48);				//画线
		LCD_DrawLine(lcddev.width/2, lcddev.height-1-48, lcddev.width/2, lcddev.height-1);		//画线
		LCD_ShowString(lcddev.width/4-24, lcddev.height-1-36, 200, 24, 24,"Back");
		LCD_ShowString(3*lcddev.width/4-48, lcddev.height-1-36, 200, 24, 24,"Keyboard");
	}
	else
	{
		for (i = 1; i <= 3; ++i)
		{
			LCD_DrawLine(0, lcddev.height-1-i*48, lcddev.width-1, lcddev.height-1-i*48);					//画线
			LCD_DrawLine(i*lcddev.width/4, lcddev.height-1-4*48, i*lcddev.width/4, lcddev.height-1);		//画线
		}
		LCD_DrawLine(0, lcddev.height-1-4*48, lcddev.width-1, lcddev.height-1-4*48);						//画线
		
		LCD_ShowString(lcddev.width/8-24, lcddev.height-1-48+12, 200, 24, 24,"Sign");
		LCD_ShowString(3*lcddev.width/8-6, lcddev.height-1-48+12, 200, 24, 24,"0");
		LCD_ShowString(5*lcddev.width/8-6, lcddev.height-1-48+12, 200, 24, 24,".");
		LCD_ShowString(7*lcddev.width/8-24, lcddev.height-1-48+12, 200, 24, 24,"Hide");
		LCD_ShowString(lcddev.width/8-6, lcddev.height-1-2*48+12, 200, 24, 24,"1");
		LCD_ShowString(3*lcddev.width/8-6, lcddev.height-1-2*48+12, 200, 24, 24,"2");
		LCD_ShowString(5*lcddev.width/8-6, lcddev.height-1-2*48+12, 200, 24, 24,"3");
		LCD_ShowString(7*lcddev.width/8-24, lcddev.height-1-2*48+12, 200, 24, 24,"Back");
		LCD_ShowString(lcddev.width/8-6, lcddev.height-1-3*48+12, 200, 24, 24,"4");
		LCD_ShowString(3*lcddev.width/8-6, lcddev.height-1-3*48+12, 200, 24, 24,"5");
		LCD_ShowString(5*lcddev.width/8-6, lcddev.height-1-3*48+12, 200, 24, 24,"6");
		LCD_ShowString(7*lcddev.width/8-36, lcddev.height-1-3*48+12, 200, 24, 24,"Delete");
		LCD_ShowString(lcddev.width/8-6, lcddev.height-1-4*48+12, 200, 24, 24,"7");
		LCD_ShowString(3*lcddev.width/8-6, lcddev.height-1-4*48+12, 200, 24, 24,"8");
		LCD_ShowString(5*lcddev.width/8-6, lcddev.height-1-4*48+12, 200, 24, 24,"9");
		LCD_ShowString(7*lcddev.width/8-12, lcddev.height-1-4*48+12, 200, 24, 24,"OK");
	}
}

/********************************    END OF FILE    ***************************/

