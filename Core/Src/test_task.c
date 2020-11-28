

void StartTestTask(void const * argument)
{
  /* init code for USB_DEVICE */
  //MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

	/*
	FRESULT fresult;
  FIL fp;
  uint8_t sd_card_detect = 0;
  sd_card_detect = BSP_PlatformIsDetected();
  fresult = f_mount(&fatfs, "", 0);
  uint8_t testString[] = "The Soar Instrument..";
  uint32_t writtenBytes = 0;
  if (FR_OK == fresult)
  {
	  fresult = f_open(&fp, "testabc.txt",FA_CREATE_ALWAYS | FA_WRITE);

	  if (FR_OK == fresult)
	  {
		  fresult = f_write (&fp, testString, 19, (UINT*)&writtenBytes);
	  }
  }
  fresult = f_close (&fp);

*/
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}
