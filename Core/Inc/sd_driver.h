/*
 * sd_driver.h
 *
 *  Created on: Jun 19, 2017
 *      Author: schaefer
 */

#ifndef SD_DRIVER_H_
#define SD_DRIVER_H_

#include "FreeRTOS_wrapper.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "my_assert.h"

class SD_driver_t
{
public:
	SD_driver_t( void)
	: SDPath({0})
	{
	wipe( SDFatFs);
	if( ! (
		(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
		&&
		(f_mount( &SDFatFs, (TCHAR const*) SDPath, 0) == FR_OK)
	  )   )
		{
			wipe( SDFatFs);
		}
	}
	bool healthy( void)
	{
		return SDPath[0] != 0;
	}
private:
	FATFS SDFatFs; /* File system object for SD card logical drive */
	char SDPath[4]; /* SD card logical drive path */
};

class out_file
{
public:
	out_file( char * filename, bool overwrite=false)
	: open( false)
	{
		if( f_open(&MyFile, filename,
			   (overwrite ? FA_CREATE_ALWAYS : FA_CREATE_NEW) | FA_WRITE) != FR_OK)
			wipe( MyFile);
		else
			open=true;
	}
	bool fopen( char * filename)
	{
		if( open)
			return false;
		if( f_open(&MyFile, filename, FA_CREATE_NEW | FA_WRITE) != FR_OK)
		{
			wipe( MyFile);
			return false;
		}
		open=true;
		return true;
	}
	bool healthy( void) const
	{
		return open;
	}
	bool write( char * data, uint16_t length = 0)
	{
		UINT byteswritten;
		if( length == 0)
			length = strlen( data);
		return(
				( f_write(&MyFile, data, length, &byteswritten) == FR_OK)
				&&
				(byteswritten == length)
			  );
	}
	bool close( void)
	{
		return f_close(&MyFile) == FR_OK;
	}
	bool sync( void)
	{
		return f_sync(&MyFile) == FR_OK;
	}
	~out_file(void)
	{
		close();
	}
private:
	FIL MyFile; /* File object */
	bool open;
};

#endif /* SD_DRIVER_H_ */
