#ifndef drv8711_h
#define drv8711_h

#include <Arduino.h>
#include "registers.h"
#include "SPI.h"

// set debug to 1 for extended serial logging
#define debug 0 

#define DTIME_400 0x00
#define DTIME_450 0x01
#define DTIME_650 0x02
#define DTIME_850 0x03

#define ISGAIN_5  	0x00
#define ISGAIN_10 	0x01
#define ISGAIN_20 	0x02
#define ISGAIN_40 	0x03

#define STEPS_1 	0x00
#define STEPS_2 	0x01
#define STEPS_4 	0x02
#define STEPS_8 	0x03
#define STEPS_16 	0x04
#define STEPS_32 	0x05
#define STEPS_64 	0x06
#define STEPS_128 	0x07
#define STEPS_256 	0x08

#define SIMPLTH_50 	0x00
#define SIMPLTH_100 0x01
#define SIMPLTH_200 0x02
#define SIMPLTH_300 0x03
#define SIMPLTH_400 0x04
#define SIMPLTH_600 0x05
#define SIMPLTH_800 0x06
#define SIMPLTH_1000 0x07

#define DECMOD_SLOW 	0x00
#define DECMOD_SLOWMIX  0x01
#define DECMOD_FAST 	0x02
#define DECMOD_MIXED 	0x03
#define DECMOD_SLOWAUTO 0x04
#define DECMOD_MIXAUTO 	0x05

#define VDIV_32		0x00
#define VDIV_16		0x01
#define VDIV_8		0x02
#define VDIV_4		0x03

#define SDCNT_1		0x00
#define SDCNT_2		0x01
#define SDCNT_4		0x02
#define SDCNT_8		0x03

#define IDRIVEP_50	0x00
#define IDRIVEP_100	0x01
#define IDRIVEP_150 0x02
#define IDRIVEP_200	0x03

#define IDRIVEN_100	0x00
#define IDRIVEN_200	0x01
#define IDRIVEN_300	0x02
#define IDRIVEN_400	0x03

#define TDRIVEP_250	0x00
#define TDRIVEP_500 0x01
#define TDRIVEP_1000 0x02
#define TDRIVEP_2000 0x03

#define TDRIVEN_250 0x00
#define TDRIVEN_500 0x01
#define TDRIVEN_1000 0x02
#define TDRIVEN_2000 0x03

#define OCPDEG_1	0x00
#define OCPDEG_2	0x01
#define OCPDEG_4	0x02
#define OCPDEG_8	0x03

#define OCPTH_250	0x00
#define OCPTH_500	0x01
#define OCPTH_750	0x02
#define OCPTH_1000	0x03

#define ON			0x01
#define OFF			0x00 

#define REGWRITE    0x0000
#define REGREAD     0x8000

class drv8711
{
 public:
  drv8711 (int pin);

  struct CTRL_Register 		G_CTRL_REG;
  struct TORQUE_Register 	G_TORQUE_REG;
  struct OFF_Register 		G_OFF_REG;
  struct BLANK_Register		G_BLANK_REG;
  struct DECAY_Register 	G_DECAY_REG;
  struct STALL_Register 	G_STALL_REG;
  struct DRIVE_Register 	G_DRIVE_REG;
  struct STATUS_Register 	G_STATUS_REG;

  bool SavedStart; 
  bool ErrorFlag;
  
  void stepper_motor_init(); 
  void actuator_init();
  void enable ();  
  void disable ();
  void get_status ();
  void clear_status ();
  void clear_error ();
  void set_stepper_motor_defaults ();
  void set_actuator_defaults();
  
  void ReadAllRegisters () ;
  void ReadCTRLRegister();
  void ReadTORQUERegister();	
  void ReadOFFRegister();
  void ReadBLANKRegister();
  void ReadDECAYRegister();
  void ReadSTALLRegister();
  void ReadDRIVERegister();
  void ReadSTATUSRegister();
  
  void WriteAllRegisters () ;
  void WriteCTRLRegister();
  void WriteTORQUERegister();	
  void WriteOFFRegister();
  void WriteBLANKRegister();
  void WriteDECAYRegister();
  void WriteSTALLRegister();
  void WriteDRIVERegister();
  void WriteSTATUSRegister();

private:
 
  byte _pin;
  
  unsigned int SPI_ReadWrite(unsigned int sendData);
  bool SPI_VerifiedWrite(unsigned int sendData);
 
 };


 

#endif