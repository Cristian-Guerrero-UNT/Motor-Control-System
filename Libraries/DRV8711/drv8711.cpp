#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "drv8711.h"

drv8711::drv8711 (int pin)
{
	pinMode(pin, OUTPUT);
	_pin = pin;
	SavedStart = OFF;
	ErrorFlag = false;
	
	SPI.begin(); 
    SPI.setClockDivider(SPI_CLOCK_DIV16); // The default SPI_CLOCK_DIV4 causes errors while communicating. 
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
 }

void drv8711::stepper_motor_init () 
{
    set_stepper_motor_defaults();
    WriteAllRegisters();
 }

void drv8711::actuator_init () 
{
    set_actuator_defaults();
    WriteAllRegisters();
 }

void drv8711::set_stepper_motor_defaults ()
{
 	// CTRL Register
	G_CTRL_REG.Address 	= 0x00;
	G_CTRL_REG.DTIME 	= DTIME_850; 	//Dead Time in ns
	G_CTRL_REG.ISGAIN   = ISGAIN_10;	//ISENSE amp gain 
	G_CTRL_REG.EXSTALL 	= OFF;			//External Stall Detect
	G_CTRL_REG.MODE     = STEPS_4;		//Microstepping Mode
	G_CTRL_REG.RSTEP 	= OFF;			//Step 
	G_CTRL_REG.RDIR 	= OFF;			//Reverse Direction Input
	G_CTRL_REG.ENBL 	= OFF;			//Enable Motor
	
	// TORQUE Register
	G_TORQUE_REG.Address = 0x01;
	G_TORQUE_REG.SIMPLTH = SIMPLTH_100;	//Back EMF Sample Threshold uS
	G_TORQUE_REG.TORQUE  = 208;			//Torque Value 0..255
        
	// OFF Register
	G_OFF_REG.Address 	= 0x02;
	G_OFF_REG.PWMMODE 	= OFF;			//PWM Direct Mode - OFF for Indexed Mode 
	G_OFF_REG.TOFF 		= 47;			//Off Time 0..255 in 500ns inc.

	// BLANK Register
	G_BLANK_REG.Address = 0x03;			
	G_BLANK_REG.ABT 	= OFF;			//Enable Adaptive Blanking Time
	G_BLANK_REG.TBLANK 	= 125;			//Blanking Time 0..255 in 20ns inc ( 1us min )

	// DECAY Register.
	G_DECAY_REG.Address = 0x04;
	G_DECAY_REG.DECMOD  = DECMOD_MIXAUTO;	//Decay Mode
	G_DECAY_REG.TDECAY 	= 15;				//Decay Time 0..255 * 500ns

	// STALL Register
	G_STALL_REG.Address = 0x05;
	G_STALL_REG.VDIV 	= VDIV_4;		//Back EMF Div factor
	G_STALL_REG.SDCNT 	= SDCNT_8;		//Stall Step Count 
	G_STALL_REG.SDTHR 	= 0 ;			//Stall Detection Threshold 0..255

	// DRIVE Register
	G_DRIVE_REG.Address = 0x06;
	G_DRIVE_REG.IDRIVEP = IDRIVEP_50;	//High Side Gate Current mA (Source)
	G_DRIVE_REG.IDRIVEN = IDRIVEN_100;	//Low Side Gate Current mA (Sink)
	G_DRIVE_REG.TDRIVEP = TDRIVEP_500;	//High Side Gate Drive Time ns 
	G_DRIVE_REG.TDRIVEN = TDRIVEN_500;	//Low Side Gate Drive Time ns 
	G_DRIVE_REG.OCPDEG  = OCPDEG_2;		//OCP Deglitch time uS  
	G_DRIVE_REG.OCPTH   = OCPTH_500;	//OCP Threshold mV

	// STATUS Register
	G_STATUS_REG.Address = 0x07;
	G_STATUS_REG.STDLAT  = OFF;			//Latched Stall Detect
	G_STATUS_REG.STD     = OFF;  		//Stall Detected 
	G_STATUS_REG.UVLO    = OFF;  		//UnderVoltage Lockout  
	G_STATUS_REG.BPDF    = OFF; 	 	//Channel B Predriver Fault 
	G_STATUS_REG.APDF    = OFF;			//Channel A Predriver Fault 
	G_STATUS_REG.BOCP    = OFF;			//Channel B OverCurrent 
	G_STATUS_REG.AOCP    = OFF;			//Channel A OverCurrent 
	G_STATUS_REG.OTS     = OFF;			//Over Temperature 
}

void drv8711::set_actuator_defaults ()
{
 	// CTRL Register
	G_CTRL_REG.Address 	= 0x00;
	G_CTRL_REG.DTIME 	= DTIME_850; 	//Dead Time in ns
	G_CTRL_REG.ISGAIN   = ISGAIN_10;	//ISENSE amp gain 
	G_CTRL_REG.EXSTALL 	= OFF;			//External Stall Detect
	G_CTRL_REG.MODE     = STEPS_32;		//Microstepping Mode
	G_CTRL_REG.RSTEP 	= OFF;			//Step 
	G_CTRL_REG.RDIR 	= OFF;			//Reverse Direction Input
	G_CTRL_REG.ENBL 	= OFF;			//Enable Motor
	
	// TORQUE Register
	G_TORQUE_REG.Address = 0x01;
	G_TORQUE_REG.SIMPLTH = SIMPLTH_100;	//Back EMF Sample Threshold uS
	G_TORQUE_REG.TORQUE  = 128;			//Torque Value 0..255
        
	// OFF Register
	G_OFF_REG.Address 	= 0x02;
	G_OFF_REG.PWMMODE 	= ON;			//PWM Direct Mode - OFF for Indexed Mode 
	G_OFF_REG.TOFF 		= 1;			//Off Time 0..255 in 500ns inc.

	// BLANK Register
	G_BLANK_REG.Address = 0x03;			
	G_BLANK_REG.ABT 	= OFF;			//Enable Adaptive Blanking Time
	G_BLANK_REG.TBLANK 	= 125;			//Blanking Time 0..255 in 20ns inc ( 1us min )

	// DECAY Register.
	G_DECAY_REG.Address = 0x04;
	G_DECAY_REG.DECMOD  = DECMOD_SLOW;	//Decay Mode
	G_DECAY_REG.TDECAY 	= 15;				//Decay Time 0..255 * 500ns

	// STALL Register
	G_STALL_REG.Address = 0x05;
	G_STALL_REG.VDIV 	= VDIV_4;		//Back EMF Div factor
	G_STALL_REG.SDCNT 	= SDCNT_8;		//Stall Step Count 
	G_STALL_REG.SDTHR 	= 0 ;			//Stall Detection Threshold 0..255

	// DRIVE Register
	G_DRIVE_REG.Address = 0x06;
	G_DRIVE_REG.IDRIVEP = IDRIVEP_50;	//High Side Gate Current mA (Source)
	G_DRIVE_REG.IDRIVEN = IDRIVEN_100;	//Low Side Gate Current mA (Sink)
	G_DRIVE_REG.TDRIVEP = TDRIVEP_500;	//High Side Gate Drive Time ns 
	G_DRIVE_REG.TDRIVEN = TDRIVEN_500;	//Low Side Gate Drive Time ns 
	G_DRIVE_REG.OCPDEG  = OCPDEG_2;		//OCP Deglitch time uS  
	G_DRIVE_REG.OCPTH   = OCPTH_500;	//OCP Threshold mV

	// STATUS Register
	G_STATUS_REG.Address = 0x07;
	G_STATUS_REG.STDLAT  = OFF;			//Latched Stall Detect
	G_STATUS_REG.STD     = OFF;  		//Stall Detected 
	G_STATUS_REG.UVLO    = OFF;  		//UnderVoltage Lockout  
	G_STATUS_REG.BPDF    = OFF; 	 	//Channel B Predriver Fault 
	G_STATUS_REG.APDF    = OFF;			//Channel A Predriver Fault 
	G_STATUS_REG.BOCP    = OFF;			//Channel B OverCurrent 
	G_STATUS_REG.AOCP    = OFF;			//Channel A OverCurrent 
	G_STATUS_REG.OTS     = OFF;			//Over Temperature 
}
 
void drv8711::enable ()
{
	// Set Enable
	G_CTRL_REG.ENBL 	= ON;  //enable motor
	WriteCTRLRegister();
}

void drv8711::disable ()
{
	// Set Disable Bit
	G_CTRL_REG.ENBL 	= OFF;  //disable motor
	WriteCTRLRegister();
}

void drv8711::get_status ()
{
	ReadSTATUSRegister();
}

void drv8711::clear_status ()
{
	G_STATUS_REG.STDLAT  = OFF;
	G_STATUS_REG.STD     = OFF;
	G_STATUS_REG.UVLO    = OFF;
	G_STATUS_REG.BPDF    = OFF;
	G_STATUS_REG.APDF    = OFF;
	G_STATUS_REG.BOCP    = OFF;
	G_STATUS_REG.AOCP    = OFF;
	G_STATUS_REG.OTS     = OFF;

	WriteSTATUSRegister();
}

void drv8711::clear_error()
{
	ErrorFlag = false;
}

void drv8711::ReadCTRLRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read CTRL Register
    sendData = REGREAD | (G_CTRL_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_CTRL_REG.DTIME        = ((readData >> 10) & 0x0003);
    G_CTRL_REG.ISGAIN       = ((readData >> 8) & 0x0003);
    G_CTRL_REG.EXSTALL      = ((readData >> 7) & 0x0001);
    G_CTRL_REG.MODE         = ((readData >> 3) & 0x000F);
    G_CTRL_REG.RSTEP        = ((readData >> 2) & 0x0001);
    G_CTRL_REG.RDIR         = ((readData >> 1) & 0x0001);
    G_CTRL_REG.ENBL         = ((readData >> 0) & 0x0001);
}

void drv8711::ReadTORQUERegister()
{
	unsigned int sendData = 0;
    unsigned int readData = 0;

	// Read TORQUE Register
    sendData = REGREAD | (G_TORQUE_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_TORQUE_REG.SIMPLTH    = ((readData >> 8) & 0x0007);
    G_TORQUE_REG.TORQUE     = ((readData >> 0) & 0x00FF);
}

void drv8711::ReadOFFRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read OFF Register
    sendData = REGREAD | (G_OFF_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_OFF_REG.PWMMODE       = ((readData >> 8) & 0x0001);
    G_OFF_REG.TOFF          = ((readData >> 0) & 0x00FF);
}

void drv8711::ReadBLANKRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read BLANK Register
    sendData = REGREAD | (G_BLANK_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_BLANK_REG.ABT         = ((readData >> 8) & 0x0001);
    G_BLANK_REG.TBLANK      = ((readData >> 0) & 0x00FF);
}

void drv8711::ReadDECAYRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read DECAY Register
    sendData = REGREAD | (G_DECAY_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_DECAY_REG.DECMOD      = ((readData >> 8) & 0x0007);
    G_DECAY_REG.TDECAY      = ((readData >> 0) & 0x00FF);
}

void drv8711::ReadSTALLRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read STALL Register
    sendData = REGREAD | (G_STALL_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_STALL_REG.VDIV        = ((readData >> 10) & 0x0003);
    G_STALL_REG.SDCNT       = ((readData >> 8) & 0x0003);
    G_STALL_REG.SDTHR       = ((readData >> 0) & 0x00FF);
}

void drv8711::ReadDRIVERegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read DRIVE Register
    sendData = REGREAD | (G_DRIVE_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_DRIVE_REG.IDRIVEP     = ((readData >> 10) & 0x0003);
    G_DRIVE_REG.IDRIVEN     = ((readData >> 8) & 0x0003);
    G_DRIVE_REG.TDRIVEP     = ((readData >> 6) & 0x0003);
    G_DRIVE_REG.TDRIVEN     = ((readData >> 4) & 0x0003);
    G_DRIVE_REG.OCPDEG      = ((readData >> 2) & 0x0003);
    G_DRIVE_REG.OCPTH       = ((readData >> 0) & 0x0003);
}

void drv8711::ReadSTATUSRegister()
{
    unsigned int sendData = 0;
    unsigned int readData = 0;

    // Read STATUS Register
    sendData = REGREAD | (G_STATUS_REG.Address << 12);
    readData = SPI_ReadWrite(sendData);
    G_STATUS_REG.STDLAT     = ((readData >> 7) & 0x0001);
    G_STATUS_REG.STD        = ((readData >> 6) & 0x0001);
    G_STATUS_REG.UVLO       = ((readData >> 5) & 0x0001);
    G_STATUS_REG.BPDF       = ((readData >> 4) & 0x0001);
    G_STATUS_REG.APDF       = ((readData >> 3) & 0x0001);
    G_STATUS_REG.BOCP       = ((readData >> 2) & 0x0001);
    G_STATUS_REG.AOCP       = ((readData >> 1) & 0x0001);
    G_STATUS_REG.OTS        = ((readData >> 0) & 0x0001);
}

void drv8711::ReadAllRegisters()
{
	ReadCTRLRegister();
	ReadTORQUERegister();	
	ReadOFFRegister();
	ReadBLANKRegister();
	ReadDECAYRegister();
	ReadSTALLRegister();
	ReadDRIVERegister();
	ReadSTATUSRegister();
}

void drv8711::WriteCTRLRegister()
{
    unsigned int sendData = 0;
    
	// Write CTRL Register
    sendData = REGWRITE | (G_CTRL_REG.Address << 12) | (G_CTRL_REG.DTIME << 10) | (G_CTRL_REG.ISGAIN << 8);
    sendData |= (G_CTRL_REG.EXSTALL << 7) | (G_CTRL_REG.MODE << 3) | (G_CTRL_REG.RSTEP << 2) | (G_CTRL_REG.RDIR << 1) | (G_CTRL_REG.ENBL);
	#if debug
	  Serial.println("Writing CTRL Reg");
	#endif
	if (!SPI_VerifiedWrite(sendData)){
	  #if debug
	    Serial.println("Write to CTRL Register Failed");
	  #endif
	  ErrorFlag = true;
	};
}

void drv8711::WriteTORQUERegister()
{
    unsigned int sendData = 0;
    
    // Write TORQUE Register
    sendData = REGWRITE | (G_TORQUE_REG.Address << 12) | (G_TORQUE_REG.SIMPLTH << 8);
    sendData |= G_TORQUE_REG.TORQUE;
	#if debug 
	  Serial.println("Writing TORQUE Reg");
    #endif
	if (!SPI_VerifiedWrite(sendData)){
	  #if debug
	    Serial.println("Write to TORQUE Register Failed");
	  #endif
	  ErrorFlag = true;
	};
}	

void drv8711::WriteOFFRegister()
{
    unsigned int sendData = 0;
    
    // Write OFF Register
    sendData = REGWRITE | (G_OFF_REG.Address << 12) | (G_OFF_REG.PWMMODE << 8);
    sendData |= G_OFF_REG.TOFF;
	#if debug
	  Serial.println("Writing OFF Reg");
    #endif
	if (!SPI_VerifiedWrite(sendData)){
	  #if debug 
	    Serial.println("Write to OFF Register Failed");
	  #endif 
	  ErrorFlag = true;
	};
}

void drv8711::WriteBLANKRegister()
{
    unsigned int sendData = 0;
    
    // Write BLANK Register
    sendData = REGWRITE | (G_BLANK_REG.Address << 12) | (G_BLANK_REG.ABT << 8);
    sendData |= G_BLANK_REG.TBLANK;
	#if debug
	  Serial.println("Writing BLANK Reg");
    #endif
	if (!SPI_VerifiedWrite(sendData)){
	  #if debug
	    Serial.println("Write to BLANK Register Failed");
	  #endif
	  ErrorFlag = true;
	};
}

void drv8711::WriteDECAYRegister()
{
    unsigned int sendData = 0;
    
    // Write DECAY Register
    sendData = REGWRITE | (G_DECAY_REG.Address << 12) | (G_DECAY_REG.DECMOD << 8);
    sendData |= G_DECAY_REG.TDECAY;
	#if debug
	  Serial.println("Writing DECAY Reg");
    #endif
	if (!SPI_VerifiedWrite(sendData)){
	  #if debug
	    Serial.println("Write to DECAY Register Failed");
	  #endif
	  ErrorFlag = true;
	};
}

void drv8711::WriteSTALLRegister()
{
    unsigned int sendData = 0;
    
    // Write STALL Register
    sendData = REGWRITE | (G_STALL_REG.Address << 12) | (G_STALL_REG.VDIV << 10) | (G_STALL_REG.SDCNT << 8);
    sendData |= G_STALL_REG.SDTHR;
    #if debug
	  Serial.println("Writing STALL Reg");
	#endif
	if (!SPI_VerifiedWrite(sendData)){
	  #if debug
	   Serial.println("Write to STALL Register Failed");
	  #endif
	  ErrorFlag = true;
	};
}

void drv8711::WriteDRIVERegister()
{
    unsigned int sendData = 0;
    
    // Write DRIVE Register
    sendData = REGWRITE | (G_DRIVE_REG.Address << 12) | (G_DRIVE_REG.IDRIVEP << 10) | (G_DRIVE_REG.IDRIVEN << 8);
    sendData |= (G_DRIVE_REG.TDRIVEP << 6) | (G_DRIVE_REG.TDRIVEN << 4) | (G_DRIVE_REG.OCPDEG << 2) | (G_DRIVE_REG.OCPTH);
    #if debug
	 Serial.println("Writing DRIVE Reg");
	#endif 
	if (!SPI_VerifiedWrite(sendData)){
	  #if debug
	    Serial.println("Write to DRIVE Register Failed");
      #endif 	  
	  ErrorFlag = true;
	};
}

void drv8711::WriteSTATUSRegister()
{
    unsigned int sendData = 0;
    
    // Write STATUS Register
    sendData = REGWRITE | (G_STATUS_REG.Address << 12) ;
    sendData |= (G_STATUS_REG.STDLAT << 7) | (G_STATUS_REG.STD << 6) | (G_STATUS_REG.UVLO << 5) | (G_STATUS_REG.BPDF << 4) | (G_STATUS_REG.APDF << 3) | (G_STATUS_REG.BOCP << 2) | (G_STATUS_REG.AOCP << 1) | (G_STATUS_REG.OTS);
    #if debug 
	 Serial.println("Writing STATUS Reg");
	#endif
	SPI_ReadWrite(sendData);
}

void drv8711::WriteAllRegisters()
{
	WriteCTRLRegister();
	WriteTORQUERegister();	
	WriteOFFRegister();
	WriteBLANKRegister();
	WriteDECAYRegister();
	WriteSTALLRegister();
	WriteDRIVERegister();
	WriteSTATUSRegister();
}

bool drv8711::SPI_VerifiedWrite(unsigned int sendData)
{
    unsigned int readData = 0;
	int attempts = 0;
    const int maxtries = 10;	
	bool success = false;
	
	do {
	  attempts++;
	  // Write
      SPI_ReadWrite(sendData);
      // Readback
      readData = SPI_ReadWrite(REGREAD | sendData);
      // Compare
	  if ((readData << 4) == (sendData << 4)) {
		  success = true;
	    } else {
		  #if debug
		    Serial.println ("SPI Write Error, attempt:" + String(attempts));
		  #endif
		  delayMicroseconds(attempts); // delay before trying again
		}
	} while ( (success == false) && (attempts < maxtries) ) ;
	return success;
}


unsigned int drv8711::SPI_ReadWrite(unsigned int sendData)
{
	unsigned int readData = 0;
	
    digitalWrite(_pin, HIGH);
	readData = SPI.transfer16(sendData);
	digitalWrite(_pin, LOW);
	return readData;
}