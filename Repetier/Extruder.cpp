/*
    This file is part of the Repetier-Firmware for RF devices from Conrad Electronic SE.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "Repetier.h"
#include "pins_arduino.h"

#if EEPROM_MODE!=0
#include "Eeprom.h"
#endif // EEPROM_MODE!=0


uint8_t             manageMonitor = 255; ///< Temp. we want to monitor with our host. 1+NUM_EXTRUDER is heated bed
volatile uint8_t    execute100msPeriodical = 0;
volatile uint8_t    execute50msPeriodical = 0;
volatile uint8_t    execute16msPeriodical = 0;
volatile uint8_t    execute10msPeriodical = 0;

#if FEATURE_DITTO_PRINTING
uint8_t             Extruder::dittoMode = 0;
#endif // FEATURE_DITTO_PRINTING

#if ANALOG_INPUTS>0
const uint8         osAnalogInputChannels[] PROGMEM = ANALOG_INPUT_CHANNELS;
volatile uint8      osAnalogInputCounter[ANALOG_INPUTS] = {0};
volatile uint       osAnalogInputBuildup[ANALOG_INPUTS] = {0};
volatile uint8      osAnalogInputPos=0; // Current sampling position
volatile uint       osAnalogInputValues[ANALOG_INPUTS] = {0};
#endif // ANALOG_INPUTS>0

#ifdef USE_GENERIC_THERMISTORTABLE_1
short               temptable_generic1[GENERIC_THERM_NUM_ENTRIES][2];
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_2
short               temptable_generic2[GENERIC_THERM_NUM_ENTRIES][2];
#endif
#ifdef USE_GENERIC_THERMISTORTABLE_3
short               temptable_generic3[GENERIC_THERM_NUM_ENTRIES][2];
#endif

/** Makes updates to temperatures and heater state every call.
Is called every 100ms.
*/
static uint8_t extruderTempErrors = 0;
void Extruder::manageTemperatures()
{
#if FEATURE_MILLING_MODE
    if( Printer::operatingMode != OPERATING_MODE_PRINT )
    {
        // we do not check temperatures in case we are not in operating mode print
        return;
    }
#endif // FEATURE_MILLING_MODE

    uint8_t errorDetected = 0;

    for(uint8_t controller=0; controller<NUM_TEMPERATURE_LOOPS; controller++)
    {
        TemperatureController *act = tempController[controller];

        // Get Temperature
        act->updateCurrentTemperature();
        if(controller == autotuneIndex) continue;  // Ignore heater we are currently testing

        if(controller<NUM_EXTRUDER)
        {
#if NUM_EXTRUDER>=2 && EXT0_EXTRUDER_COOLER_PIN==EXT1_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN>=0
            if(controller==1 && autotuneIndex!=0 && autotuneIndex!=1)
                if(tempController[0]->currentTemperatureC<EXTRUDER_FAN_COOL_TEMP && tempController[0]->targetTemperatureC<EXTRUDER_FAN_COOL_TEMP &&
                        tempController[1]->currentTemperatureC<EXTRUDER_FAN_COOL_TEMP && tempController[1]->targetTemperatureC<EXTRUDER_FAN_COOL_TEMP)
                    extruder[0].coolerPWM = 0;
                else
                    extruder[0].coolerPWM = extruder[0].coolerSpeed;
            if(controller>1)
#endif // NUM_EXTRUDER>=2 && EXT0_EXTRUDER_COOLER_PIN==EXT1_EXTRUDER_COOLER_PIN && EXT0_EXTRUDER_COOLER_PIN>=0

                if(act->currentTemperatureC<EXTRUDER_FAN_COOL_TEMP && act->targetTemperatureC<EXTRUDER_FAN_COOL_TEMP)
                    extruder[controller].coolerPWM = 0;
                else
                    extruder[controller].coolerPWM = extruder[controller].coolerSpeed;
        }

        if(!act->isSensorDefect() && (act->currentTemperatureC < MIN_DEFECT_TEMPERATURE || act->currentTemperatureC > MAX_DEFECT_TEMPERATURE))   // no temp sensor or short in sensor, disable heater
        {
            extruderTempErrors++; //pro sensor. max +10+1+1+1.... bleibt also < 255
            errorDetected = 1;

            if(extruderTempErrors > 10)   // Ignore short temporary failures
            {
                act->setSensorDefect(true);
                if(!Printer::isAnyTempsensorDefect()){
                    Printer::setSomeTempsensorDefect(true);
                    Printer::flag2 |= PRINTER_FLAG2_GOT_TEMPS; //we are not waiting for first temp measurements anymore.

                    reportTempsensorAndHeaterErrors();
					Printer::stopPrint();

                    showError( (void*)ui_text_temperature_manager, (void*)ui_text_sensor_error );
                }
            }
        }
        if(Printer::isAnyTempsensorDefect()) continue;
		
        uint8_t on = act->currentTemperatureC >= act->targetTemperatureC ? LOW : HIGH;
        if(!on && act->isAlarm())
        {
            beep(50*(controller+1),3);
            act->setAlarm(false);  //reset alarm
        }

        
        millis_t time = HAL::timeInMilliseconds(); // compare time for decouple tests
        // Run test if heater and sensor are decoupled
        bool decoupleTestRequired = !errorDetected && act->decoupleTestPeriod > 0 && (time - act->lastDecoupleTest) > act->decoupleTestPeriod; // time enough for temperature change?
        if (decoupleTestRequired) { // Only test when powered
            if (act->isDecoupleFull()) { // Phase 1: Heating fully until target range is reached
                if (act->currentTemperatureC - act->lastDecoupleTemp < DECOUPLING_TEST_MIN_TEMP_RISE) { // failed test
                    extruderTempErrors++;
                    errorDetected = 1;

                    if (extruderTempErrors > 10) { // Ignore short temporary failures
                        act->setSensorDecoupled(true);
                        if (!Printer::isAnyTempsensorDefect()) {
                            Printer::setSomeTempsensorDefect(true);
							
							Com::printErrorFLN(Com::tHeaterDecoupledWarning);
							Com::printF(PSTR("Error:Temp. raised to slow. Rise = "), act->currentTemperatureC - act->lastDecoupleTemp);
							Com::printF(PSTR(" after "), (int32_t)(time - act->lastDecoupleTest));
							Com::printFLN(PSTR(" ms"));
							
							reportTempsensorAndHeaterErrors();
							Printer::stopPrint();
							
							showError( (void*)ui_text_temperature_manager, (void*)ui_text_heater_error );
                        }
                    }
                } else {
					//wir ziehen hier die zeit und die temperatur hoch: starten einen neuen test auf einer neuen stufe.
                    act->stopDecouple(); //without stop no clean start.
                    act->startFullDecoupleTest(time); //set new time and current temp
                }
            } else if (act->isDecoupleHold()) { // Phase 2: Holding temperature inside a target corridor
                if (fabs(act->currentTemperatureC - act->targetTemperatureC) > DECOUPLING_TEST_MAX_HOLD_VARIANCE) { // failed test
                    extruderTempErrors++;
                    errorDetected = 1;

                    if (extruderTempErrors > 10) { // Ignore short temporary failures
                        act->setSensorDecoupled(true);
                        if (!Printer::isAnyTempsensorDefect()) {
                            Printer::setSomeTempsensorDefect(true);
							
							Com::printErrorFLN(Com::tHeaterDecoupledWarning);
							Com::printF(PSTR("Error:Could not hold temperature "), act->lastDecoupleTemp);
							Com::printF(PSTR(" measured "), act->currentTemperatureC);
							Com::printFLN(Com::tC);
							
							reportTempsensorAndHeaterErrors();
							Printer::stopPrint();
							
							showError( (void*)ui_text_temperature_manager, (void*)ui_text_heater_error );
                        }
                    }
                } else {
					//this test ignores decoupleTestPeriod in configuration: "- act->decoupleTestPeriod" and runs every second. But not at first hold-start-time.
                    act->lastDecoupleTest = time - act->decoupleTestPeriod + 1000; // once running test every second
                }
            }
        }
        if (Printer::isAnyTempsensorDefect()) continue;
        
        act->tempArray[act->tempPointer++] = act->currentTemperatureC; //ist in jedem fall voll mit gültigen temperaturen, wenn der regelbereich erreicht wird.
        //act->tempPointer &= 3; // 3 = springe von 4 = 100b auf 0 zurück,    wenn 3. -> 1/300ms  -> 3.33 = reciproke     !!tempArray needs [4] ...
        //act->tempPointer &= 7; // 7 = springe von 8 = 1000b auf 0 zurück,   wenn 7. -> 1/700ms  -> 1.42 = reciproke     !!tempArray needs [8] ...
        act->tempPointer &= 15; // 15 = springe von 16 = 10000b zurück auf 0, wenn 15 -> 1/1500ms -> 0.666 = reciproke    !!tempArray needs [16] ...

        uint8_t output;
        float error = act->targetTemperatureC - act->currentTemperatureC;

        if ( act->targetTemperatureC < 20.0f || act->targetTemperatureC < MAX_ROOM_TEMPERATURE )
        {
			//targetTemperature is off
			//we never set heating on if the commanded temperature might be lower than max room temperature.
            output = 0; // off is off, even if damping term wants a heat peak!
            act->stopDecouple();
            act->tempIState = 0;
        }
        else if ( error > PID_CONTROL_RANGE )
        {
			//heating up to targetTemperature
            output = act->pidMax;
            act->startFullDecoupleTest(time);
            act->tempIState = 0;
        }
        else if ( error < -PID_CONTROL_RANGE )
        {
			//cooling down to targetTemperature
            output = 0;
            act->tempIState = 0;
        }
        else
        {
            act->startHoldDecoupleTest(time);
			
            float pidTerm = 0;
            float pgain = act->pidPGain * error;
            pidTerm += pgain;
            act->tempIState = constrain(act->tempIState + error, act->tempIStateLimitMin, act->tempIStateLimitMax);
            float igain = act->pidIGain * act->tempIState * 0.1;  // 0.1 = 10Hz
            pidTerm += igain;
            float dgain = act->pidDGain * (act->tempArray[act->tempPointer] - act->currentTemperatureC) * 0.666f; // raising dT/dt, 0.666 = reciproke of time interval (1500 ms) -> temparray greift weiter zurück als letzte messung.
            pidTerm += dgain;

#if SCALE_PID_TO_MAX==1
            pidTerm = (pidTerm*act->pidMax)*0.0039062;
#endif // SCALE_PID_TO_MAX==1

            output = constrain((int)pidTerm, 0, act->pidMax);
        }
        pwm_pos[act->pwmIndex] = output;

#ifdef EXTRUDER_MAX_TEMP
        if (act->currentTemperatureC>EXTRUDER_MAX_TEMP) // Force heater off if EXTRUDER_MAX_TEMP is exceeded
            pwm_pos[act->pwmIndex] = 0;
#endif // EXTRUDER_MAX_TEMP

#if LED_PIN>-1
        if (act == &Extruder::current->tempControl)
            WRITE(LED_PIN,on);
#endif // LED_PIN>-1
    } // for controller

    if (errorDetected == 0 && extruderTempErrors>0)
        extruderTempErrors--;

    if (Printer::isAnyTempsensorDefect())
    {
        for (uint8_t controller = 0; controller < NUM_TEMPERATURE_LOOPS; controller++)
        {
			tempController[controller]->setTargetTemperature(0,0);
        }
        Printer::debugLevel |= 8; // Go into dry mode
    } else {
        if (!errorDetected) Printer::flag2 |= PRINTER_FLAG2_GOT_TEMPS; //we are not waiting for first temp measurements anymore.
    }

} // manageTemperatures


void Extruder::initHeatedBed()
{
#if HAVE_HEATED_BED
    heatedBedController.updateTempControlVars();
#endif // HAVE_HEATED_BED
} // initHeatedBed


#if defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
void createGenericTable(short table[GENERIC_THERM_NUM_ENTRIES][2],short minTemp,short maxTemp,float beta,float r0,float t0,float r1,float r2)
{
    t0 += 273.15f;
    float rs, vs;
    if(r1==0)
    {
        rs = r2;
        vs = GENERIC_THERM_VREF;
    }
    else
    {
        vs =static_cast<float>((GENERIC_THERM_VREF * r1) / (r1 + r2));
        rs = (r2 * r1) / (r1 + r2);
    }
    float k = r0 * exp(-beta / t0);
    float delta = (maxTemp-minTemp) / (GENERIC_THERM_NUM_ENTRIES - 1.0f);
    for(uint8_t i = 0; i < GENERIC_THERM_NUM_ENTRIES; i++)
    {
        float t = maxTemp - i * delta;
        float r = exp(beta / (t + 272.65)) * k;
        float v = 4092 * r * vs / ((rs + r) * GENERIC_THERM_VREF);
        int adc = static_cast<int>(v);
        t *= 8;
        if(adc > 4092) adc = 4092;
        table[i][0] = (adc >> (ANALOG_REDUCE_BITS));
        table[i][1] = static_cast<int>(t);
#ifdef PRINT_GENERIC_TEMP_TABLE
        Com::printF(Com::tGenTemp,table[i][0]);
        Com::printFLN(Com::tComma,table[i][1]);
#endif // PRINT_GENERIC_TEMP_TABLE
    }
}
#endif


/** \brief Initalizes all extruder.
Updates the pin configuration needed for the extruder and activates extruder 0.
Starts a interrupt based analog input reader, which is used by simple thermistors
for temperature reading.
*/
void Extruder::initExtruder()
{
    uint8_t i;
    Extruder::current = &extruder[0];

#ifdef USE_GENERIC_THERMISTORTABLE_1
    createGenericTable(temptable_generic1,GENERIC_THERM1_MIN_TEMP,GENERIC_THERM1_MAX_TEMP,GENERIC_THERM1_BETA,GENERIC_THERM1_R0,GENERIC_THERM1_T0,GENERIC_THERM1_R1,GENERIC_THERM1_R2);
#endif // USE_GENERIC_THERMISTORTABLE_1

#ifdef USE_GENERIC_THERMISTORTABLE_2
    createGenericTable(temptable_generic2,GENERIC_THERM2_MIN_TEMP,GENERIC_THERM2_MAX_TEMP,GENERIC_THERM2_BETA,GENERIC_THERM2_R0,GENERIC_THERM2_T0,GENERIC_THERM2_R1,GENERIC_THERM2_R2);
#endif // USE_GENERIC_THERMISTORTABLE_2

#ifdef USE_GENERIC_THERMISTORTABLE_3
    createGenericTable(temptable_generic3,GENERIC_THERM3_MIN_TEMP,GENERIC_THERM3_MAX_TEMP,GENERIC_THERM3_BETA,GENERIC_THERM3_R0,GENERIC_THERM3_T0,GENERIC_THERM3_R1,GENERIC_THERM3_R2);
#endif // USE_GENERIC_THERMISTORTABLE_3

#if defined(EXT0_STEP_PIN) && EXT0_STEP_PIN>-1
    SET_OUTPUT(EXT0_DIR_PIN);
    SET_OUTPUT(EXT0_STEP_PIN);
#endif // defined(EXT0_STEP_PIN) && EXT0_STEP_PIN>-1

#if defined(EXT1_STEP_PIN) && EXT1_STEP_PIN>-1 && NUM_EXTRUDER>1
    SET_OUTPUT(EXT1_DIR_PIN);
    SET_OUTPUT(EXT1_STEP_PIN);
#endif // defined(EXT1_STEP_PIN) && EXT1_STEP_PIN>-1 && NUM_EXTRUDER>1

    for(i=0; i<NUM_EXTRUDER; ++i)
    {
        Extruder *act = &extruder[i];
        if(act->enablePin > -1)
        {
            HAL::pinMode(act->enablePin,OUTPUT);
            if(!act->enableOn) HAL::digitalWrite(act->enablePin,HIGH);
        }
        act->tempControl.lastTemperatureUpdate = HAL::timeInMilliseconds();
        act->tempControl.updateTempControlVars();
    }

#if HEATED_BED_HEATER_PIN>-1
    SET_OUTPUT(HEATED_BED_HEATER_PIN);
    WRITE(HEATED_BED_HEATER_PIN,HEATER_PINS_INVERTED);
    Extruder::initHeatedBed();
#endif // HEATED_BED_HEATER_PIN>-1

    HAL::analogStart();

} // initExtruder


void TemperatureController::updateTempControlVars()
{
    if(pidIGain!=0)   // prevent division by zero
    {
        tempIStateLimitMax = (float)pidDriveMax * PID_CONTROL_DRIVE_MAX_LIMIT_FACTOR / pidIGain;
        tempIStateLimitMin = (float)pidDriveMin * PID_CONTROL_DRIVE_MIN_LIMIT_FACTOR / pidIGain; //Bisher hatte der PID-Regler keinen negativen I-Anteil, weil die Limits nicht ins Negative dürfen. Jetzt schon. Es ist das Minus in der Config, das die Stabilität bringt.
    }
} // updateTempControlVars


/** \brief Select extruder ext_num.
This function changes and initalizes a new extruder. This is also called, after the eeprom values are changed.
*/
void Extruder::selectExtruderById(uint8_t extruderId)
{
#if FEATURE_MILLING_MODE
    if( Printer::operatingMode == OPERATING_MODE_MILL )
    {
        // in operating mode mill, the extruders are not used
        return;
    }
#endif // FEATURE_MILLING_MODE

    if(extruderId>=NUM_EXTRUDER)
        extruderId = 0;

#if NUM_EXTRUDER>1
    bool executeSelect = false;
    if(extruderId!=Extruder::current->id)
    {
        GCode::executeFString(Extruder::current->deselectCommands);
        executeSelect = true;
    }
#endif // NUM_EXTRUDER>1

#if STEPPER_ON_DELAY
    Extruder::current->enabled = 0;
#endif // STEPPER_ON_DELAY

    Extruder::current->extrudePosition = Printer::queuePositionLastSteps[E_AXIS];
    Extruder::current = &extruder[extruderId];

#ifdef SEPERATE_EXTRUDER_POSITIONS
    // Use seperate extruder positions only if beeing told. Slic3r e.g. creates a continuous extruder position increment
    Printer::queuePositionLastSteps[E_AXIS] = Extruder::current->extrudePosition;
#endif // SEPERATE_EXTRUDER_POSITIONS

    Printer::queuePositionTargetSteps[E_AXIS] = Printer::queuePositionLastSteps[E_AXIS];
    Printer::axisStepsPerMM[E_AXIS] = Extruder::current->stepsPerMM;
    Printer::invAxisStepsPerMM[E_AXIS] = 1.0f/Printer::axisStepsPerMM[E_AXIS];
    Printer::maxFeedrate[E_AXIS] = Extruder::current->maxFeedrate;
    Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = Extruder::current->maxAcceleration;
    Printer::maxTravelAccelerationStepsPerSquareSecond[E_AXIS] = Printer::maxPrintAccelerationStepsPerSquareSecond[E_AXIS] = Printer::maxAccelerationMMPerSquareSecond[E_AXIS] * Printer::axisStepsPerMM[E_AXIS];

    g_nManualSteps[E_AXIS] = uint32_t(Printer::axisStepsPerMM[E_AXIS] * DEFAULT_MANUAL_MM_E);
    g_nPauseSteps[E_AXIS]  = long    (Printer::axisStepsPerMM[E_AXIS] * DEFAULT_PAUSE_MM_E);

#if USE_ADVANCE
    Printer::maxExtruderSpeed = (uint8_t)floor(HAL::maxExtruderTimerFrequency() / (Extruder::current->maxFeedrate * Extruder::current->stepsPerMM));
    if(Printer::maxExtruderSpeed>15) Printer::maxExtruderSpeed = 15;
    float fmax=((float)HAL::maxExtruderTimerFrequency()/((float)Printer::maxExtruderSpeed*Printer::axisStepsPerMM[E_AXIS])); // Limit feedrate to interrupt speed
    if(fmax<Printer::maxFeedrate[E_AXIS]) Printer::maxFeedrate[E_AXIS] = fmax;
#endif // USE_ADVANCE

    Extruder::current->tempControl.updateTempControlVars();
    Printer::extruderOffset[X_AXIS] = -Extruder::current->xOffset*Printer::invAxisStepsPerMM[X_AXIS];
    Printer::extruderOffset[Y_AXIS] = -Extruder::current->yOffset*Printer::invAxisStepsPerMM[Y_AXIS];
    Printer::extruderOffset[Z_AXIS] = -Extruder::current->zOffset*Printer::invAxisStepsPerMM[Z_AXIS];

    //uncomment when inserting diameter for hotend x // Commands::changeFlowrateMultiply(static_cast<float>(Printer::extrudeMultiply)); // needed to adjust extrusionFactor to possibly different diameter

    if(Printer::areAxisHomed())
    {
        Printer::moveToReal(IGNORE_COORDINATE,IGNORE_COORDINATE,IGNORE_COORDINATE,IGNORE_COORDINATE,Printer::homingFeedrate[X_AXIS]);
    }
    Printer::updateCurrentPosition();
#if USE_ADVANCE
    HAL::resetExtruderDirection();
#endif // USE_ADVANCE

#if NUM_EXTRUDER>1
    if(executeSelect) // Run only when changing
        GCode::executeFString(Extruder::current->selectCommands);
#endif // NUM_EXTRUDER>1

} // selectExtruderById


void Extruder::setTemperatureForExtruder(float temperatureInCelsius,uint8_t extr,bool beep)
{
    if( extr >= NUM_EXTRUDER )
    {
        // do not set the temperature for an extruder which is not present - this attempt could heat up the extruder without any control and could significantly overheat the extruder
        if(temperatureInCelsius > 0) Com::printFLN( PSTR( "setTemperatureForExtruder(): cant set Temp for Extr. T" ), extr );
        return;
    }

    bool alloffs = true;
    for(uint8_t i=0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 0) alloffs = false;

#ifdef EXTRUDER_MAX_TEMP
    if(temperatureInCelsius > EXTRUDER_MAX_TEMP) temperatureInCelsius = EXTRUDER_MAX_TEMP;
#endif // EXTRUDER_MAX_TEMP

    if(temperatureInCelsius < 0) temperatureInCelsius=0;
    TemperatureController *tc = tempController[extr];
	if(tc->isSensorDefect() || tc->isSensorDecoupled()) temperatureInCelsius = 0;
    if(tc->sensorType == 0) temperatureInCelsius = 0;
    tc->setTargetTemperature(temperatureInCelsius,0);
    tc->updateTempControlVars();
    if(beep && temperatureInCelsius > 30)
        tc->setAlarm(true);
    if(temperatureInCelsius>=EXTRUDER_FAN_COOL_TEMP) extruder[extr].coolerPWM = extruder[extr].coolerSpeed;

    if( Printer::debugInfo() )
    {
        Com::printF(Com::tTargetExtr,extr,0);
        Com::printFLN(Com::tColon,temperatureInCelsius,0);
    }

#if FEATURE_CASE_FAN && !CASE_FAN_ALWAYS_ON
    if(Printer::ignoreFanOn){
        //ignore the case fan whenever there is another cooling solution available // Nibbels
        //the fan should still be connected within the rf2000 but might be avoided to suppress noise.
        //the ignore-flag has to be set at runtime to prevent unlearned persons to risk overheat having the wrong startcode.
        //enable and disable the fan with M3120 or M3121 or M3300 P3 S{1,0}
        Printer::prepareFanOff = 0;
        Printer::fanOffDelay = 0;
    }else{
        bool isheating = false;
        for(uint8_t i=0; i < NUM_EXTRUDER; i++) if(tempController[i]->targetTemperatureC > CASE_FAN_ON_TEMPERATURE) isheating = true;
        if( isheating )
        {
            // enable the case fan in case any extruder is turned on
            Printer::prepareFanOff = 0;
            WRITE(CASE_FAN_PIN, 1);
        }
        else
        {
            // disable the case fan in case the extruder is turned off
            if( Printer::fanOffDelay )
            {
                // we are going to disable the case fan after the delay
                Printer::prepareFanOff = HAL::timeInMilliseconds();
            }
            else
            {
                // we are going to disable the case fan now
                Printer::prepareFanOff = 0;
                WRITE(CASE_FAN_PIN, 0);
            }
        }
    }
#endif // FEATURE_CASE_FAN && !CASE_FAN_ALWAYS_ON

#if FEATURE_DITTO_PRINTING
    if(Extruder::dittoMode && extr == 0)
    {
        TemperatureController *tc2 = tempController[1];
        tc2->setTargetTemperature(temperatureInCelsius,0);
        tc2->updateTempControlVars();
        if(temperatureInCelsius >= EXTRUDER_FAN_COOL_TEMP) extruder[1].coolerPWM = extruder[1].coolerSpeed;
    }
#endif // FEATURE_DITTO_PRINTING

    bool alloff = true;
    for(uint8_t i=0; i < NUM_EXTRUDER; i++)
        if(tempController[i]->targetTemperatureC > 0) alloff = false;

#if EEPROM_MODE != 0
    if(alloff && !alloffs) // All heaters are now switched off?
    {
#if FEATURE_MILLING_MODE
        if( Printer::operatingMode == OPERATING_MODE_PRINT )
        {
#endif // FEATURE_MILLING_MODE
            EEPROM::updatePrinterUsage();
#if FEATURE_MILLING_MODE
        }
#endif // FEATURE_MILLING_MODE
    }
#endif // EEPROM_MODE != 0

    if(alloffs && !alloff) // heaters are turned on, start measuring printing time
    {
        Printer::msecondsPrinting = HAL::timeInMilliseconds();
        Printer::filamentPrinted = 0;  // new print, new counter
        Printer::flag2 &= ~PRINTER_FLAG2_RESET_FILAMENT_USAGE;
    }

} // setTemperatureForExtruder

void Extruder::setTemperatureForAllExtruders(float temperatureInCelsius, bool beep){
    for(uint8_t extrNr = 0; extrNr < NUM_EXTRUDER; extrNr++) Extruder::setTemperatureForExtruder(temperatureInCelsius, extrNr, beep);
}

void Extruder::setHeatedBedTemperature(float temperatureInCelsius,bool beep)
{
#if HAVE_HEATED_BED
    float   offset = 0.0;

    if(temperatureInCelsius>HEATED_BED_MAX_TEMP) temperatureInCelsius = HEATED_BED_MAX_TEMP;
    if(temperatureInCelsius<0) temperatureInCelsius = 0;
	if(heatedBedController.isSensorDefect() || heatedBedController.isSensorDecoupled()) temperatureInCelsius = 0;
							
    if(heatedBedController.targetTemperatureC==temperatureInCelsius) return; // don't flood log with messages if killed

#if FEATURE_HEAT_BED_TEMP_COMPENSATION
    offset = -getHeatBedTemperatureOffset( temperatureInCelsius );
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION

    heatedBedController.setTargetTemperature(temperatureInCelsius, offset);
    if(beep && temperatureInCelsius>30) heatedBedController.setAlarm(true);

    if( Printer::debugInfo() )
    {
        Com::printFLN(Com::tTargetBedColon,heatedBedController.targetTemperatureC,0);
    }
#endif // HAVE_HEATED_BED
} // setHeatedBedTemperature

float Extruder::getHeatedBedTemperature()
{
#if HAVE_HEATED_BED
    TemperatureController *c = tempController[NUM_TEMPERATURE_LOOPS-1];
    return c->currentTemperatureC;
#else
    return -1;
#endif // HAVE_HEATED_BED

} // getHeatedBedTemperature


void Extruder::disableCurrentExtruderMotor()
{
    if(Extruder::current->enablePin > -1)
        HAL::digitalWrite(Extruder::current->enablePin,!Extruder::current->enableOn);

#if FEATURE_DITTO_PRINTING
    if(Extruder::dittoMode)
    {
        if(extruder[1].enablePin > -1)
            HAL::digitalWrite(extruder[1].enablePin,!extruder[1].enableOn);
    }
#endif // FEATURE_DITTO_PRINTING

#if STEPPER_ON_DELAY
    Extruder::current->enabled = 0;
#endif // STEPPER_ON_DELAY

    cleanupEPositions();

} // disableCurrentExtruderMotor


void Extruder::disableAllExtruders()
{
#if FEATURE_MILLING_MODE
    if( Printer::operatingMode == OPERATING_MODE_PRINT )
    {
#endif // FEATURE_MILLING_MODE
        Extruder*   e;
        for(uint8_t i=0; i<NUM_EXTRUDER; i++ )
        {
            e = &extruder[i];

            if(e->enablePin > -1)
                HAL::digitalWrite(e->enablePin,!e->enableOn);

 #if STEPPER_ON_DELAY
            e->enabled = 0;
 #endif // STEPPER_ON_DELAY
        }
#if FEATURE_MILLING_MODE
    }
#endif // FEATURE_MILLING_MODE

    cleanupEPositions();
} // disableAllExtruders


#define NUMTEMPS_1 28
// Epcos B57560G0107F000
const short temptable_1[NUMTEMPS_1][2] PROGMEM =
{
    {0,4000},{92,2400},{105,2320},{121,2240},{140,2160},{162,2080},{189,2000},{222,1920},{261,1840},{308,1760},
    {365,1680},{434,1600},{519,1520},{621,1440},{744,1360},{891,1280},{1067,1200},{1272,1120},
    {1771,960},{2357,800},{2943,640},{3429,480},{3760,320},{3869,240},{3912,200},{3948,160},{4077,-160},{4094,-440}
};

// is 200k thermistor
#define NUMTEMPS_2 21
const short temptable_2[NUMTEMPS_2][2] PROGMEM =
{
    {1*4, 848*8},{54*4, 275*8}, {107*4, 228*8}, {160*4, 202*8},{213*4, 185*8}, {266*4, 171*8}, {319*4, 160*8}, {372*4, 150*8},
    {425*4, 141*8}, {478*4, 133*8},{531*4, 125*8},{584*4, 118*8},{637*4, 110*8},{690*4, 103*8},{743*4, 95*8},{796*4, 86*8},
    {849*4, 77*8},{902*4, 65*8},{955*4, 49*8},{1008*4, 17*8},{1020*4, 0*8} //safety
};

// mendel-parts thermistor (EPCOS G550) = NTC mit 100kOhm
#define NUMTEMPS_3 28

const short temptable_3[NUMTEMPS_3][2] PROGMEM =
{
    {1*4,864*8},{21*4,300*8},{25*4,290*8},{29*4,280*8},{33*4,270*8},{39*4,260*8},{46*4,250*8},{54*4,240*8},{64*4,230*8},{75*4,220*8},
    {90*4,210*8},{107*4,200*8},{128*4,190*8},{154*4,180*8},{184*4,170*8},{221*4,160*8},{265*4,150*8},{316*4,140*8},{375*4,130*8},
    {441*4,120*8},{513*4,110*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}
};


// is 10k thermistor
#define NUMTEMPS_4 20
const short temptable_4[NUMTEMPS_4][2] PROGMEM =
{
    {1*4, 430*8},{54*4, 137*8},{107*4, 107*8},{160*4, 91*8},{213*4, 80*8},{266*4, 71*8},{319*4, 64*8},{372*4, 57*8},{425*4, 51*8},
    {478*4, 46*8},{531*4, 41*8},{584*4, 35*8},{637*4, 30*8},{690*4, 25*8},{743*4, 20*8},{796*4, 14*8},{849*4, 7*8},{902*4, 0*8},
    {955*4, -11*8},{1008*4, -35*8}
};

// ATC Semitec 104GT-2 / E3D Hotend Thermistor
#define NUMTEMPS_8 34
const short temptable_8[NUMTEMPS_8][2] PROGMEM =
{
    {0,8000},{69,2400},{79,2320},{92,2240},{107,2160},{125,2080},{146,2000},{172,1920},{204,1840},{244,1760},{291,1680},{350,1600},
    {422,1520},{511,1440},{621,1360},{755,1280},{918,1200},{1114,1120},{1344,1040},{1608,960},{1902,880},{2216,800},{2539,720},
    {2851,640},{3137,560},{3385,480},{3588,400},{3746,320},{3863,240},{3945,160},{4002,80},{4038,0},{4061,-80},{4075,-160}
};

// 100k Honeywell 135-104LAG-J01
#define NUMTEMPS_9 67
const short temptable_9[NUMTEMPS_9][2] PROGMEM =
{
    {1*4, 941*8},{19*4, 362*8},{37*4, 299*8}, //top rating 300C
    {55*4, 266*8},{73*4, 245*8},{91*4, 229*8},{109*4, 216*8},{127*4, 206*8},{145*4, 197*8},{163*4, 190*8},{181*4, 183*8},{199*4, 177*8},
    {217*4, 171*8},{235*4, 166*8},{253*4, 162*8},{271*4, 157*8},{289*4, 153*8},{307*4, 149*8},{325*4, 146*8},{343*4, 142*8},{361*4, 139*8},
    {379*4, 135*8},{397*4, 132*8},{415*4, 129*8},{433*4, 126*8},{451*4, 123*8},{469*4, 121*8},{487*4, 118*8},{505*4, 115*8},{523*4, 112*8},
    {541*4, 110*8},{559*4, 107*8},{577*4, 105*8},{595*4, 102*8},{613*4, 99*8},{631*4, 97*8},{649*4, 94*8},{667*4, 92*8},{685*4, 89*8},
    {703*4, 86*8},{721*4, 84*8},{739*4, 81*8},{757*4, 78*8},{775*4, 75*8},{793*4, 72*8},{811*4, 69*8},{829*4, 66*8},{847*4, 62*8},
    {865*4, 59*8},{883*4, 55*8},{901*4, 51*8},{919*4, 46*8},{937*4, 41*8},
    {955*4, 35*8},{973*4, 27*8},{991*4, 17*8},{1009*4, 1*8},{1023*4, 0}  //to allow internal 0 degrees C
};

// 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
#define NUMTEMPS_10 20
const short temptable_10[NUMTEMPS_10][2] PROGMEM =
{
    {1*4, 704*8},{54*4, 216*8},{107*4, 175*8},{160*4, 152*8},{213*4, 137*8},{266*4, 125*8},{319*4, 115*8},{372*4, 106*8},{425*4, 99*8},
    {478*4, 91*8},{531*4, 85*8},{584*4, 78*8},{637*4, 71*8},{690*4, 65*8},{743*4, 58*8},{796*4, 50*8},{849*4, 42*8},{902*4, 31*8},
    {955*4, 17*8},{1008*4, 0}
};

// 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
#define NUMTEMPS_11 31
const short temptable_11[NUMTEMPS_11][2] PROGMEM =
{
    {1*4, 936*8},{36*4, 300*8},{71*4, 246*8},{106*4, 218*8},{141*4, 199*8},{176*4, 185*8},{211*4, 173*8},{246*4, 163*8},{281*4, 155*8},
    {316*4, 147*8},{351*4, 140*8},{386*4, 134*8},{421*4, 128*8},{456*4, 122*8},{491*4, 117*8},{526*4, 112*8},{561*4, 107*8},{596*4, 102*8},
    {631*4, 97*8},{666*4, 92*8},{701*4, 87*8},{736*4, 81*8},{771*4, 76*8},{806*4, 70*8},{841*4, 63*8},{876*4, 56*8},{911*4, 48*8},
    {946*4, 38*8},{981*4, 23*8},{1005*4, 5*8},{1016*4, 0}
};

// 100k RS thermistor 198-961 (4.7k pullup)
#define NUMTEMPS_12 31
const short temptable_12[NUMTEMPS_12][2] PROGMEM =
{
    {1*4, 929*8},{36*4, 299*8},{71*4, 246*8},{106*4, 217*8},{141*4, 198*8},{176*4, 184*8},{211*4, 173*8},{246*4, 163*8},{281*4, 154*8},{316*4, 147*8},
    {351*4, 140*8},{386*4, 134*8},{421*4, 128*8},{456*4, 122*8},{491*4, 117*8},{526*4, 112*8},{561*4, 107*8},{596*4, 102*8},{631*4, 97*8},{666*4, 91*8},
    {701*4, 86*8},{736*4, 81*8},{771*4, 76*8},{806*4, 70*8},{841*4, 63*8},{876*4, 56*8},{911*4, 48*8},{946*4, 38*8},{981*4, 23*8},{1005*4, 5*8},{1016*4, 0*8}
};


// PT100 E3D
#define NUMTEMPS_13 19
const short temptable_13[NUMTEMPS_13][2] PROGMEM =
{
    {0,0},{908,8},{942,10*8},{982,20*8},{1015,8*30},{1048,8*40},{1080,8*50},{1113,8*60},{1146,8*70},{1178,8*80},{1211,8*90},{1276,8*110},{1318,8*120}
    ,{1670,8*230},{2455,8*500},{3445,8*900},{3666,8*1000},{3871,8*1100},{4095,8*2000}
};

/*
//TODO: Tabelle startet erst bei 92°C ... Kleine Rundungszacken am Anfang, weil alles *4.
#define NUMTEMPS_13 61 // NTC 3950 100k thermistor - Conrad V3
const short temptable_13[NUMTEMPS_13][2] PROGMEM =
{
  {23*4, 300*8},{25*4, 295*8},{27*4, 290*8},{28*4, 285*8},{31*4, 280*8},{33*4, 275*8},{35*4, 270*8},{38*4, 265*8},{41*4, 260*8},{44*4, 255*8},
  {48*4, 250*8},{52*4, 245*8},{56*4, 240*8},{61*4, 235*8},{66*4, 230*8},{71*4, 225*8},{78*4, 220*8},{84*4, 215*8},{92*4, 210*8},{100*4, 205*8},
  {109*4, 200*8},{120*4, 195*8},{131*4, 190*8},{143*4, 185*8},{156*4, 180*8},{171*4, 175*8},{187*4, 170*8},{205*4, 165*8},{224*4, 160*8},{245*4, 155*8},
  {268*4, 150*8},{293*4, 145*8},{320*4, 140*8},{348*4, 135*8},{379*4, 130*8},{411*4, 125*8},{445*4, 120*8},{480*4, 115*8},{516*4, 110*8},{553*4, 105*8},
  {591*4, 100*8},{628*4, 95*8},{665*4, 90*8},{702*4, 85*8},{737*4, 80*8},{770*4, 75*8},{801*4, 70*8},{830*4, 65*8},{857*4, 60*8},{881*4, 55*8},
  {903*4, 50*8},{922*4, 45*8},{939*4, 40*8},{954*4, 35*8},{966*4, 30*8},{977*4, 25*8},{985*4, 20*8},{993*4, 15*8},{999*4, 10*8},{1004*4, 5*8},
  {1008*4, 0*8}
};
*/
// Thermistor NTC 3950 100k Ohm (other source)
#define NUMTEMPS_14 103
const short temptable_14[NUMTEMPS_14][2] PROGMEM = {
    {1*4,938*8},{11*4,423*8},{21*4,351*8},{31*4,314*8},{41*4,290*8},{51*4,272*8},{61*4,258*8},{71*4,247*8},\
{81*4,237*8},{91*4,229*8},{101*4,221*8},{111*4,215*8},{121*4,209*8},{131*4,204*8},{141*4,199*8},{151*4,195*8},\
{161*4,190*8},{171*4,187*8},{181*4,183*8},{191*4,179*8},{201*4,176*8},{211*4,173*8},{221*4,170*8},{231*4,167*8},\
{241*4,165*8},{251*4,162*8},{261*4,160*8},{271*4,157*8},{281*4,155*8},{291*4,153*8},{301*4,150*8},{311*4,148*8},\
{321*4,146*8},{331*4,144*8},{341*4,142*8},{351*4,140*8},{361*4,139*8},{371*4,137*8},{381*4,135*8},{391*4,133*8},\
{401*4,131*8},{411*4,130*8},{421*4,128*8},{431*4,126*8},{441*4,125*8},{451*4,123*8},{461*4,122*8},{471*4,120*8},\
{481*4,119*8},{491*4,117*8},{501*4,116*8},{511*4,114*8},{521*4,113*8},{531*4,111*8},{541*4,110*8},{551*4,108*8},\
{561*4,107*8},{571*4,105*8},{581*4,104*8},{591*4,102*8},{601*4,101*8},{611*4,100*8},{621*4,98*8},{631*4,97*8},\
{641*4,95*8},{651*4,94*8},{661*4,92*8},{671*4,91*8},{681*4,90*8},{691*4,88*8},{701*4,87*8},{711*4,85*8},{721*4,84*8},\
{731*4,82*8},{741*4,81*8},{751*4,79*8},{761*4,77*8},{771*4,76*8},{781*4,74*8},{791*4,72*8},{801*4,71*8},{811*4,69*8},\
{821*4,67*8},{831*4,65*8},{841*4,63*8},{851*4,62*8},{861*4,60*8},{871*4,57*8},{881*4,55*8},{891*4,53*8},{901*4,51*8},\
{911*4,48*8},{921*4,45*8},{931*4,42*8},{941*4,39*8},{951*4,36*8},{961*4,32*8},{971*4,28*8},{981*4,23*8},{991*4,17*8},\
{1001*4,9*8},{1011*4,-1*8},{1021*4,-26*8}
};

// E3D PT100 Board (direct AD voltage in / PTC type)
#define NUMTEMPS_15 19
const short temptable_15[NUMTEMPS_15][2] PROGMEM = {
    {0, 0}, {908, 8}, {942, 10 * 8}, {982, 20 * 8}, {1015, 8 * 30}, {1048, 8 * 40}, {1080, 8 * 50}, {1113, 8 * 60}, {1146, 8 * 70}, {1178, 8 * 80}, {1211, 8 * 90}, {1276, 8 * 110}, {1318, 8 * 120}
    , {1670, 8 * 230}, {2455, 8 * 500}, {3445, 8 * 900}, {3666, 8 * 1000}, {3871, 8 * 1100}, {4095, 8 * 2000}
};

#if NUM_TEMPS_USERTHERMISTOR0>0
const short temptable_5[NUM_TEMPS_USERTHERMISTOR0][2] PROGMEM = USER_THERMISTORTABLE0 ;
#endif // NUM_TEMPS_USERTHERMISTOR0>0

#if NUM_TEMPS_USERTHERMISTOR1>0
const short temptable_6[NUM_TEMPS_USERTHERMISTOR1][2] PROGMEM = USER_THERMISTORTABLE1 ;
#endif // NUM_TEMPS_USERTHERMISTOR1>0

#if NUM_TEMPS_USERTHERMISTOR2>0
const short temptable_7[NUM_TEMPS_USERTHERMISTOR2][2] PROGMEM = USER_THERMISTORTABLE2 ;
#endif // NUM_TEMPS_USERTHERMISTOR2>0

const short * const temptables[15] PROGMEM = {(short int *)&temptable_1[0][0],(short int *)&temptable_2[0][0],(short int *)&temptable_3[0][0],(short int *)&temptable_4[0][0]
#if NUM_TEMPS_USERTHERMISTOR0>0
        ,(short int *)&temptable_5[0][0]
#else
        ,0
#endif // NUM_TEMPS_USERTHERMISTOR0>0

#if NUM_TEMPS_USERTHERMISTOR1>0
        ,(short int *)&temptable_6[0][0]
#else
        ,0
#endif // NUM_TEMPS_USERTHERMISTOR1>0

#if NUM_TEMPS_USERTHERMISTOR2>0
        ,(short int *)&temptable_7[0][0]
#else
        ,0
#endif // NUM_TEMPS_USERTHERMISTOR2>0

        ,(short int *)&temptable_8[0][0]
        ,(short int *)&temptable_9[0][0]
        ,(short int *)&temptable_10[0][0]
        ,(short int *)&temptable_11[0][0]
        ,(short int *)&temptable_12[0][0]
        ,(short int *)&temptable_13[0][0]
        ,(short int *)&temptable_14[0][0]
        ,(short int *)&temptable_15[0][0]
    };
const uint8_t temptables_num[15] PROGMEM = {
                                            NUMTEMPS_1,
                                            NUMTEMPS_2,
                                            NUMTEMPS_3,
                                            NUMTEMPS_4,
                                            NUM_TEMPS_USERTHERMISTOR0,
                                            NUM_TEMPS_USERTHERMISTOR1,
                                            NUM_TEMPS_USERTHERMISTOR2,
                                            NUMTEMPS_8,
                                            NUMTEMPS_9,
                                            NUMTEMPS_10,
                                            NUMTEMPS_11,
                                            NUMTEMPS_12,
                                            NUMTEMPS_13,
                                            NUMTEMPS_14,
                                            NUMTEMPS_15
                                           };


void TemperatureController::updateCurrentTemperature()
{
    uint8_t type = sensorType;
    // get raw temperature
    switch(type)
    {
        case 0:
            currentTemperature = 25;
            break;
#if ANALOG_INPUTS>0
        case 1: // Epcos B57560G0107F000
        case 2: // is 200k thermistor
        case 3: // V2 Sensor Conrad Renkforce / mendel-parts thermistor (EPCOS G550) = NTC mit 100kOhm
        case 4: // is 10k thermistor
        case 5: // user thermistor 0
        case 6: // user thermistor 1
        case 7: // user thermistor 2
        case 8: // E3D Thermistor
        case 9: // 100k Honeywell 135-104LAG-J01
        case 10: // 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
        case 11: // 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
        case 12: // 100k RS thermistor 198-961 (4.7k pullup)
        case 13: // NTC 3950 100k thermistor - Conrad V3
        case 14: // Thermistor NTC 3950 100k Ohm
        case 97: // Define Raw Thermistor and Restistor-Settings within configuration.h see USE_GENERIC_THERMISTORTABLE_1 and GENERIC_THERM_NUM_ENTRIES
        case 98: // Define Raw Thermistor and Restistor-Settings within configuration.h see USE_GENERIC_THERMISTORTABLE_2 and GENERIC_THERM_NUM_ENTRIES
        case 99: // Define Raw Thermistor and Restistor-Settings within configuration.h see USE_GENERIC_THERMISTORTABLE_3 and GENERIC_THERM_NUM_ENTRIES
        {
            currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - (osAnalogInputValues[sensorPin] >> (ANALOG_REDUCE_BITS)); // Convert to 10 bit result
            break;
        }
        case 50: // User defined PTC table
        case 51:
        case 52:
        case 53: // E3D PT100 Board
        case 60: // HEATER_USES_AD8495 (Delivers 5mV/degC)
        case 100: // AD595
        {
            currentTemperature = (osAnalogInputValues[sensorPin] >> (ANALOG_REDUCE_BITS));
            break;
        }
#endif // ANALOG_INPUTS>0

        default:
        {
            currentTemperature = 4095; // unknown method, return high value to switch heater off for safety
            break;
        }
    }

    switch(type)
    {
        case 0:
            currentTemperatureC = 25;
            break;
        //NTC
        case 1:
        case 2:
        case 3:
        case 4:
        case 5: //user thermistor 0
        case 6: //user thermistor 1
        case 7: //user thermistor 2
        case 8: //E3D Thermistor
        case 9:
        case 10:
        case 11:
        case 12:
        case 13: // Conrad V3
        case 14: // Thermistor NTC 3950 100k Ohm
        //PTC
        case 50: // User defined PTC thermistor 0
        case 51: // user defined PTC thermistor 1
        case 52: // user defined PTC thermistor 2
        case 53: // E3D PT100 Board
        {
            //###########################################TYPE DISTINCT
            #define NTC_thermistor true
            #define PTC_thermistor false
            bool thermistortype = NTC_thermistor;

            if(type >= 50 && type <= 52){
                type -= 45; //50 -> 5 .. 52 -> 7 :: das heißt verwende user thermistor position 5..7, aber als PTC statt NTC
                thermistortype = PTC_thermistor;
            }
            else if(type == 53){
                type -= 38; //53 -> 15 PT100 E3D
                thermistortype = PTC_thermistor;
            }
            type--; //num to index

            uint8_t num = pgm_read_byte(&temptables_num[type]) << 1;
            uint8_t i = 2;
            const int16_t *temptable = (const int16_t *)pgm_read_word(&temptables[type]);
            int16_t oldraw  = pgm_read_word(&temptable[0]);
            int16_t oldtemp = pgm_read_word(&temptable[1]);
            int16_t newtemp = 0;

            //NTC: Suche Temperatur andersrum, weil Steigung der Umrechnungskurve invertiert ist.
            if(thermistortype == NTC_thermistor)
                currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - currentTemperature;

            while(i<num)
            {
                int16_t newraw = pgm_read_word(&temptable[i++]);
                       newtemp = pgm_read_word(&temptable[i++]);
                if (newraw > currentTemperature)
                {
                    currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature - oldraw) * (float)(newtemp - oldtemp) / (newraw - oldraw));
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
                    currentTemperatureC += getHeatBedTemperatureOffset( currentTemperatureC );
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION

                    return;
                }
                oldtemp = newtemp;
                oldraw = newraw;
            }
            // Overflow: Set to last value in the table
            currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
            currentTemperatureC += getHeatBedTemperatureOffset( currentTemperatureC );
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION

            break;
        }
        case 60: // AD8495 (Delivers 5mV/degC vs the AD595's 10mV)
        {
            currentTemperatureC = ((float)currentTemperature * 1000.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
            currentTemperatureC += getHeatBedTemperatureOffset( currentTemperatureC );
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION

            break;
        }
        case 100: // AD595
        {
            currentTemperatureC = ((float)currentTemperature * 500.0f / (1024 << (2 - ANALOG_REDUCE_BITS)));
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
            currentTemperatureC += getHeatBedTemperatureOffset( currentTemperatureC );
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION

            break;
        }

#if defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
        case 97:
        case 98:
        case 99:
        {
            uint8_t i =  2;
            const int16_t *temptable;

#ifdef USE_GENERIC_THERMISTORTABLE_1
            if(type == 97)
                temptable = (const int16_t *)temptable_generic1;
#endif // USE_GENERIC_THERMISTORTABLE_1

#ifdef USE_GENERIC_THERMISTORTABLE_2
            if(type == 98)
                temptable = (const int16_t *)temptable_generic2;
#endif // USE_GENERIC_THERMISTORTABLE_2

#ifdef USE_GENERIC_THERMISTORTABLE_3
            if(type == 99)
                temptable = (const int16_t *)temptable_generic3;
#endif // USE_GENERIC_THERMISTORTABLE_3

            int16_t oldraw = temptable[0];
            int16_t oldtemp = temptable[1];
            int16_t newtemp;

            currentTemperature = (1023 << (2 - ANALOG_REDUCE_BITS)) - currentTemperature; //NTC -> switch to search from other side

            while(i < GENERIC_THERM_NUM_ENTRIES*2)
            {
                int16_t newraw = temptable[i++];
                       newtemp = temptable[i++];
                if (newraw > currentTemperature)
                {
                    currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature - oldraw) * (float)(newtemp - oldtemp) / (newraw - oldraw));
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
                    currentTemperatureC += getHeatBedTemperatureOffset( currentTemperatureC );
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION

                    return;
                }
                oldtemp = newtemp;
                oldraw = newraw;
            }
            // Overflow: Set to last value in the table
            currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
            currentTemperatureC += getHeatBedTemperatureOffset( currentTemperatureC );
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION

            break;
        }
#endif // defined(USE_GENERIC_THERMISTORTABLE_1) || defined(USE_GENERIC_THERMISTORTABLE_2) || defined(USE_GENERIC_THERMISTORTABLE_3)
    }

} // updateCurrentTemperature


void TemperatureController::setTargetTemperature(float target, float offset)
{
	if(target <= 0.0f || target < targetTemperatureC) pwm_pos[pwmIndex] = 0;
    targetTemperatureC = target;
    stopDecouple();
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
    offsetC =  offset; //this->offsetC
#else
    (void)offset;
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION
	
	//do not set back temperatures to "not idle temperatures" after pause, if they have been changed manually within the pause.
	paused = 0;
} // setTargetTemperature

uint8_t autotuneIndex = 255;

void TemperatureController::waitForTargetTemperature(uint8_t plus_temp_tolerance) {
    if(Printer::debugDryrun()) return;
    bool dirRising = targetTemperatureC > currentTemperatureC;
    if(dirRising){
        UI_STATUS_UPD( UI_TEXT_HEATING_UP );
    }else{
        UI_STATUS_UPD( UI_TEXT_COOLING_DOWN );
    }
    g_uStartOfIdle = 0; //start waitForTargetTemperature
    while(true) {
        Commands::printTemperatures();
        Commands::checkForPeriodicalActions( WaitHeater );
        if( fabs(targetTemperatureC - currentTemperatureC) <= TEMP_TOLERANCE + plus_temp_tolerance ) break;
        if( !dirRising && currentTemperatureC < MAX_ROOM_TEMPERATURE ) break;
    }
    g_uStartOfIdle = HAL::timeInMilliseconds(); //end waitForTargetTemperature
}

void TemperatureController::autotunePID(float temp, uint8_t controllerId, int maxCycles, bool storeValues, int method)
{
    g_uStartOfIdle = 0; // start autotunePID
    UI_STATUS_UPD(UI_TEXT_PID);
    float currentTemp;
    int cycles=0;
    bool heating = true;

    uint32_t temp_millis = HAL::timeInMilliseconds();
    uint32_t t1=temp_millis;
    uint32_t t2=temp_millis;
    int32_t t_high = 0;
    int32_t t_low = 0;

    int32_t bias=pidMax>>1;
    int32_t d = pidMax>>1;
    float Ku, Tu;
    float Kp = 0, Ki = 0, Kd = 0;
    float maxTemp=20, minTemp=20;
    if(maxCycles < 5){
        maxCycles = 5;
    }
    if(maxCycles > 20){
        maxCycles = 20;
    }
    Com::printInfoFLN(Com::tPIDAutotuneStart);
    Com::printF( PSTR("Ruleset: "), (int)method );
    switch(method){
        case 4: //Tyreus-Lyben
        {
            Com::printFLN(Com::tAPIDTyreusLyben);
            break;
        }
        case 3: //PID no overshoot
        {
            Com::printFLN(Com::tAPIDNone);
            break;
        }
        case 2: //PID some overshoot
        {
            Com::printFLN(Com::tAPIDSome);
            break;
        }
        case 1: //PID Pessen Integral Rule
        {
            Com::printFLN(Com::tAPIDPessen);
            break;
        }
        default: //PID classic Ziegler-Nichols
        {
            Com::printFLN(Com::tAPIDClassic);
            method = 0; //filter not available methods.
        }
    }

    autotuneIndex = controllerId;
    pwm_pos[pwmIndex] = pidMax;
    if(controllerId<NUM_EXTRUDER)
    {
        extruder[controllerId].coolerPWM = extruder[controllerId].coolerSpeed;
        extruder[0].coolerPWM = extruder[0].coolerSpeed;
    }

    for(;;)
    {
        Commands::checkForPeriodicalActions( WaitHeater ); // update heaters etc. https://github.com/repetier/Repetier-Firmware/commit/241c550ac004023842d6886c6e0db15a1f6b56d7
        updateCurrentTemperature();
        currentTemp = currentTemperatureC;

        millis_t time = HAL::timeInMilliseconds();
        maxTemp=RMath::max(maxTemp,currentTemp);
        minTemp=RMath::min(minTemp,currentTemp);
        if(heating == true && currentTemp > temp)   // switch heating -> off
        {
            if(time - t2 > (controllerId<NUM_EXTRUDER ? 2500 : 1500))
            {
                heating = false;
                pwm_pos[pwmIndex] = (bias - d);
                t1 = time;
                t_high = t1 - t2;
                maxTemp=temp;
            }
        }
        if(heating == false && currentTemp < temp)
        {
            if(time - t1 > (controllerId<NUM_EXTRUDER ? 5000 : 3000))
            {
                heating = true;
                t2 = time;
                t_low = t2 - t1; // half wave length
                if(cycles > 0)
                {
                    bias += (d*(t_high - t_low))/(t_low + t_high);
                    bias = constrain(bias, 20 ,pidMax - 20);
                    if(bias > pidMax/2) d = pidMax - 1 - bias;
                    else d = bias;

                    Com::printF(Com::tAPIDBias,bias);
                    Com::printF(Com::tAPIDD,d);
                    Com::printF(Com::tAPIDMin,minTemp);
                    Com::printFLN(Com::tAPIDMax,maxTemp);
                    if(cycles > 2)
                    {
                        // Parameter according Ziegler¡§CNichols method: http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
                        Ku = (4.0 * d)/(3.14159 * (maxTemp - minTemp));
                        Tu = static_cast<float>(t_low + t_high) / 1000.0;
                        Com::printF(Com::tAPIDKu,Ku);
                        Com::printFLN(Com::tAPIDTu,Tu);
/**
https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
KP = KP
KI = KP / Ti
KD = KP * Td
wegen Formel u(t) = KP*( rest )

Ti = Tu / Maßzahl lt. Tabelle
Td = Tu / Maßzahl lt. Tabelle
KP = Ku * Maßzahl lt. Tabelle

see also: http://www.mstarlabs.com/control/znrule.html
*/
                        switch(method){
                            case 4: { //PID Tyreus-Lyben ("lazy--" -> Heated Bed!)
                               Kp = 0.4545f*Ku;      //1/2.2 KRkrit
                               Ki = Kp/Tu/2.2f;        //2.2 Tkrit
                               Kd = Kp*Tu/6.3f;      //1/6.3 Tkrit[/code]
                               Com::printFLN(Com::tAPIDTyreusLyben);
                               break;
                            }
                            case 3: { //PID no overshoot ("lazy-" -> Heated Bed!)
                               Kp = 0.2f*Ku;          //0.2 KRkrit
                               Ki = 2.0f*Kp/Tu;       //0.5 Tkrit
                               Kd = Kp*Tu/3.0f;       //0.333 Tkrit
                               Com::printFLN(Com::tAPIDNone);
                               break;
                            }
                            case 2: { //PID some overshoot
                               Kp = 0.33f*Ku;         //0.33 KRkrit
                               Ki = 2.0f*Kp/Tu;       //0.5 Tkrit
                               Kd = Kp*Tu/3.0f;       //0.333 Tkrit
                               Com::printFLN(Com::tAPIDSome);
                               break;
                            }
                            case 1: { //PID Pessen Integral Rule ("dynamic++" -> fast Hotend!)
                               Kp = 0.7f*Ku;          //0.7 KRkrit
                               Ki = 2.5f*Kp/Tu;       //0.4 Tkrit
                               Kd = Kp*Tu*3.0f/20.0f; //0.15 Tkrit
                               Com::printFLN(Com::tAPIDPessen);
                               break;
                            }
                            default: { //PID classic Ziegler-Nichols  ("dynamic+" -> Hotend!)
                               Kp = 0.6f*Ku;          //0.6 KRkrit
                               Ki = 2.0f*Kp/Tu;       //0.5 Tkrit
                               Kd = Kp*Tu/8.0f;       //0.125 Tkrit
                               Com::printFLN(Com::tAPIDClassic);
                            }
                        }
                        Com::printFLN(Com::tAPIDKp,Kp);
                        Com::printFLN(Com::tAPIDKi,Ki);
                        Com::printFLN(Com::tAPIDKd,Kd);
                    }
                }
                pwm_pos[pwmIndex] = (bias + d);
                cycles++;
                minTemp=temp;
            }
        }
        if(currentTemp > (temp + 40))
        {
            Com::printErrorFLN(Com::tAPIDFailedHigh);
            showError( (void*)ui_text_autodetect_pid, (void*)ui_text_temperature_wrong );
            autotuneIndex = 255;
            break;
        }

        Commands::printTemperatures();

        if(((time - t1) + (time - t2)) > (10L*60L*1000L*2L))   // 20 Minutes
        {
            Com::printErrorFLN(Com::tAPIDFailedTimeout);
            showError( (void*)ui_text_autodetect_pid, (void*)ui_text_timeout );
            autotuneIndex = 255;
            break;
        }
        if(cycles > maxCycles)
        {
            Com::printInfoFLN(Com::tAPIDFinished);
            UI_STATUS_UPD( UI_TEXT_AUTODETECT_PID_DONE );
            autotuneIndex = 255;
            if(storeValues)
            {
                pidPGain = Kp;
                pidIGain = Ki;
                pidDGain = Kd;
                EEPROM::storeDataIntoEEPROM();
            }
            g_uStartOfIdle = HAL::timeInMilliseconds()+30000; //end autotunePID cycles
            return;
        }
    }
    g_uStartOfIdle = HAL::timeInMilliseconds(); //end autotunePID with error
} // autotunePID

void reportTempsensorAndHeaterErrors()
{
	//Write full status list to console output
	for(uint8_t i=0; i<NUM_TEMPERATURE_LOOPS; i++)
	{
		if(i==NUM_EXTRUDER) Com::printF(Com::tHeatedBed);
		else Com::printF(Com::tExtruderSpace,i);

		if (tempController[i]->isSensorDefect()) {
			Com::printF(Com::tTempSensorDefect);
		}
		if (tempController[i]->isSensorDecoupled()) {
			Com::printF(Com::tTempHeaterDefect);
		}
		if (!tempController[i]->isSensorDecoupled() && !tempController[i]->isSensorDefect()) {
			Com::printF(Com::tTempSensorWorking);
		}
		Com::println();
	}
	Com::printErrorFLN(Com::tDryModeUntilRestart);
} // reportTempsensorAndHeaterErrors


Extruder *Extruder::current;

#if NUM_EXTRUDER>0
const char ext0_select_cmd[] PROGMEM = EXT0_SELECT_COMMANDS;
const char ext0_deselect_cmd[] PROGMEM = EXT0_DESELECT_COMMANDS;
#endif // NUM_EXTRUDER>0

#if NUM_EXTRUDER>1
const char ext1_select_cmd[] PROGMEM = EXT1_SELECT_COMMANDS;
const char ext1_deselect_cmd[] PROGMEM = EXT1_DESELECT_COMMANDS;
#endif // NUM_EXTRUDER>1

Extruder extruder[NUM_EXTRUDER] =
{
#if NUM_EXTRUDER>0
    {
        0,(int32_t)(EXT0_X_OFFSET_MM * XAXIS_STEPS_PER_MM),(int32_t)(EXT0_Y_OFFSET_MM * YAXIS_STEPS_PER_MM),(int32_t)(EXT0_Z_OFFSET_MM * ZAXIS_STEPS_PER_MM),EXT0_STEPS_PER_MM,EXT0_ENABLE_PIN,EXT0_ENABLE_ON,
        EXT0_MAX_FEEDRATE,EXT0_MAX_ACCELERATION,EXT0_MAX_START_FEEDRATE,0,EXT0_WATCHPERIOD
        ,EXT0_WAIT_RETRACT_TEMP,EXT0_WAIT_RETRACT_UNITS,0

#if USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        ,EXT0_ADVANCE_K
#endif // ENABLE_QUADRATIC_ADVANCE
        ,EXT0_ADVANCE_L,EXT0_ADVANCE_BACKLASH_STEPS
#endif // USE_ADVANCE

        ,{
            0,EXT0_TEMPSENSOR_TYPE,EXT0_SENSOR_INDEX,0,0,0,
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
            0,
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION
            0,
            0,EXT0_PID_INTEGRAL_DRIVE_MAX,EXT0_PID_INTEGRAL_DRIVE_MIN,EXT0_PID_P,EXT0_PID_I,EXT0_PID_D,EXT0_PID_MAX,0,0,
            0,{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
            0,
            0, 0, EXT0_DECOUPLE_TEST_PERIOD
			,0 //uint8_t paused
        }
        ,ext0_select_cmd,ext0_deselect_cmd,EXT0_EXTRUDER_COOLER_SPEED,0
#if STEPPER_ON_DELAY
        , '\x0'
#endif // STEPPER_ON_DELAY by Nibbels gegen xtruder.cpp:1620:1: warning: missing initializer for member 'Extruder::enabled'
    }
#endif // NUM_EXTRUDER>0

#if NUM_EXTRUDER>1
    ,{
        1,(int32_t)(EXT1_X_OFFSET_MM * XAXIS_STEPS_PER_MM),(int32_t)(EXT1_Y_OFFSET_MM * YAXIS_STEPS_PER_MM),(int32_t)(EXT1_Z_OFFSET_MM * ZAXIS_STEPS_PER_MM),EXT1_STEPS_PER_MM,EXT1_ENABLE_PIN,EXT1_ENABLE_ON,
        EXT1_MAX_FEEDRATE,EXT1_MAX_ACCELERATION,EXT1_MAX_START_FEEDRATE,0,EXT1_WATCHPERIOD
        ,EXT1_WAIT_RETRACT_TEMP,EXT1_WAIT_RETRACT_UNITS,0

#if USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        ,EXT1_ADVANCE_K
#endif // ENABLE_QUADRATIC_ADVANCE
        ,EXT1_ADVANCE_L,EXT1_ADVANCE_BACKLASH_STEPS
#endif // USE_ADVANCE

        ,{
            1,EXT1_TEMPSENSOR_TYPE,EXT1_SENSOR_INDEX,0,0,0,
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
            0,
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION
            0,
            0,EXT1_PID_INTEGRAL_DRIVE_MAX,EXT1_PID_INTEGRAL_DRIVE_MIN,EXT1_PID_P,EXT1_PID_I,EXT1_PID_D,EXT1_PID_MAX,0,0,
            0,{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
            0,
            0, 0, EXT1_DECOUPLE_TEST_PERIOD
			,0 //uint8_t paused
        }
        ,ext1_select_cmd,ext1_deselect_cmd,EXT1_EXTRUDER_COOLER_SPEED,0
#if STEPPER_ON_DELAY
        , '\x0'
#endif // STEPPER_ON_DELAY by Nibbels gegen xtruder.cpp:1620:1: warning: missing initializer for member 'Extruder::enabled'
    }
#endif // NUM_EXTRUDER>1
};

#if HAVE_HEATED_BED
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER+1
TemperatureController heatedBedController = {
		NUM_EXTRUDER,HEATED_BED_SENSOR_TYPE,BED_SENSOR_INDEX,0,0,0,
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
		0,
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION
		0,
		0,HEATED_BED_PID_INTEGRAL_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MIN,HEATED_BED_PID_PGAIN,HEATED_BED_PID_IGAIN,HEATED_BED_PID_DGAIN,HEATED_BED_PID_MAX,0,0,
		0,{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
		0,
		0, 0, BED_DECOUPLE_TEST_PERIOD,
		0 //uint8_t paused
    };
#else
#define NUM_TEMPERATURE_LOOPS NUM_EXTRUDER
#endif // HAVE_HEATED_BED



#if RESERVE_ANALOG_INPUTS
/** \brief This is an optional Temperature Sensor. //Nibbels
There is no Controller involved! It is not hooked into tempController
Do not try to do anything other than updateCurrentTemperature and reading the Temps.
TODO: Making a totally clean class, without the logic to controll something.
*/
TemperatureController optTempController = {
        0,RESERVE_ANALOG_SENSOR_TYPE,RESERVE_SENSOR_INDEX,0,0,0,
#if FEATURE_HEAT_BED_TEMP_COMPENSATION
        0,
#endif // FEATURE_HEAT_BED_TEMP_COMPENSATION
        0,
        0,0,0,0,0,0,0,0,0,
        0,{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        0,
        0, 0, 0 /*0 = no decouple test*/,
		0 //uint8_t paused
    };
#endif // RESERVE_ANALOG_INPUTS


TemperatureController *tempController[NUM_TEMPERATURE_LOOPS] =
{
#if NUM_EXTRUDER>0
    &extruder[0].tempControl
#endif // NUM_EXTRUDER>0

#if NUM_EXTRUDER>1
    ,&extruder[1].tempControl
#endif // NUM_EXTRUDER>1

#if HAVE_HEATED_BED
#if NUM_EXTRUDER==0
    &heatedBedController
#else
    ,&heatedBedController
#endif // NUM_EXTRUDER==0
#endif // HAVE_HEATED_BED
};
