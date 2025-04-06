#include "FujiHeatPump.h"



FujiFrame FujiHeatPump::decodeFrame() {
    FujiFrame ff;

    ff.messageSource       =  readBuf[0];
    ff.messageDest         =  readBuf[1] & 0b01111111;
    ff.messageType         = (readBuf[2] & 0b00110000) >> 4;

    ff.acError             = (readBuf[kErrorIndex] & kErrorMask) >> kErrorOffset;
    ff.temperature         = (readBuf[kTemperatureIndex] & kTemperatureMask) >> kTemperatureOffset;
    ff.acMode              = (readBuf[kModeIndex] & kModeMask) >> kModeOffset;
    ff.fanMode             = (readBuf[kFanIndex] & kFanMask) >> kFanOffset;
    ff.economyMode         = (readBuf[kEconomyIndex] & kEconomyMask) >> kEconomyOffset;
    ff.swingMode           = (readBuf[kSwingIndex] & kSwingMask) >> kSwingOffset;
    ff.swingStep           = (readBuf[kSwingStepIndex] & kSwingStepMask) >> kSwingStepOffset;
    ff.controllerPresent   = (readBuf[kControllerPresentIndex] & kControllerPresentMask) >> kControllerPresentOffset;
    ff.updateMagic         = (readBuf[kUpdateMagicIndex] & kUpdateMagicMask) >> kUpdateMagicOffset;
    ff.onOff               = (readBuf[kEnabledIndex] & kEnabledMask) >> kEnabledOffset;
    ff.controllerTemp      = (readBuf[kControllerTempIndex] & kControllerTempMask) >> kControllerTempOffset; // there is one leading bit here that is unknown - probably a sign bit for negative temps?

    ff.writeBit =   (readBuf[2] & 0b00001000) != 0;
    ff.loginBit =   (readBuf[1] & 0b00100000) != 0;
    ff.unknownBit = (readBuf[1] & 0b10000000)  > 0;

    return ff;
}

void FujiHeatPump::encodeFrame(FujiFrame ff){

    memset(writeBuf, 0, 8);

    writeBuf[0] = ff.messageSource;

    writeBuf[1] &= 0b10000000;
    writeBuf[1] |= ff.messageDest & 0b01111111;

    writeBuf[2] &= 0b11001111;
    writeBuf[2] |= ff.messageType << 4;

    if(ff.writeBit){
        writeBuf[2] |= 0b00001000;
    } else {
        writeBuf[2] &= 0b11110111;
    }

    writeBuf[1] &= 0b01111111;
    if(ff.unknownBit) {
        writeBuf[1] |= 0b10000000;
    }

    if(ff.loginBit){
        writeBuf[1] |= 0b00100000;
    } else {
        writeBuf[1] &= 0b11011111;
    }

    writeBuf[kModeIndex] =              (writeBuf[kModeIndex]              & ~kModeMask)              | (ff.acMode << kModeOffset);
    writeBuf[kModeIndex] =              (writeBuf[kEnabledIndex]           & ~kEnabledMask)           | (ff.onOff << kEnabledOffset);
    writeBuf[kFanIndex] =               (writeBuf[kFanIndex]               & ~kFanMask)               | (ff.fanMode << kFanOffset);
    writeBuf[kErrorIndex] =             (writeBuf[kErrorIndex]             & ~kErrorMask)             | (ff.acError << kErrorOffset);
    writeBuf[kEconomyIndex] =           (writeBuf[kEconomyIndex]           & ~kEconomyMask)           | (ff.economyMode << kEconomyOffset);
    writeBuf[kTemperatureIndex] =       (writeBuf[kTemperatureIndex]       & ~kTemperatureMask)       | (ff.temperature << kTemperatureOffset);
    writeBuf[kSwingIndex] =             (writeBuf[kSwingIndex]             & ~kSwingMask)             | (ff.swingMode << kSwingOffset);
    writeBuf[kSwingStepIndex] =         (writeBuf[kSwingStepIndex]         & ~kSwingStepMask)         | (ff.swingStep << kSwingStepOffset);
    writeBuf[kControllerPresentIndex] = (writeBuf[kControllerPresentIndex] & ~kControllerPresentMask) | (ff.controllerPresent << kControllerPresentOffset);
    writeBuf[kUpdateMagicIndex] =       (writeBuf[kUpdateMagicIndex]       & ~kUpdateMagicMask)       | (ff.updateMagic << kUpdateMagicOffset);
    writeBuf[kControllerTempIndex] =    (writeBuf[kControllerTempIndex]    & ~kControllerTempMask)    | (ff.controllerTemp << kControllerTempOffset);

}

byte FujiHeatPump::fujiAddrToIndex(byte fa)
{
    // index for our lastObserved{State,Time} array
    // [0]: Start
    // [1]: Unit
    // [2]: Primary
    // [3]: Secondary
    // [4]: Unknown - byte was probably invalid, but we may as well store it
    fa = fa == 32 ? 2 : fa;
    fa = fa == 33 ? 3 : fa;
    fa = fa  >  4 ? 4 : fa;
    return fa;
}

void FujiHeatPump::connect(HardwareSerial *serial, bool secondary){
    return this->connect(serial, secondary, -1, -1);
}

void FujiHeatPump::connect(HardwareSerial *serial, bool secondary, int rxPin=-1, int txPin=-1){
    _serial = serial;
    if(rxPin != -1 && txPin != -1) {
#ifdef ESP32
        _serial->begin(500, SERIAL_8E1, rxPin, txPin);
#else
        Serial.print("Setting RX/TX pin unsupported, using defaults.\n");
        _serial->begin(500, SERIAL_8E1);
#endif
    } else {
        _serial->begin(500, SERIAL_8E1);
    }
    _serial->setTimeout(200);
    
    if(secondary) {
        controllerIsPrimary = false;
        controllerAddress = static_cast<byte>(FujiAddress::SECONDARY);
    } else {
        controllerIsPrimary = true;
        controllerAddress = static_cast<byte>(FujiAddress::PRIMARY);
    }
    
    lastFrameReceived = 0;
}

void FujiHeatPump::printFrame(byte buf[8], FujiFrame ff) {
  Serial.printf("%02X %02X %02X %02X %02X %02X %02X %02X  ", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  Serial.printf(" mSrc: %2d mDst: %2d mType: %d write: %d login: %d unknown: %d onOff: %d temp: %2d, mode: %d cP:%d uM:%2d cTemp:%2d acError:%d \n", ff.messageSource, ff.messageDest, ff.messageType, ff.writeBit, ff.loginBit, ff.unknownBit, ff.onOff, ff.temperature, ff.acMode, ff.controllerPresent, ff.updateMagic, ff.controllerTemp, ff.acError);

}

void FujiHeatPump::printFrameFriendly(byte buf[8], FujiFrame ff) {
    Serial.printf("%02X %02X %02X %02X %02X %02X %02X %02X @%3d  ", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], millis()/1000);   
    // Serial.printf(" %9s ==> %9s: %7s, write:%d, login:%d, unk:%d, onOff:%d, temp:%2d, mode:%d, cP:%d, uM:%2d, cTemp:%2d, err:%d \n",
    //             src.c_str(), dst.c_str(), type.c_str(), ff.writeBit, ff.loginBit, ff.unknownBit, ff.onOff, ff.temperature, ff.acMode, ff.controllerPresent, ff.updateMagic, ff.controllerTemp, ff.acError);
    Serial.printf("%9s ==> %9s: %7s,wr%d,log%d,unk%d,on%d,tmp%2d,mod%d,cP%d,uM%2d,cTmp%2d,err%d\n",
                 ToString(static_cast<FujiAddress>(ff.messageSource)).c_str(), 
                 ToString(static_cast<FujiAddress>(ff.messageDest)).c_str(), 
                 ToString(static_cast<FujiMessageType>(ff.messageType)).c_str(), 
                 ff.writeBit, ff.loginBit, ff.unknownBit, ff.onOff, ff.temperature, ff.acMode, ff.controllerPresent, ff.updateMagic, ff.controllerTemp, ff.acError);
  }

void FujiHeatPump::sendPendingFrame() {
    if(pendingFrame && (millis() - lastFrameReceived) > 50) {
        _serial->write(writeBuf, 8);
        _serial->flush();
        pendingFrame = false;
        updateFields = 0;

        byte i = fujiAddrToIndex(controllerAddress);
        memcpy(&lastObservedStates[i], writeBuf, sizeof(FujiFrame));
        lastObservedTimes[i] = millis();

        _serial->readBytes(writeBuf, 8); // read back our own frame so we dont process it again
    }
}

bool FujiHeatPump::waitForFrame() {
    FujiFrame ff;
    
    if(_serial->available()) {

        memset(readBuf, 0, 8);
        int bytesRead = _serial->readBytes(readBuf,8);

        if(bytesRead < 8) {
            // skip incomplete frame
            return false;
        }
        
        for(int i=0;i<8;i++) {
            readBuf[i] ^= 0xFF;
        }
    
        ff = decodeFrame();

        if(debugPrint) {
            Serial.printf("< ");
            printFrameFriendly(readBuf, ff);
        }
        
        // Store the last message we received
        if(ff.messageType == static_cast<byte>(FujiMessageType::STATUS))
        {
            // index for our lastObservedStates/lastObservedTimes array
            byte i = fujiAddrToIndex(ff.messageSource);
            memcpy(&lastObservedStates[i], &ff, sizeof(FujiFrame));
            lastObservedTimes[i] = millis();

        }

        if(ff.messageDest == controllerAddress) {
            lastFrameReceived = millis();
            
            if(ff.messageType == static_cast<byte>(FujiMessageType::STATUS)){

                if(ff.controllerPresent == 1) {
                    // we have logged into the indoor unit
                    // this is what most frames are
                    ff.messageSource     = controllerAddress;
                    
                    if(seenSecondaryController) {
                        ff.messageDest       = static_cast<byte>(FujiAddress::SECONDARY);
                        ff.loginBit          = true;
                        ff.controllerPresent = 0;
                    } else {
                        ff.messageDest       = static_cast<byte>(FujiAddress::UNIT);
                        ff.loginBit          = false;
                        ff.controllerPresent = 1;
                    }
                    
                    ff.updateMagic       = 0;
                    ff.unknownBit        = true;
                    ff.writeBit          = 0;
                    ff.messageType       = static_cast<byte>(FujiMessageType::STATUS);
                    
                } else {
                    if(controllerIsPrimary) {
                        // if this is the first message we have received, announce ourselves to the indoor unit
                        ff.messageSource     = controllerAddress;
                        ff.messageDest       = static_cast<byte>(FujiAddress::UNIT);
                        ff.loginBit          = false;
                        ff.controllerPresent = 0;
                        ff.updateMagic       = 0;
                        ff.unknownBit        = true;
                        ff.writeBit          = 0;
                        ff.messageType       = static_cast<byte>(FujiMessageType::LOGIN);
                        ff.controllerTemp    = 7;
                        ff.onOff             = 0;
                        ff.temperature       = 0;
                        ff.acMode            = 0;
                        ff.fanMode           = 0;
                        ff.swingMode         = 0;
                        ff.swingStep         = 0;
                        ff.acError           = 0;
                    } else {
                        // secondary controller never seems to get any other message types, only status with controllerPresent == 0
                        // the secondary controller seems to send the same flags no matter which message type
                        
                        ff.messageSource     = controllerAddress;
                        ff.messageDest       = static_cast<byte>(FujiAddress::UNIT);
                        // if we've not sent a message yet, forward the frame unaltered
                        // if (!seenSecondaryController)
                        // {
                        //     seenSecondaryController = true;
                        // }
                        // // otherwise, do our normal business
                        // else
                        // {
                            
                            ff.loginBit          = false;
                            ff.controllerPresent = 1;
                            ff.updateMagic       = 2;
                            ff.unknownBit        = true;
                            ff.writeBit          = 0;
                        // }
                        
                    }
                    
                }
                
                // if we have any updates, set the flags
                if(updateFields) {
                    ff.writeBit = 1;
                }
                
                if(updateFields & kOnOffUpdateMask) {
                    ff.onOff = updateState.onOff;
                }
                
                if(updateFields & kTempUpdateMask) {
                    ff.temperature = updateState.temperature;
                }
                
                if(updateFields & kModeUpdateMask) {
                    ff.acMode = updateState.acMode;
                }
                
                if(updateFields & kFanModeUpdateMask) {
                    ff.fanMode = updateState.fanMode;
                }
                
                if(updateFields & kSwingModeUpdateMask) {
                    ff.swingMode = updateState.swingMode;
                }
                
                if(updateFields & kSwingStepUpdateMask) {
                    ff.swingStep = updateState.swingStep;
                }

                if(updateFields & kEconomyModeUpdateMask) {
                    ff.economyMode = updateState.economyMode;
                }
                
                memcpy(&currentState, &ff, sizeof(FujiFrame));

            }
            else if(ff.messageType == static_cast<byte>(FujiMessageType::LOGIN)){
                // received a login frame OK frame
                // the primary will send packet to a secondary controller to see if it exists
                ff.messageSource     = controllerAddress;
                ff.messageDest       = static_cast<byte>(FujiAddress::SECONDARY);
                ff.loginBit          = true;
                ff.controllerPresent = 1;
                ff.updateMagic       = 0;
                ff.unknownBit        = true;
                ff.writeBit          = 0;
                
                ff.onOff             = currentState.onOff;
                ff.temperature       = currentState.temperature;
                ff.acMode            = currentState.acMode;
                ff.fanMode           = currentState.fanMode;
                ff.swingMode         = currentState.swingMode;
                ff.swingStep         = currentState.swingStep;
                ff.acError           = currentState.acError;
            } else if(ff.messageType == static_cast<byte>(FujiMessageType::ERROR)) {
                Serial.printf("AC ERROR RECV: ");
                printFrame(readBuf, ff);
                // handle errors here
                return false;
            }
            
            encodeFrame(ff);

            if(debugPrint) {
                Serial.printf("> ");
                printFrameFriendly(writeBuf, ff);
            }

            for(int i=0;i<8;i++) {
                writeBuf[i] ^= 0xFF;
            }
                    
            pendingFrame = true;
                        

        } else if (ff.messageDest == static_cast<byte>(FujiAddress::SECONDARY)) {
            seenSecondaryController = true;
            currentState.controllerTemp = ff.controllerTemp; // we dont have a temp sensor, use the temp reading from the secondary controller
        }
        
        return true;
    }
    
    return false;
}

bool FujiHeatPump::isBound() {
    if(millis() - lastFrameReceived < 1000) {
        return true;
    }
    return false;
}

bool FujiHeatPump::updatePending() {
    if(updateFields) {
        return true;
    }
    return false;
}

void FujiHeatPump::setOnOff(bool o){
    updateFields |= kOnOffUpdateMask;
    updateState.onOff = o ? 1 : 0;   
}
void FujiHeatPump::setTemp(byte t){
    updateFields |= kTempUpdateMask;
    updateState.temperature = t;
}
void FujiHeatPump::setMode(byte m){
    updateFields |= kModeUpdateMask;
    updateState.acMode = m;
}
void FujiHeatPump::setFanMode(byte fm){
    updateFields |= kFanModeUpdateMask;
    updateState.fanMode = fm;
}
void FujiHeatPump::setEconomyMode(byte em){
    updateFields |= kEconomyModeUpdateMask;
    updateState.economyMode = em;
}
void FujiHeatPump::setSwingMode(byte sm){
    updateFields |= kSwingModeUpdateMask;
    updateState.swingMode = sm;
}
void FujiHeatPump::setSwingStep(byte ss){
    updateFields |= kSwingStepUpdateMask;
    updateState.swingStep = ss;  
}

bool FujiHeatPump::getOnOff(){
    return currentState.onOff == 1 ? true : false;
}
byte FujiHeatPump::getTemp(){
    return currentState.temperature;
}
byte FujiHeatPump::getMode(){
    return currentState.acMode;
}
byte FujiHeatPump::getFanMode(){
    return currentState.fanMode;
}
byte FujiHeatPump::getEconomyMode(){
    return currentState.economyMode;
}
byte FujiHeatPump::getSwingMode(){
    return currentState.swingMode;
}
byte FujiHeatPump::getSwingStep(){
    return currentState.swingStep;
}
byte FujiHeatPump::getControllerTemp(){
    return currentState.controllerTemp;
}

FujiFrame *FujiHeatPump::getCurrentState(){
    return &currentState;
}

FujiFrame *FujiHeatPump::getUpdateState(){
    return &updateState;
}

FujiFrame *FujiHeatPump::getLastObservedState(byte id)
{
    byte ix = fujiAddrToIndex(id);
    return &lastObservedStates[ix];
}

unsigned long FujiHeatPump::getLastObservedTime(byte id)
{
    byte ix = fujiAddrToIndex(id);
    return lastObservedTimes[ix];
}


byte FujiHeatPump::getUpdateFields(){
    return updateFields;
}
 

const std::string ToString(const FujiMode fm)
{
    switch (fm)
    {
        case FujiMode::UNKNOWN:
            return "UNKNOWN";
        case FujiMode::FAN:
            return "FAN";
        case FujiMode::DRY:
            return "DRY";
        case FujiMode::COOL:
            return "COOL";
        case FujiMode::HEAT:
            return "HEAT";
        case FujiMode::AUTO:
            return "AUTO";
        default:
            return "invalid";
    }
}
const std::string ToString(const FujiMessageType fmt)
{
    switch (fmt)
    {
        case FujiMessageType::STATUS:
            return "STATUS";
        case FujiMessageType::ERROR:
            return "ERROR";
        case FujiMessageType::LOGIN:
            return "LOGIN";
        case FujiMessageType::UNKNOWN:
            return "UNKNOWN";
        default:
            return "invalid";
    }
}
const std::string ToString(const FujiAddress fa)
{
    switch (fa)
    {
        case FujiAddress::START:
            return "START";
        case FujiAddress::UNIT:
            return "UNIT";
        case FujiAddress::PRIMARY:
            return "PRIMARY";
        case FujiAddress::SECONDARY:
            return "SECONDARY";
        default:
            return "invalid";
    }
}
const std::string ToString(const FujiFanMode ffm)
{
    switch (ffm)
    {
        case FujiFanMode::FAN_AUTO:
            return "FAN_AUTO";
        case FujiFanMode::FAN_QUIET:
            return "FAN_QUIET";
        case FujiFanMode::FAN_LOW:
            return "FAN_LOW";
        case FujiFanMode::FAN_MEDIUM:
            return "FAN_MEDIUM";
        case FujiFanMode::FAN_HIGH:
            return "FAN_HIGH";
        default:
            return "invalid";
    }
}