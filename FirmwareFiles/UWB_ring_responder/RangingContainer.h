#ifndef RANGING_CONTAINER_H
#define RANGING_CONTAINER_H

class Ranging {
    public:
    const uint64_t SPEED_OF_LIGHT = 299792458L; 
    DW1000Time PollTxTime;
    DW1000Time PollRxTime;
    DW1000Time RespTxTime;
    DW1000Time RespRxTime;
    DW1000Time FinalTxTime;
    DW1000Time FinalRxTime;
    
    DW1000Time Ra;
    DW1000Time Rb;
    DW1000Time Da;
    DW1000Time Db;
    
    int calculateRange() {
        Ra = (RespRxTime - PollTxTime).wrap();
        Rb = (FinalRxTime - RespTxTime).wrap();
        Da = (FinalTxTime - RespRxTime).wrap();
        Db = (RespTxTime - PollRxTime).wrap();
        // Serial.print("Ra: ");
        // Serial.print(Ra);
        // Serial.print(", Rb: ");
        // Serial.print(Rb);
        // Serial.print(", Da: ");
        // Serial.print(Da);
        // Serial.print(", Db: ");
        // Serial.println(Db);
        DW1000Time ToF;
        ToF = ((Ra*Rb)-(Da*Db))/(Ra+Rb+Da+Db);
        //Serial.print("ToF: ");
        //Serial.println(ToF);
        float ToF_microseconds = ToF.getAsMicroSeconds();
        //Serial.print("ToF_microseconds: ");
        //Serial.println(ToF_microseconds);
        int output = ToF_microseconds*SPEED_OF_LIGHT / 1e3;
        //Serial.print("Output: ");
        //Serial.println(output);
        return output;
        
    }

    void initialize() {
      PollTxTime.setTimestamp((int64_t)0);
      PollRxTime.setTimestamp((int64_t)0);
      RespTxTime.setTimestamp((int64_t)0);
      RespRxTime.setTimestamp((int64_t)0);
      FinalTxTime.setTimestamp((int64_t)0);
      FinalRxTime.setTimestamp((int64_t)0);
    }

    void printAll() {
      Serial.print("PollTxTime: ");
      Serial.println(PollTxTime);
      Serial.print("RespRxTime: ");
      Serial.println(RespRxTime);

      Serial.print("RespTxTime: ");
      Serial.println(RespTxTime);
      Serial.print("FinalRxTime: ");
      Serial.println(FinalRxTime);

      Serial.print("FinalTxTime: ");
      Serial.println(FinalTxTime);
      Serial.print("PollRxTime: ");
      Serial.println(PollRxTime);

      
    }
    
};

#endif
