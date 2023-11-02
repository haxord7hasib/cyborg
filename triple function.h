void read_triple_sensors() { 
    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);
    lox3.rangingTest(&measure3, false);

    // Adjusted values
    int adjustedValue1 = measure1.RangeMilliMeter - 23;
    int adjustedValue2 = measure2.RangeMilliMeter - 23;
    int adjustedValue3 = measure3.RangeMilliMeter - 43;

    // Print adjusted values to Serial
    Serial.print(F("1: "));
    if(measure1.RangeStatus != 4) {
        Serial.print(adjustedValue1);
    } else {
        Serial.print(F("Out of range"));
    }

    Serial.print(F(" "));

    Serial.print(F("2: "));
    if(measure2.RangeStatus != 4) {
        Serial.print(adjustedValue2);
    } else {
        Serial.print(F("Out of range"));
    }

    Serial.print(F(" "));

    Serial.print(F("3: "));
    if(measure3.RangeStatus != 4) {
        Serial.print(adjustedValue3);
    } else {
        Serial.print(F("Out of range"));
    }
  
    Serial.println();
}
