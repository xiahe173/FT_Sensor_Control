#include <Arduino.h>
#include <iarduino_ADC_CS1237.h>

const int CLK = 13;
const int DOUT[] = {7, 8, 9, 10, 11, 12};
const int adc_freq = 1280;         // Hz, [10, 40, 640, 1280]
const int serial_update_freq = 20; // Hz
const int average_window_len = 10;
int32_t calib[6] = {0, 0, 0, 0, 0, 0};
int32_t samples[6] = {0, 0, 0, 0, 0, 0};

iarduino_ADC_CS1237 adc_channels[6] = {
    iarduino_ADC_CS1237(CLK, DOUT[0]),
    iarduino_ADC_CS1237(CLK, DOUT[1]),
    iarduino_ADC_CS1237(CLK, DOUT[2]),
    iarduino_ADC_CS1237(CLK, DOUT[3]),
    iarduino_ADC_CS1237(CLK, DOUT[4]),
    iarduino_ADC_CS1237(CLK, DOUT[5])};

void calibrate()
{
    // read 100 samples and calculate average for 6 channels to set the offset
    Serial.println("Calibrating channels");
    for (int k = 0; k < 6; k++)
    {
        calib[k] = 0;
    }
    for (int j = 0; j < 100; j++)
    {
        for (int k = 0; k < 6; k++)
        {
            calib[k] += adc_channels[k].analogRead() / 100;
        }
        delay(20);
    }
    Serial.print("Calibration done, new offsets are:");
    for (int k = 0; k < 6; k++)
    {
        Serial.print(String(calib[k]) + ",");
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Start");

    for (int i = 0; i < 6; i++)
    {
        bool state;
        state = adc_channels[i].begin();
        state = state && adc_channels[i].setSpeed(adc_freq);
        state = state && adc_channels[i].setPGA(128);
        state = state && adc_channels[i].setPinVrefOut(true);
        if (!state)
        {
            Serial.println("Error init channel " + String(i));
        }
        else
            Serial.println("Channel " + String(i) + " initialized");
    }
    calibrate();
    //    Configure the ADC:                                                        //
    //    i=adc.setPulseWidth(30);    if( !i ){ Serial.println("Error width"  ); }  // Set the pulse width on the SCL line. Default is 5µs. Increase is required only for long lines. The function is called before initialization begin().
    //    i=adc.begin();              if( !i ){ Serial.println("Error begin"  ); }  // Initialize the ADC.
    //    i=adc.setPinVrefOut(true);  if( !i ){ Serial.println("Error VrefOut"); }  // Enable the VrefOut pin. Possible values: true=on (default), false=off. When the VrefOut output is on, a smoothed supply voltage of the chip (Vcc) will appear, which can be applied to the VrefIn input.
    //    i=adc.setVrefIn(5.0);       if( !i ){ Serial.println("Error VrefIn" ); }  // Set the reference voltage applied to the VrefIn input. Default is 5V. The VrefIn input can receive either an external voltage from 1.5 to Vcc+0.1, or the chip's supply voltage (Vcc) from the VrefOut output.
    //    i=adc.setSpeed(40);         if( !i ){ Serial.println("Error speed"  ); }  // Set the conversion speed to 10Hz. Possible values: 10, 40, 640, 1280 Hz. This is the frequency of new data readiness for reading. Default is 10 Hz.
    //    i=adc.setPGA(1);            if( !i ){ Serial.println("Error gain"   ); }  // Set the gain factor. Possible values: 1, 2, 64, 128. Default is 128. Measured voltages are in the range ±0.5 VrefIn / PGA.
    //    i=adc.setChannel(0);        if( !i ){ Serial.println("Error channel"); }  // Select the ADC channel. Possible values: 0, 2, 3. 0-Channel A (default), 1-Reserved, 2-Temperature, 3-Internal short circuit.
    //
    //    //   Read ADC settings:                                                   //
    //    bool     pin   = adc.getPinVrefOut();                                     // Get the state of the VrefOut pin. The function returns true-on or false-off.
    //    uint16_t speed = adc.getSpeed();                                          // Get the current conversion speed in Hertz. The function returns the frequency of new data readiness: 10, 40, 640, or 1280 Hz.
    //    uint8_t  gain  = adc.getPGA();                                            // Get the current gain factor. The function returns the value: 1, 2, 64, or 128.
    //    uint8_t  chan  = adc.getChannel();                                        // Get the used ADC channel. The function returns one of the channels: 0, 2, or 3.
    //    uint16_t width = adc.getPulseWidth();                                     // Get the pulse width in µs. The function returns the pulse and pause duration on the SCL line used by the library in µs.
    //    float    Vref  = adc.getVrefIn();                                         // Get the reference voltage value VrefIn used by the library to calculate the voltage returned by the getVoltage() function.
    //    // Output the read ADC settings:                                           //
    //    Serial.println( (String) "VrefOut pin is "+(pin?"en":"dis")+"abled."    ); //
    //    Serial.println( (String) "Data update rate is "+speed+" Hz."            ); //
    //    Serial.println( (String) "Gain factor is "+gain+"x."                    ); //
    //    Serial.println( (String) "Using channel "+chan+" of the ADC."           ); //
    //    Serial.println( (String) "Pulse width on the SCL line is "+width+" µs." ); //
    //    Serial.println( (String) "Reference voltage VrefIn is "+Vref+" V."      ); //
}

void loop()
{
    if (Serial.available())
    {
        char c = Serial.read();
        if (c == 't')
        {
            calibrate();
        }
    }

    // for (int i = 0; i < 6; i++)
    // {
    //     Serial.print(String(adc_channels[i].analogRead() - calib[i]) + ",");
    // }
    // Serial.println();
    // delay(1000 / serial_update_freq);

    for (int k = 0; k < 6; k++)
    {
        samples[k] = 0;
    }
    for (int j = 0; j < average_window_len; j++)
    {
        for (int k = 0; k < 6; k++)
        {
            samples[k] += adc_channels[k].analogRead() / average_window_len;
        }
        // delay(1000 / serial_update_freq / 4);
    }
    for (int k = 0; k < 6; k++)
    {
        Serial.print(String(samples[k] - calib[k]) + ",");
    }
    Serial.println();
}
