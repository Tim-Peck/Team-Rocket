/*Rough pseudocode outline of data compression and decompression functions

Expected strings

See to interperate GNSS GGA string https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html

Required  Data	Bits required	  Sensor Output
Accl_X	  16	  16 Bits         
Accl_Y	  16	  16 Bits         
Accl_Z	  16	  16 Bits         
Bat_voltage	12	12 Bits         
Longitude	42	  42 Bits         
Latidue	  42	  42 Bits         
Altitude	42	  18 Bits         
UTC time	18	  18 Bits         


string data_compression(string gnss, float bat_volt, float accl_x, float accl_y, float accl_z);

outputString = data_compression(gnss, bat_volt, accl_x, accl_y, accl_z) {
//output string format = UTC,alt,lat,long,bat,accl_x,accl_y,accl_z

convert gnss string to char array, extract UTC, alt, lat, and long at relavant sizes

convert to format uses least bits

repeat for other data types

combine into 1 output message?
}

sample NMEA String
$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F

*/

#include <stdio.h>

char NMEA[] = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F";
float bat_volt = 2.54;
float accl_x = 0.25;
float accl_y = 3.01;
float accl_z = 30.45;


void data_compression(char *NMEA, float bat_volt, float accl_x, float accl_y, float accl_z);


int main() {
    data_compression(NMEA, bat_volt, accl_x, accl_y, accl_z);
}


void data_compression(char *NMEA, float bat_volt, float accl_x, float accl_y, float accl_z) {
const int locationRes = 13; //Placeholder variable sizes
const int timeRes = 8;
char latitude[locationRes] = {0};
char longitude[locationRes] = {0};
char altitude[locationRes] = {0};
char UTC[timeRes] = {0};
int commaCount = 0;
int lastComma = 0;
int currentComma = 0;
int index = 0;

//detect commas
while (commaCount <= 10) {
    if (NMEA[index] == ',') { //triggers action every time comma is located
        commaCount++;
        lastComma = currentComma;
        currentComma = index;
        int n = 1;
        int i;

        //Current configuration records the least signifigant bits of the location, assuming the gernal area is known
        for (i = currentComma-1; i > lastComma; i--) { //This statement copies specific parts of the string
            switch(commaCount) {
                //if statement prevents out of bound array elements being accessed
                //Counts down backwards through both NMEA string and target array
                case (2): //Sets UTC time
                    if(n <= timeRes) UTC[timeRes - n] = NMEA[i]; 
                break;
                case (3): //Sets latitude variable in DMS format (North/South assumed)
                    if(n <= locationRes) latitude[locationRes - n] = NMEA[i];
                break;
                case (5): //Sets longitude variable in DMS format (East/West Assumed)
                    if(n <= locationRes) longitude[locationRes - n] = NMEA[i];
                break;
                case (10): //Sets altitude in meters
                    if(n <= locationRes) altitude[locationRes - n] = NMEA[i];
                break;
            }
            n++;
        }
    }
    index++; //Increments the index value to keep track of current location in the string
}

//Take printf results with grain of salt, the individual variables seem to be storing the correct value, 
//however printf is printing wrong
/* 
printf("UTC = %s \n",UTC);
printf("Latitude = %s \n",latitude);
printf("Longitude = %s \n",longitude);
printf("Altitude = %s \n",altitude);
*/
 //return UTC + latitude + longitude + altitude;
}




