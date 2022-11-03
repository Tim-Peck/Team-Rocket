/*Rough pseudocode outline of data compression and decompression functions

Expected strings

See to interperate GNSS GGA string https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html

Required    Data	Bits required	  Sensor Output
Accl_X	    16	  16 Bits         
Accl_Y	    16	  16 Bits         
Accl_Z	    16	  16 Bits         
Bat_voltage	12	  12 Bits         
Longitude	42	  42 Bits         
Latidue	    42	  42 Bits         
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

where
        //N/S = +ve/-ve
        //E/W = +ve/-ve

*/

#include <stdio.h>

char NMEA[] = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,E,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F";
float bat_volt = 2.54;
float accl_x = 0.25;
float accl_y = 3.01;
float accl_z = 30.45;


void data_compression(char *NMEA, float bat_volt, float accl_x, float accl_y, float accl_z);
int power(int input, int exponent);
int char2Int(char *inputString);
char* int2Bin(int inputInt);


int main() {
    char *test;
    data_compression(NMEA, bat_volt, accl_x, accl_y, accl_z);

    test = int2Bin(5);
    // printf("\nBinary of Given Number is=");    
    // for(int i=10;i>=0;i--)    
    //     {    
    //     printf("%d",*test[i]);    
    //     }    
    free(test);
}


void data_compression(char *NMEA, float bat_volt, float accl_x, float accl_y, float accl_z) {


/*
Once seperated, char -> float -> binary representation -> concatinate all values into single number -> split into char array (1 char = 1 byte)*/

//Add check to ensure correct data is input
static bool firstRun = true;
const int latRes = 13; //Placeholder variable sizes (doesn't 1 byte removed for decimal point, most signficant bit determins direction (N/S, E/W))
const int longRes = 14;
const int timeRes = 7;
const int altRes = 5;
char latitude[latRes] = {0};
char longitude[longRes] = {0};
char altitude[altRes] = {0};
char UTC[timeRes] = {0};
int commaCount = 0;
int lastComma = 0;
int currentComma = 0;
int index = 0;

char outputString[26] = {0}; //this is the output string, all data must fit within this.
 

//detect commas
while (commaCount <= 10) {
    if (NMEA[index] == ',') { //triggers action every time comma is located
        commaCount++;
        lastComma = currentComma;
        currentComma = index;
        int n = 1;
        int i;


        //Current configuration records the least signifigant bits of the location, assuming the gernal area is known
        for (i = lastComma+1; i < currentComma; i++) { //This statement copies specific parts of the string
            if(NMEA[i] == '.') i++; //skip the decimal points since they will always be in the same location.
            switch(commaCount) {
                //if statement prevents out of bound array elements being accessed
                //Counts down backwards through both NMEA string and target array
                case (2): //Sets UTC time
                    if(n <= timeRes) UTC[n] = NMEA[i]; 
                break;
                case (3): //Sets latitude variable in DMS format (North/South assumed)
                    if(n <= latRes) latitude[n] = NMEA[i];
                break;
                case (4):
                    (NMEA[i] == 'N') ? (latitude[0] = '1') : (latitude[0] = '0');
                break;
                case (5): //Sets longitude variable in DMS format (East/West Assumed)
                    if(n <= longRes) longitude[n] = NMEA[i];
                break;
                case (6):
                    (NMEA[i] == 'E') ? (longitude[0] = '1') : (longitude[0] = '0');
                break;
                case (10): //Sets altitude in meters
                    if(n <= altRes) altitude[n-1] = NMEA[i];
                break;
            }
            n++;
        }
    }
    index++; //Increments the index value to keep track of current location in the string
}

//char2Int -> int2binary, storing binary results in each byte in the output byte array

//Take printf results with grain of salt, the individual variables seem to be storing the correct value, 
//however printf is printing wrong
/* 
printf("UTC = %s \n",UTC);
printf("Latitude = %s \n",latitude);
printf("Longitude = %s \n",longitude);
printf("Altitude = %s \n",altitude);
*/
 //return UTC + latitude + longitude + altitude;


/*
Locations for adresses
i = 192 - 207 Accl_X
i = 176 - 191 Accl_Y
i = 160 - 175 Accl_Z
i = 148 - 159 Bat_voltage
i = 106 - 147 Longitude
i = 63 - 105 Latitude
i = 21 - 62 Altitude
i = 2 - 20 UTC time*/

for (int i = 28*8 - 1; i>=0; i--){

}

}



char* int2Bin(int inputInt) {
    const int bitSize = 48; //can convert a maximum of 48 bit numbers
    char *output[bitSize] = {0}; 
    int i;
    for (i = 0; inputInt > 0; i++) {
        output[i] = inputInt%2;
        inputInt = inputInt/2.0;
    }

    int n = 1;
    for (i = i-1; i>=0; i--) { //This right shifts the array to so that the least signfigant bit is in the least signficant array element
        output[bitSize - n] = output[i];
        output[i] = 0;
        n++;
    }
    return output;
}

int char2Int(char *inputString) {
    int size = sizeof(inputString);
    int output = 0;
    for (int i = 0; i < size; i++) {
        output += (inputString[i] - '0') * power(10,i); 
    }
    return output;
}

int power(int input, int exponent) {
    int output = input;
    int i = 1;
    if (exponent == 0) {
        return 1;
    }
    while (i < exponent) {
        output = output * input;
    }
    return output;
}


