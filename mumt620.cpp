

#include <stdio.h>
#include <cmath>
#include <unistd.h> 
#include <fcntl.h>  
#include <termios.h> 

//g++ -g /Users/louisdeng/Desktop/tmp/mumt620.cpp -o /Users/louisdeng/Desktop/tmp/mumt620.o -Istk/include/ -Lstk/src/ -D__MACOSX_CORE__ -lstk -lpthread -framework CoreAudio -framework CoreMIDI -framework CoreFoundation

#include <RtAudio.h>
#include <Delay.h>
#include <Echo.h>
#include <PoleZero.h>

using namespace stk;
using namespace std;

bool done;

//if delay from controller is too high: lower this value. 
//if audio is glitchy, increase this value. 
unsigned int bufferamt = 128; 

// at least 38400 (1 byte of message per sample, 7 samples to receive all message) to avoid glitching, use >38400. 
const int baudRate = 115200;    //recommended = 115200
char* newPath;
const char initPath[] = "/dev/tty.";
const char defaultPathStr[] = "/dev/tty.usbmodem144301";
int portmsg, dt;
unsigned char buff[16];

//create controller data set
struct ControllerData{
    int r_read, t_read;
    int fsr1, fsr2, fsr3, fsr4;
    ControllerData()
        : r_read(0), t_read(0), fsr1(0), fsr2(0), fsr3(0), fsr4(0) {}
};
//create scaled data set
StkFloat rscaled;
StkFloat fsr1scaled, fsr2scaled, fsr3scaled, fsr4scaled;

ControllerData controldata;

//obselete: 
int nowread = 0;

//usb serial port initialization function
//adapted from unsigned author in arduino forum: https://forum.arduino.cc/t/mac-os-x-c-question/41272/5
void init_port(int *fd, unsigned int baud)
{
    struct termios options;
    tcgetattr(*fd,&options);
    switch(baud)
    {
        case 9600: cfsetispeed(&options,B9600);
            cfsetospeed(&options,B9600);
            break;
        case 19200: cfsetispeed(&options,B19200);
            cfsetospeed(&options,B19200);
            break;
        case 38400: cfsetispeed(&options,B38400);
            cfsetospeed(&options,B38400);
            break;
        case 115200: cfsetispeed(&options,B115200);
            cfsetospeed(&options,B115200);
            break;
        default:cfsetispeed(&options,B115200);
            cfsetospeed(&options,B115200);
            break;
    }
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    tcsetattr(*fd,TCSANOW,&options);
}

//this receive message function recmsg() will update control data inputs when the order matches. 
void recmsg( ControllerData* dataptr ) {
    register int rec;
    register int v0;
    rec = read( portmsg, buff, 7 );
    v0 = (int)buff[0];
    if ( v0 == 255 ) {
        dataptr->r_read = (int)buff[1];
        dataptr->t_read = (int)buff[2];
        dataptr->fsr1 = (int)buff[3];
        dataptr->fsr2 = (int)buff[4];
        dataptr->fsr3 = (int)buff[5];
        dataptr->fsr4 = (int)buff[6];
    }
}  

//ping pong delay
//        1           to2 ~
//  --iL--.--+--[dL]--.--+-->dry/wet
//
//  --iR--.--+--[dR]--.--+-->dry/wet
//        2           to1 ~%
//simple buffered sample delay
Echo dLineL( (unsigned long)Stk::sampleRate()*2 );
Echo dLineR( (unsigned long)Stk::sampleRate()*2 );
unsigned long dLenL = 24107;
unsigned long dLenR = 18203;

void ppDelay(StkFloat *inL, StkFloat *inR, StkFloat dry, StkFloat wet, unsigned long dL, unsigned long dR, StkFloat crov2L, StkFloat crov2R){
    //in-output 2 samples, left and right
    StkFloat din[2];
    StkFloat dtemp[2];

    dLineL.setDelay(dL);
    dLineR.setDelay(dR);

    din[0] = *inL;
    din[1] = *inR;
    
    dtemp[0] = dLineL.tick( din[0] + crov2L*dLineR.lastOut() ) ;
    dtemp[1] = dLineR.tick( din[1] + crov2R*dLineL.lastOut() ) ;

    *inL = dry*din[0] + wet*dtemp[0];
    *inR = dry*din[1] + wet*dtemp[1];

}

//cross over reverb
//variation of the JCRev
//                                       1      vLineL->to2
//  -->iL--[SchoederAP1]--[SchoederAP2]--.------.--[SchoederAP3]------4xFBCF---->oL(dry/wet)
//
//  -->iR--[SchoederAP1]--[SchoederAP2]--.------.--[SchoederAP3]------4xFBCF---->oR(dry/wet)
//                                       2      vLineR->to1
//
//initialize some necessary components for building the reverb
Delay ap_1[2];  //L,R
Delay ap_2[2];  //L,R
Delay ap_3[2];  //L,R
StkFloat fbcfL[] = {0.046, 0.057, 0.041, 0.054};
StkFloat fbcfR[] = {0.047, 0.056, 0.042, 0.053};
Delay cb_L[4];  //1,2,3,4
Delay cb_R[4];  //1,2,3,4
StkFloat apcoeff = 0.7;
//simple buffered sample delay with shared delay length
Echo vLineL( (unsigned long)Stk::sampleRate()*2 );  //L
Echo vLineR( (unsigned long)Stk::sampleRate()*2 );  //R
unsigned long vLen = 23091;

void coRev(StkFloat *inL, StkFloat *inR, StkFloat dry, StkFloat wet, unsigned long sharedLen, StkFloat crov2L, StkFloat crov2R){
    //in-output 2 samples, left and right
    StkFloat vin[2];
    StkFloat ap1out[2];
    StkFloat tmp1[2];
    StkFloat ap2out[2];
    StkFloat tmp2[2];
    StkFloat tmplattice[2];
    StkFloat ap3out[2];
    StkFloat tmp3[2];
    StkFloat cbout[2];

    vLineL.setDelay(sharedLen);
    vLineR.setDelay(sharedLen);

    vin[0] = *inL;
    vin[1] = *inR;

    //1st schroeder AP
    for (int i = 0; i < 2; i++ ){
        tmp1[i] = vin[i] + apcoeff * ap_1[i].lastOut();
        ap1out[i] = -apcoeff * tmp1[i] + ap_1[i].lastOut();
        ap_1[i].tick(tmp1[i]);
    }
    //2nd AP
    for (int i = 0; i < 2; i++ ){
        tmp2[i] = ap1out[i] + apcoeff * ap_2[i].lastOut();
        ap2out[i] = -apcoeff * tmp2[i] + ap_2[i].lastOut();
        ap_2[i].tick(tmp2[i]);
    }
    //cross-over lengthy delay line (lattice structure)
    tmplattice[0] = vLineR.lastOut();
    tmplattice[1] = vLineL.lastOut();
    for (int i = 0; i < 2; i++ ){
        ap2out[i] += tmplattice[i];
    }
    vLineL.tick( tmplattice[0] );
    vLineR.tick( tmplattice[1] );
    //3rd AP
    for (int i = 0; i < 2; i++ ){
        tmp3[i] = ap2out[i] + apcoeff * ap_3[i].lastOut();
        ap3out[i] = -apcoeff * tmp3[i] + ap_3[i].lastOut();
        ap_3[i].tick(tmp3[i]);
    }
    //4 x feedbackcombfilter
    cbout[0] = 0.0;
    cbout[1] = 0.0;
    for (int i = 0; i < 4; i++ ){
        cbout[0] += cb_L[i].lastOut();
        cb_L[i].tick( cb_L[1].lastOut() );
    }
    for (int i = 0; i < 4; i++ ){
        cbout[1] += cb_R[i].lastOut();
        cb_R[i].tick( cb_R[1].lastOut() );
    }
    //output dry-wet mixing
    *inL = dry*vin[0] + wet*cbout[0];
    *inR = dry*vin[1] + wet*cbout[1];

}

int tick( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void *dataPointer ){
    //do tick computation
    register StkFloat *oSamples = (StkFloat *) outputBuffer, *iSamples = (StkFloat *) inputBuffer;
    register StkFloat sampleL, sampleR;

    //2-Channel
    for (int i=(int)nBufferFrames; i>0; i-- ){
        
        //Retrieve input
        sampleL = *iSamples++ * 0.8;
        sampleR = *iSamples++ * 0.8;
        
        //Stereo ping-pong delay
        ppDelay(&sampleL, &sampleR, 1-0.33*rscaled, 1.1*rscaled, (unsigned long)dLenL*(0.1+fsr1scaled), (unsigned long)dLenR*(0.1+fsr4scaled), 0.6500*(0.5+fsr2scaled), -0.3750*(0.5+fsr3scaled));
        //Stereo crossover JCRev
        coRev(&sampleL, &sampleR, 1-0.33*rscaled, 1.1*rscaled, (unsigned long)vLen*(fsr1scaled+fsr4scaled)/2, fsr2scaled, fsr3scaled);
        //hard limiter
        
        if( abs(sampleL) >= 1.0){
            sampleL = sampleL / abs( sampleL );
        }
        if( abs(sampleR) >= 1.0){
            sampleR = sampleR / abs( sampleR );
        }
        
        //Inject output
        *oSamples++ = sampleL;
        *oSamples++ = sampleR;
    }
    
    //continue stream
    return 0;
}

int main(int argc, char *argv[]){

    //initialize some delays...
    ap_1[0].setDelay( 1051 );
    ap_1[0].setDelay( 1063 );
    ap_2[0].setDelay( 337 );
    ap_2[0].setDelay( 317 );
    ap_3[0].setDelay( 113 );
    ap_3[0].setDelay( 127 );
    for ( int i = 0; i < 4; i++ ){
        cb_L[i].setDelay( (unsigned long)Stk::sampleRate()*fbcfL[i] );
    }
    for ( int i = 0; i < 4; i++ ){
        cb_R[i].setDelay( (unsigned long)Stk::sampleRate()*fbcfR[i] );
    }

    //create session
    RtAudio session;
    unsigned int numApi = session.getDeviceCount();
    std::cout << "Created session. " << std::endl;
    std::cout << "Numbers of API: " << numApi << std::endl;
    //cycle through all the devices
    RtAudio::DeviceInfo iodevice[numApi];
    for (int d = 0; d < numApi; d++){
        iodevice[d] = session.getDeviceInfo(d);
        std::cout << "Device [" << d << "]: " << iodevice[d].name <<std::endl;
    }
    //retrieve infos
    unsigned int defInputInd = session.getDefaultInputDevice();
    unsigned int defOutputInd = session.getDefaultOutputDevice();
    unsigned int customInput, customOutput;
    std::cout << "Default input device: " << iodevice[defInputInd].name << ", index number: " << defInputInd << std::endl;
    std::cout << "Default output device: " << iodevice[defOutputInd].name << ", index number: " << defOutputInd << std::endl;
    std::cout << "Which device do you want to use as INPUT? Enter device API number. " << std::endl;
    std::cin >> customInput;
    std::cout << "and OUT? Enter device API number. " << std::endl;
    std::cin >> customOutput;
    if (customInput >= numApi || customOutput >= numApi){
        std::cout << "Wrong index for input or output device, exiting program! " << std::endl;
        exit(0);
    }
    RtAudio::StreamParameters iparam;
    iparam.deviceId = customInput;
    iparam.nChannels = 2;
    RtAudio::StreamParameters oparam;
    oparam.deviceId = customOutput;
    oparam.nChannels = 2;
    
    //open stream
    try{
        session.openStream( &oparam, &iparam, RTAUDIO_FLOAT64, (unsigned int)Stk::sampleRate(), &bufferamt, &tick );
        std::cout << "\nSucessfully opened RtAudio Stream! \n" << std::endl;
    }catch(RtAudioError & ){
        goto cleanup;
    }

    //open serial port
    if ( argc == 1 ) {
        newPath = (char*)malloc(strlen(defaultPathStr)); 
        strcpy(newPath, defaultPathStr ); 
        std::cout << "trying DEFAULT serial port at: " << newPath << std::endl;
    } else {
        newPath = (char*)malloc(strlen(initPath)+strlen(argv[1])+1); 
        strcpy(newPath, initPath ); 
        strcat(newPath, argv[1]);
        std::cout << "trying CUSTOM serial port at: " << newPath << std::endl;
    }
    
    portmsg = open( newPath, O_RDWR | O_NOCTTY | O_NDELAY );
    if( portmsg == -1 ){
        std::cout << "cannot open serial port" << std::endl;
        exit(0);
    } else {
        std::cout << "\nSuccesfully opened serial port to device " << newPath << "\n" << std::endl;
        init_port( &portmsg, baudRate );
    }
    

    //start stream
    try{
        session.startStream();
        std::cout << "\n\n\n * * * stream running! * * * \n\n\n" << std::endl; 
    }catch(RtAudioError & ){
        goto cleanup;
    }

    //check done status periodically
    while (!done){
        //check serial messages and overwrite read values
        recmsg( &controldata );
        //scale control value
        rscaled = controldata.r_read/200.0;
        fsr1scaled = controldata.fsr1/250.0;
        fsr2scaled = controldata.fsr2/250.0;
        fsr3scaled = controldata.fsr3/250.0;
        fsr4scaled = controldata.fsr4/250.0;
        //std::cout << controldata.r_read << "\t" << controldata.t_read << "\t" << controldata.fsr1 << "\t" << controldata.fsr2 << "\t" << controldata.fsr3 << "\t" << controldata.fsr4 << "\t" << std::endl;
        //Stk::sleep(1);
    }

    //close
    try{
        session.closeStream();
        close(portmsg);
    }catch(RtAudioError & ){
        exit(0);
    }
    //for (int i=0; i<numApi; i++){}
    cleanup:
    std::cout << "\nexiting program\n\n";
    return 0;
}
 