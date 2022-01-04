#include <M5Core2.h>
#include <driver/i2s.h>

#define CONFIG_I2S_BCK_PIN 12
#define CONFIG_I2S_LRCK_PIN 0
#define CONFIG_I2S_DATA_PIN 2
#define CONFIG_I2S_DATA_IN_PIN 34
#define Speak_I2S_NUMBER I2S_NUM_0
#define MODE_MIC 0
#define MODE_SPK 1

#define DATA_SIZE 32
#define SAMPLING_RATE 44100
#define TEMPLATE_SIZE 320

double soundbank[6][TEMPLATE_SIZE]={{0.0}};

//arp pattern
double pattern[7][16] = {
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  {0,0,0,0,0,0,0,0,0.80,0.92,0.93,0.974,1,1,1,1},
  {1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0},
  {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0},
  {1,1,0,0,0.9,0.5,0,0,0.80,0.92,0.93,0.974,1,1,0,0},
  {1,1,1,0,0,0,0,0,0,0,0.90,0.90,0,0,0,0},
  {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0}
  };

double hz[37];

int arp = 0;
double *wave1;
double *wave2;
int wavetype = 0;
int wave1type = 1;
int wave2type = 0;
double phase1 = 0.0;
double phase2 = 0.0;
double beat = 0.0;
double freq = 1;
double detune = 1.015;
double velocity = 0.0;
double attack = 0.0;
double Ga = 1.0;
double decay = 1.0;
double Gd = 1.0;

double pfreq = -1.0;
double pgate = 0.0;

bool touch = false;
unsigned char sdata[DATA_SIZE];
int edit = 0;
double gain = 0.25;
double noiserate = 0.2;

#define delaytime 0.1
#define BPM 4.0*128.0/120.0
#define DELAY_SIZE ((int)(delaytime*BPM*SAMPLING_RATE/2.0))
double delaywave[DELAY_SIZE]={0.0};
int delayphase = 0;
double delayrate = 0.32;

double px=-1.0,py=-1.0;
double nx=-1.0,ny=-1.0;

void SetSoundBank(void)
{
  int i;
  
  //tone
  for(i=0;i<37;++i)
  {
    hz[i] = (double)440*pow(2,i/12.0);
  }

  //sound bank
  for(i=0;i<TEMPLATE_SIZE;++i)
  {
    //sine wave
    soundbank[1][i] = sin(2.0*PI*(double)i/(double)TEMPLATE_SIZE);
    
    //saw wave
    soundbank[2][i] = 1.0-2.0*(double)i/(double)TEMPLATE_SIZE;
    
    //square wave
    if(i<TEMPLATE_SIZE/2)
      soundbank[3][i] = 1.0;
    else
      soundbank[3][i] = -1.0;
    
    //user wave
    soundbank[4][i] = -sin(4.0*PI*(double)i/(double)TEMPLATE_SIZE);
    
    //white noise
    soundbank[5][i] = -1.0+(double)random(1024)/1024.0*2.0;
  }
}

bool InitI2SSpeakOrMic(int mode)
{
    esp_err_t err = ESP_OK;

    i2s_driver_uninstall(Speak_I2S_NUMBER);
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = 128,
    };
    if (mode == MODE_MIC) {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    }
    else {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
        i2s_config.use_apll = false;
        i2s_config.tx_desc_auto_clear = true;
    }
    err += i2s_driver_install(Speak_I2S_NUMBER, &i2s_config, 0, NULL);
    i2s_pin_config_t tx_pin_config;

    tx_pin_config.bck_io_num = CONFIG_I2S_BCK_PIN;
    tx_pin_config.ws_io_num = CONFIG_I2S_LRCK_PIN;
    tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;
    tx_pin_config.data_in_num = CONFIG_I2S_DATA_IN_PIN;
    err += i2s_set_pin(Speak_I2S_NUMBER, &tx_pin_config);
    err += i2s_set_clk(Speak_I2S_NUMBER, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

    return true;
}

void InitDisplay(void)
{
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
}

void InitSpeak(void)
{
  M5.Axp.SetSpkEnable(true);
  InitI2SSpeakOrMic(MODE_SPK);
}

void SetWave(void)
{
  wave1type = wavetype%5+1;
  wave2type = wavetype/5;
  wave1 = soundbank[wave1type];
  wave2 = soundbank[wave2type];
}

void DrawWave(void)
{
  M5.Lcd.fillScreen(BLACK);
  int i;
  for(i=1;i<320;i+=2) {
    int h = (int)(120*wave1[i-1]+120);
    M5.Lcd.drawLine(i-1, 120, i-1, h,TFT_GREEN);
    h = (int)(120*wave2[i]+120);
    M5.Lcd.drawLine(i, 120, i, h,TFT_BLUE);  
  }
  
  if(arp>0) {
    for(i=1;i<320;++i) {
      int pdx = (int)(((double)(i-1)/(double)320.0)*16.0);
      int ndx = (int)(((double)i/(double)320.0)*16.0);
      M5.Lcd.drawLine(i-1, 120+50-100*pattern[arp][pdx], i, 120+50-100*pattern[arp][ndx],WHITE);  
    }
  }
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(30,220);
  M5.Lcd.printf("WAVE");
  M5.Lcd.setCursor(107+38,220);
  M5.Lcd.printf("ARP");
  M5.Lcd.setCursor(213+30,220);
  M5.Lcd.printf("EDIT");
}

void DrawUserWave(void)
{
  M5.Lcd.fillScreen(BLACK);
  int i;
  for(i=0;i<320;i++)
  {
    int h = (int)(120*soundbank[4][i]+120);
    M5.Lcd.drawLine(i, 120, i, h,TFT_YELLOW);
  } 
}

void EditUserWave(void)
{
  int i;
  if(px<nx){
    for(i=0;i<=(nx-px);i++){
      double y = (double)i*(double)(ny-py)/(double)(nx-px)+py;
      soundbank[4][(int)px+i] = (y - 120.0)/120.0;
      M5.Lcd.drawFastVLine((int)px+i,0,240,BLACK);
      M5.Lcd.drawLine((int)px+i, 120, (int)px+i, (int)y,TFT_YELLOW);
    }
  }
  else if(nx<px){
    for(i=0;i<=(px-nx);i++){
      double y = (double)i*(double)(py-ny)/(double)(px-nx)+ny;
      soundbank[4][(int)nx+i] = (y - 120.0)/120.0;
      M5.Lcd.drawFastVLine((int)nx+i,0,240,BLACK);
      M5.Lcd.drawLine((int)nx+i, 120, (int)nx+i, (int)y,TFT_YELLOW);
    }
  }
  else{       
    soundbank[4][(int)nx] = ((double)ny - 120.0)/120.0;
    M5.Lcd.drawFastVLine(nx,0,240,BLACK);
    M5.Lcd.drawLine(nx, 120, nx, ny,TFT_YELLOW);
  }  
  px = nx;
  py = ny;
}

void Synthesize(void)
{
  int i;
  int loop = DATA_SIZE/4;

  for(i=0;i<loop;++i)
  {
    double s1;
    double s2;
    double gate = 1.0;
    
    if(wave1type==5)
      s1 = noiserate*wave1[random(TEMPLATE_SIZE)];
    else
      s1 = wave1[(int)(phase1*(double)TEMPLATE_SIZE)];
    if(wave2type==5)
      s2 = noiserate*wave2[random(TEMPLATE_SIZE)];
    else
      s2 = wave2[(int)(phase2*(double)TEMPLATE_SIZE)];
    
    gate = pattern[arp][(int)(beat*16)];
    
    if(pgate==0 && gate>0.0 && touch)
      decay = 1.0;
    pgate = gate;
    
    //attack = max(attack-Ga/(double)SAMPLING_RATE*2000,0.0);    
    decay = max(decay-Gd/(double)SAMPLING_RATE*50,0.0);   
    
    double sig = (gate*velocity*127.0*(s1+s2)*decay*(1.0-attack));
    delaywave[delayphase] = delayrate*delaywave[delayphase]+sig;
    int isig = delaywave[delayphase];
    delayphase = (delayphase+1)%DELAY_SIZE;
    
    sdata[i*4+0] = (unsigned char)((isig<<8)&0xFF);
    sdata[i*4+1] = (unsigned char)((isig<<0)&0xFF);
    sdata[i*4+2] = (unsigned char)((isig<<8)&0xFF);
    sdata[i*4+3] = (unsigned char)((isig<<0)&0xFF);
   
    phase1 += (double)freq/(double)SAMPLING_RATE;
    phase2 += (double)freq*detune/(double)SAMPLING_RATE;
    beat += BPM/SAMPLING_RATE;
    
    if(phase1 >= 1.0)
      phase1 -= 1.0;
    if(phase2 >= 1.0)
      phase2 -= 1.0;
    if(beat >= 1.0)
      beat -= 1.0;
  }
}

void Tone(void)
{
  size_t bytes_written = 0;
  i2s_write(Speak_I2S_NUMBER, sdata, DATA_SIZE, &bytes_written, portMAX_DELAY);
}

void setup() {
  M5.begin(true, true, true, true);
  InitDisplay();
  InitSpeak();
  SetSoundBank();
  SetWave();
  DrawWave();
}

void loop() {
  TouchPoint_t pos= M5.Touch.getPressPoint();
  //mpu.getGyroData(&gyroX, &gyroY, &gyroZ);
  if(edit<1){
    if(pos.x >=0 && pos.y >=0 && pos.y <=240)
    {      
      int tnum = ((int)(pos.x/9)+1);
//      major penta-tonic code
//      if(tnum%12==1 || tnum%12==3 || tnum%12==5 || tnum%12==10)
//        tnum -= 1;
//      else if(tnum%12==6 || tnum%12==8 || tnum%12==11)
//        tnum += 1;

      //minor penta-tonic code
      if(tnum%12==1 || tnum%12==8)
        tnum -= 1;
      if(tnum%12==2 || tnum%12== 4 || tnum%12== 6 || tnum%12== 9 || tnum%12== 11)
        tnum += 1;
  
      velocity = min(((double)pos.y)/140.0,1.0)*gain;
      Gd = max(((double)pos.y-140)/140.0,0.0);

      freq = hz[tnum];//(double)440*pow(2,tnum/12.0);
      
      if(freq != pfreq || !touch)
      {
        touch = true;
        decay = 1.0;
        attack = 0.0;
        pfreq = freq;
      }
          
      Synthesize();
      Tone();
    }
    else if(pos.x >=0 && pos.y>240)
    {
      if(pos.x < 109) {
        if(!touch)
        {
          wavetype = (wavetype+1)%25;
          SetWave();
          DrawWave();
        }
        touch = true;
      } else if (pos.x > 218) {
        if(!touch)
        {
          edit = 1;
          DrawUserWave();
        }
        touch = true;
        
      } else if (pos.x >= 109 && pos.x <= 218) {
        if(!touch)
        {
          arp = (arp+1)%7;
          touch = true;
          DrawWave();
        }      
      }
    }
    else
    {
      touch = false;
      Gd = 1.0;
      Synthesize();
      Tone();
    }
  }
  else
  {
    M5.Lcd.setCursor(107+30,220);
    M5.Lcd.printf("BACK"); 
    if(pos.x >=0 && pos.y >=0 && pos.y <=240)
    {
      if(!touch)
      {
        px = pos.x;
        py = pos.y;
        touch = true;
      }

      nx = pos.x;
      ny = pos.y;
      EditUserWave();
    }
    else if(pos.x>=0 && pos.y>240)
    {
      if(!touch)
      {
        edit = 0;
        wavetype = 3;
        SetWave();
        DrawWave();
      }
      touch = true;     
    }
    else
    {
      touch = false;
    }
  }
}
