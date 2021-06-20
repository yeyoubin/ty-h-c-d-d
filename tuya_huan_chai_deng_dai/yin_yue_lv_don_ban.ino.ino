#include <TuyaWifi.h>
#include <SoftwareSerial.h>


// Simple demonstration on using an input device to trigger changes on your
// NeoPixels. Wire a momentary push button to connect from ground to a
// digital IO pin. When the button is pressed it will change to a new pixel
// animation. Initial state has all pixels off -- press the button once to
// start the first animation. As written, the button does not interrupt an
// animation in-progress, it works only when idle.

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Digital IO pin connected to the button. This will be driven with a
// pull-up resistor so the switch pulls the pin to ground momentarily.
// On a high -> low transition the button press logic will execute.
#define BUTTON_PIN   4

#define PIXEL_PIN    5  // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 30  // Number of NeoPixels

#include "FastLED.h"

#define sigPin A0                // ESP8266ADC引脚接MAX9814 OUT信号引脚  GAIN�?.3V
#define LED_TYPE WS2812       // LED灯带型号
#define COLOR_ORDER GRB
CRGB leds[PIXEL_COUNT];            // 建立光带leds

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

boolean oldState = HIGH;
int     mode     = 0;    // Currently-active animation mode, 0-9


TuyaWifi my_device;

unsigned char led_state = 0;
/* Connect network button pin */

int key_pin = 0;

//开�?可下发可上报)
//备注:
#define DPID_SWITCH_LED 20
//模式(可下发可上报)
//备注:
#define DPID_WORK_MODE 21
//亮度�?可下发可上报)
//备注:
#define DPID_BRIGHT_VALUE 22
//冷暖�?可下发可上报)
//备注:
#define DPID_TEMP_VALUE 23
//彩光(可下发可上报)
//备注:类型：字符；
//Value: 000011112222�?
//0000：H（色度：0-360�?X0000-0X0168）；
//1111：S (饱和�?-1000, 0X0000-0X03E8）；
//2222：V (明度�?-1000�?X0000-0X03E8)
#define DPID_COLOUR_DATA 24
//场景(可下发可上报)
//备注:类型：字�? 
//Value: 0011223344445555666677778888;
//00：情景号;
//11：单元切换间隔时间（0-100�?
//22：单元变化时间（0-100�?
//33：单元变化模式（0静�?跳变2渐变�?
//4444：H（色度：0-360�?X0000-0X0168�?
//5555：S (饱和�?-1000, 0X0000-0X03E8);
//6666：V (明度�?-1000�?X0000-0X03E8);
//7777：白光亮度（0-1000�?
//8888：色温值（0-1000�?
//注：数字1-8的标号对应有多少单元就有多少�?
#define DPID_SCENE_DATA 25
//倒计时剩余时�?可下发可上报)
//备注:
#define DPID_COUNTDOWN 26
//音乐�?只下�?
//备注:类型：字符串�?
//Value: 011112222333344445555�?
//0�?  变化方式�?表示直接输出�?表示渐变�?
//1111：H（色度：0-360�?X0000-0X0168）；
//2222：S (饱和�?-1000, 0X0000-0X03E8）；
//3333：V (明度�?-1000�?X0000-0X03E8）；
//4444：白光亮度（0-1000）；
//5555：色温值（0-1000�?
#define DPID_MUSIC_DATA 27
//调节(只下�?
//备注:类型：字符串 ;
//Value: 011112222333344445555  ;
//0�?  变化方式�?表示直接输出�?表示渐变;
//1111：H（色度：0-360�?X0000-0X0168�?
//2222：S (饱和�?-1000, 0X0000-0X03E8);
//3333：V (明度�?-1000�?X0000-0X03E8);
//4444：白光亮度（0-1000�?
//5555：色温值（0-1000�?
#define DPID_CONTROL_DATA 28
//入睡(可下发可上报)
//备注:灯光按设定的时间淡出直至熄灭
#define DPID_SLEEP_MODE 31
//唤醒(可下发可上报)
//备注:灯光按设定的时间逐渐淡入直至设定的亮�?
#define DPID_WAKEUP_MODE 32
//断电记忆(可下发可上报)
//备注:通电后，灯亮起的状�?
#define DPID_POWER_MEMORY 33
//勿扰模式(可下发可上报)
//备注:适用经常停电区域，开启通电勿扰，通过APP关灯需连续两次上电才会亮灯
//Value：ABCCDDEEFFGG
//A：版本，初始版本0x00�?
//B：模式，0x00初始默认值�?x01恢复记忆值�?x02用户定制�?
//CC：色�?H�?~360�?
//DD：饱和度 S�?~1000�?
//EE：明�?V�?~1000�?
//FF：亮度，0~1000�?
//GG：色温，0~1000�?
#define DPID_DO_NOT_DISTURB 34
//麦克风音乐律�?可下发可上报)
//备注:类型�? 字符�?
//Value�? AABBCCDDEEFFGGGGHHHHIIIIJJJJKKKKLLLLMMMMNNNN
//AA  版本
//BB  0-关闭�?-打开
//CC  模式编号，自定义�?01开�?
//DD  变换方式�? - 呼吸模式�? -跳变模式 ，�? - 经典模式
//EE  变化速度
//FF  灵敏�?
//GGGG  颜色1-色相饱和�?
//HHHH  颜色2-色相饱和�?
//......
//NNNN  颜色8-色相饱和�?
#define DPID_MIC_MUSIC_DATA 52
//炫彩情景(可下发可上报)
//备注:专门用于幻彩灯带场景
//Value：ABCDEFGHIJJKLLM...
//A：版本号�?
//B：情景模式编号；
//C：变化方式（0-静态�?-渐变�?跳变�?呼吸�?-闪烁�?0-流水�?1-彩虹�?
//D：单元切换间隔时间（0-100�?
//E：单元变化时间（0-100）；
//FGH：设置项�?
//I：亮度（亮度V�?~100）；
//JJ：颜�?（色度H�?-360）；
//K：饱和度1 (饱和度S�?-100)�?
//LL：颜�?（色度H�?-360）；
//M：饱和度2（饱和度S�?~100）；
//注：有多少个颜色单元就有多少组，最多支�?0组；
//每个字母代表一个字�?
#define DPID_DREAMLIGHT_SCENE_MODE 51
//炫彩本地音乐律动(可下发可上报)
//备注:专门用于幻彩灯带本地音乐
//Value：ABCDEFGHIJKKLMMN...
//A：版本号�?
//B：本地麦克风开关（0-关�?-开）；
//C：音乐模式编号；
//D：变化方式；
//E：变化速度�?-100�?
//F：灵敏度(1-100)�?
//GHI：设置项�?
//J：亮度（亮度V�?~100）；
//KK：颜�?（色度H�?-360）；
//L：饱和度1 (饱和度S�?-100)�?
//MM：颜�?（色度H�?-360）；
//N：饱和度2（饱和度S�?~100）；
//注：有多少个颜色单元就有多少组，最多支�?组；
//每个字母代表一个字�?
//#define DPID_DREAMLIGHTMIC_MUSIC_DATA 42
//点数/长度设置(可下发可上报)
//备注:幻彩灯带裁剪之后重新设置长度
#define DPID_LIGHTPIXEL_NUMBER_SET 53



///* Current device DP values */
unsigned char dp_bool_value = 0;
long dp_value_value = 0;
unsigned char dp_enum_value = 0;
unsigned char dp_string_value[21] = {"0"};
uint16_t Hue=0; //HSV
uint8_t Sat=0;
uint8_t Val=0;
uint8_t scene_mode=0;
unsigned char hex[10] = {"0"};
//unsigned char dp_raw_value[8] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
//int dp_fault_value = 0x01;
int sig;//麦克风读取数�?
int pre_si;//指示旧�?
int si;//指示新�?
int sig_min = 1024;//初始读取值最大小值，开机后将重新界�?
int sig_max = 0;
double triger_factor = 1.20;//阈�?过滤噪音 该数值调整范�?-2 数值越高要求声音越大才能触�?
int drop_dot; //初始掉落位置
unsigned long drop_time;//触发掉落时机
int interval = 30; //每次律动变化的过渡时�?
int drop_interval = 100; //掉落点收缩速度
int maxBrightness = 250;
int mid = int(PIXEL_COUNT / 2);
int delta = int(maxBrightness / mid); //渐变�?
unsigned long triger_time = 0;
int randC = 130; //掉落点随机颜色初�?
bool raising = true;

/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type. 
 *                                     dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
*/
unsigned char dp_array[][2] = {
  {DPID_SWITCH_LED, DP_TYPE_BOOL},
  {DPID_WORK_MODE, DP_TYPE_ENUM},
  {DPID_BRIGHT_VALUE, DP_TYPE_VALUE},
  {DPID_TEMP_VALUE, DP_TYPE_VALUE},
  {DPID_COLOUR_DATA, DP_TYPE_STRING},
  {DPID_SCENE_DATA, DP_TYPE_STRING},
  {DPID_COUNTDOWN, DP_TYPE_VALUE},
  {DPID_MUSIC_DATA, DP_TYPE_STRING},
  {DPID_CONTROL_DATA, DP_TYPE_STRING},
  {DPID_SLEEP_MODE, DP_TYPE_RAW},
  {DPID_WAKEUP_MODE, DP_TYPE_RAW},
  {DPID_POWER_MEMORY, DP_TYPE_RAW},
  {DPID_DO_NOT_DISTURB, DP_TYPE_BOOL},
  {DPID_MIC_MUSIC_DATA, DP_TYPE_STRING},
  {DPID_DREAMLIGHT_SCENE_MODE, DP_TYPE_RAW},
  
  {DPID_LIGHTPIXEL_NUMBER_SET, DP_TYPE_VALUE},
};

unsigned char pid[] = {"yburybqb2agufbw7"};//*********处替换成涂鸦IoT平台自己创建的产品的PID
unsigned char mcu_ver[] = {"1.0.0"};

unsigned long last_time = 0;
SoftwareSerial DebugSerial(15,13);
void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'

   // 麦克风初始化
  LEDS.addLeds<LED_TYPE, PIXEL_PIN, COLOR_ORDER>(leds, PIXEL_COUNT);
  // .setCorrection(TypicalLEDStrip);//如果灯带未响应，请设�?
  FastLED.setBrightness(255);                            // 设置光带亮度
  pinMode(sigPin, INPUT);

DebugSerial.begin(9600);

  Serial.begin(9600);
 //Initialize led port, turn off led.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
//Initialize networking keys.
  pinMode(key_pin, INPUT_PULLUP);

 //incoming all DPs and their types array, DP numbers
   //Enter the PID and MCU software version
    my_device.init(pid, mcu_ver);
    my_device.set_dp_cmd_total(dp_array, 16);
    //register DP download processing callback function
    my_device.dp_process_func_register(dp_process);
    //register upload all DP callback function
    my_device.dp_update_all_func_register(dp_update_all);

    last_time = millis();
  
}

void loop() {

  //Enter the connection network mode when Pin7 is pressed.
  if (digitalRead(key_pin) == LOW) {
    delay(80);
    if (digitalRead(key_pin) == LOW) {
      my_device.mcu_set_wifi_mode(SMART_CONFIG);

      
    }
  }
my_device.uart_service();


  /* LED blinks when network is being connected */
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW)) {
    if (millis()- last_time >= 500) {
      last_time = millis();

      if (led_state == LOW) {
        led_state = HIGH;
      } else {
        led_state = LOW;
      }
      digitalWrite(LED_BUILTIN, led_state);
    }
  }

  
  // Get current button state.
  boolean newState = digitalRead(BUTTON_PIN);

  // Check if state changed from high to low (button press).
  if((newState == LOW) && (oldState == HIGH)) {
    // Short delay to debounce button.
    delay(20);
    // Check if button is still low after debounce.
    newState = digitalRead(BUTTON_PIN);
    if(newState == LOW) {      // Yes, still low
      if(++mode > 8) mode = 0; // Advance to next mode, wrap around after #8
      switch(mode) {           // Start the new animation...
        case 0:
          colorWipe(strip.Color(  255,   0,   0), 50);    // Black/off
          colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
          break;
        case 1:
          colorWipe(strip.Color(255,   0,   100), 50);    // Red
           colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
          break;
        case 2:
          colorWipe(strip.Color(  50, 255,  100), 50);    // Green
           colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
          break;
        case 3:
          colorWipe(strip.Color(  100,   200, 255), 50);    // Blue
           colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
          break;
        case 4:
          theaterChase(strip.Color(127, 127, 127), 50); // White
           colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
          break;
        case 5:
          theaterChase(strip.Color(127,   0,   0), 50); // Red
           colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
          break;
        case 6:
          theaterChase(strip.Color(  0,   0, 127), 50); // BluemusicV();
          break;
        case 7:
          rainbow(10);
          break;
          case 8:
          theaterChaseRainbow(50);
          break;
           case 9:
          theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness
          theaterChase(strip.Color(127,   0,   0), 50); // Red, half brightness
          rainbow(10);             // Flowing rainbow cycle along the whole strip
          theaterChaseRainbow(50);
          break;
      }
    }
  }

  // Set the last-read button state to the old state.
  oldState = newState;
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 3 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 3*65536. Adding 256 to firstPixelHue each time
  // means we'll make 3*65536/256 = 768 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 3*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}

void musicV()//麦克值风处理
{
  //计算环境读取最大最小值，所以在上电后循环几次后达到平稳的触发值�?
  sig = analogRead(sigPin);

  if (sig > sig_max)//
  {
    sig_max = sig;
  }
  if (sig < sig_min)
  {
    sig_min = sig;
  }

  int mean = (sig_min + sig_max) / 2;

  int _sig = abs(sig - mean ) + mean;


  _sig = constrain(_sig, int(mean * triger_factor), sig_max);
  //Serial.println(_sig);

  si = map(_sig - int(mean * triger_factor), int(mean * triger_factor) - int(mean * triger_factor), int(sig_max) - int(mean * triger_factor), 0, mid - 2);
  si = constrain(si, 0, mid - 2);
  if (si == mid - 2) {
    randC = random(0, 255);

    pre_si = 0;
  }
  Serial.println(randC);
  //===================================================================================================== PART1
  if (si > pre_si)
  {
    if (si > drop_dot)
    {
      if (si * 1.6 < mid - 1) {
        drop_dot = int(si * 1.6);
      }
      else {
        drop_dot = mid - 3;
      }




    }
    for (int j = pre_si - 1; j < si + 1; j++)
    {
      while ( millis() - triger_time < int(interval / (si - pre_si) + 2))
      {

        if ((drop_dot > j + 3 ) && (millis() - drop_time > drop_interval))
        {
          leds [mid + drop_dot + 1] = CRGB::Black;
          leds [mid - 1 - drop_dot - 1] = CRGB::Black;
          leds[mid + drop_dot] = CHSV(int(255 - randC), 255, maxBrightness);
          leds[mid - 1 - drop_dot] = CHSV(int(255 - randC), 255, maxBrightness);

          FastLED.show();
          drop_dot--;
          drop_time = millis();
        }

        leds [mid + j] = CHSV(delta * j, 255, maxBrightness - delta * j);
        leds [mid - 1 - j] = CHSV(delta * j, 255, maxBrightness - delta * j);
        //leds [mid + j] = CHSV(randC, 255, 255 - delta * j);
        //leds [mid - 1 - j] = CHSV(randC, 255, 255 - delta * j);


      }
      FastLED.show();
      triger_time = millis();
    }
  }
  // --------------------------------------------------------------------------------------------------
  //===================================================================================================== PART2
  if (si < pre_si)
  {
    leds [mid + drop_dot + 1] = CRGB::Black;
    leds [mid - 1 - drop_dot - 1] = CRGB::Black;
    leds[mid + drop_dot] = CHSV(int(255 - randC), 255, maxBrightness);
    leds[mid - 1 - drop_dot] = CHSV(int(255 - randC), 255, maxBrightness);
    FastLED.show();
    drop_dot--;
    drop_time = millis();


    for (int k = pre_si + 1; k > si - 1; k--)
    {
      while ( millis() - triger_time < int(interval / (pre_si - si) + 2))
      {
        if ((drop_dot > k + 3 ) && (millis() - drop_time > drop_interval))
        {
          leds [mid + drop_dot + 1] = CRGB::Black;
          leds [mid - 1 - drop_dot - 1] = CRGB::Black;
          leds[mid + drop_dot] = CHSV(int(255 - randC), 255, maxBrightness);
          leds[mid - 1 - drop_dot] = CHSV(int(255 - randC), 255, maxBrightness);
          FastLED.show();
          drop_dot--;
          drop_time = millis();

        }


        leds[mid + k] = CRGB::Black;

        leds[mid - 1 - k] = CRGB::Black;
      }
      FastLED.show();
      triger_time = millis();

    }
  }
  // --------------------------------------------------------------------------------------------------

  if (si == 0 && pre_si == 0 && drop_dot > -1 && (millis() - drop_time > drop_interval))
  {
    leds [mid + drop_dot + 1] = CRGB::Black;
    leds [mid - 1 - drop_dot - 1] = CRGB::Black;

    leds[mid + drop_dot] = CHSV(int(255 - randC), 255, maxBrightness);
    leds[mid - 1 - drop_dot] = CHSV(int(255 - randC), 255, maxBrightness);
    if (drop_dot == 0)
    {
      leds [mid + drop_dot] = CRGB::Black;
      leds [mid - 1 - drop_dot] = CRGB::Black;
    }
    FastLED.show();
    drop_dot--;
    drop_time = millis();

  }
  pre_si = si;
}

/**
 * @description: DP download callback function.
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)
{
  switch(dpid) {
    case DPID_SWITCH_LED:
      dp_bool_value = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      if (dp_bool_value) {
        //Turn on
        colorfill (strip.Color(  0, 255,  0)); //上一次状�?
      } else {
        //Turn off
        colorfill (strip.Color(  0, 0,   0));
      }
      //Status changes should be reported.
      my_device.mcu_dp_update(dpid, value, length);
    break;
    
    case DPID_WORK_MODE:
    colorfill (strip.Color( 255, 255,  0));
    dp_enum_value  = my_device.mcu_get_dp_download_data(dpid, value, length); /* Get the value of the down DP command */
      switch(dp_enum_value){
        case 0: // white mode
          colorfill (strip.Color(  255, 255,  255));
        break;
         case 1:
          theaterChase(strip.Color(  0,   0, 127), 50); // Blue
          break;
        case 2:
          rainbow(10);
          break;
          case 3:
          theaterChaseRainbow(50);
          break;

      }
      //Status changes should be reported.
      my_device.mcu_dp_update(dpid, value, length);
    break;
    
    case DPID_COUNTDOWN:  //倒计�?
     colorfill (strip.Color( 255, 0,  0));
     colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
      my_device.mcu_dp_update(dpid, value, length);
    break;


    case DPID_MUSIC_DATA: //音乐律动  DPID_MIC_MUSIC_DATA 
     colour_data_control(value, length);
     my_device.mcu_dp_update(dpid, value, length);
        
    break;

    case DPID_MIC_MUSIC_DATA: //麦克风音乐律�? DPID_MIC_MUSIC_DATA 就这里APP下发指令后让musicV()循环运行起来
    
  
     for(int i=1;i<1000;i++)
    {
     musicV();
     
    }
     my_device.mcu_dp_update(dpid, value, length);
        
    break;
    
     case DPID_DREAMLIGHT_SCENE_MODE: //炫彩情景
     my_device.mcu_dp_update(DPID_DREAMLIGHT_SCENE_MODE, value, length);

     scene_mode=value[1];
    
     switch(scene_mode){
       case 0:
          colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
          break;
        case 1:
          colorWipe(strip.Color(255,   0,   0), 50);    // Red
          break;
        case 2:
        
          colorWipe(strip.Color(  0, 255,   0), 50);    // Green
          break;
        case 3:
          colorWipe(strip.Color(  0,   0, 255), 50);    // Blue
          break;
        case 4:
          theaterChase(strip.Color(127, 127, 127), 50); // White
          break;
        case 5:
          theaterChase(strip.Color(127,   0,   0), 50); // Red
          break;
        case 6:
          theaterChase(strip.Color(  0,   0, 127), 50); // Blue
          break;
        case 7:
          rainbow(10);
          break;
          case 8:
          theaterChaseRainbow(50);
          break;
          case 9:
          rainbow(10);
          theaterChaseRainbow(50);
           // Fill along the length of the strip in various colors...
          colorWipe(strip.Color(255,   0,   0), 50); // Red
          colorWipe(strip.Color(  0, 255,   0), 50); // Green
          colorWipe(strip.Color(  0,   0, 255), 50); // Blue

          // Do a theater marquee effect in various colors...
          theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness
          theaterChase(strip.Color(127,   0,   0), 50); // Red, half brightness
          theaterChase(strip.Color(  0,   0, 127), 50); // Blue, half brightness

          break;
          
     }
     
      break;

      case DPID_LIGHTPIXEL_NUMBER_SET: //长度设置
      my_device.mcu_dp_update(dpid, value, length);
      break;
    default:break;
  }
  return SUCCESS;
}

/**
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
  my_device.mcu_dp_update(DPID_SWITCH_LED, led_state, 1);
}



//拓展
void colorfill(uint32_t color) {
 strip.fill(color,0,PIXEL_COUNT);
    strip.show();                          //  Update strip to match   
  
}





 void colour_data_control( const unsigned char value[], u16 length)
 {
   u8 string_data[13];
    u16 h, s, v;
    u8 r, g, b;
    u16 hue;
    u8 sat,val;

    u32 c=0;
  
    string_data[0] = value[0]; //渐变、直接输�?
    string_data[1] = value[1];
    string_data[2] = value[2];
    string_data[3] = value[3];
    string_data[4] = value[4];
    string_data[5] = value[5];
    string_data[6] = value[6];
    string_data[7] = value[7];
    string_data[8] = value[8];
    string_data[9] = value[9];
    string_data[10] = value[10];
    string_data[11] = value[11];
    string_data[12] = value[12];
    
  
    h = __str2short(__asc2hex(string_data[1]), __asc2hex(string_data[2]), __asc2hex(string_data[3]), __asc2hex(string_data[4]));
    s = __str2short(__asc2hex(string_data[5]), __asc2hex(string_data[6]), __asc2hex(string_data[7]), __asc2hex(string_data[8]));
    v = __str2short(__asc2hex(string_data[9]), __asc2hex(string_data[10]), __asc2hex(string_data[11]), __asc2hex(string_data[12]));

    

    // if (v <= 10) {
    //     v = 0;
    // } else {
    //     v = color_val_lmt_get(v);
    // }
    
    //hsv2rgb((float)h, (float)s / 1000.0, (float)v / 1000.0, &r , &g, &b);

    // c= r<<16|g<<8|b;
  hue=h*182;
  sat=s/4;
  val=v/4;
    c = strip.gamma32(strip.ColorHSV(hue,sat,val)); // hue -> RGB
    DebugSerial.println(hue);
    DebugSerial.println(sat);
    DebugSerial.println(val);
 
    
    strip.fill(c,0,PIXEL_COUNT);
    
    strip.show(); // Update strip with new contents

    //tuya_light_gamma_adjust(r, g, b, &mcu_default_color.red, &mcu_default_color.green, &mcu_default_color.blue);
  
    //printf("r=%d,g=%d,b=%d\r\n", mcu_default_color.red, mcu_default_color.green, mcu_default_color.blue);
    //rgb_init(mcu_default_color.red, mcu_default_color.green, mcu_default_color.blue);
 }

/**
 * @brief  str to short
 * @param[in] {a} Single Point
 * @param[in] {b} Single Point
 * @param[in] {c} Single Point
 * @param[in] {d} Single Point
 * @return Integrated value
 * @note   Null
 */
u32 __str2short(u32 a, u32 b, u32 c, u32 d)
{
    return (a << 12) | (b << 8) | (c << 4) | (d & 0xf);
}

/**
  * @brief ASCALL to Hex
  * @param[in] {asccode} 当前ASCALL�?
  * @return Corresponding value
  * @retval None
  */
u8 __asc2hex(u8 asccode)
{
    u8 ret;
    
    if ('0' <= asccode && asccode <= '9')
        ret = asccode - '0';
    else if ('a' <= asccode && asccode <= 'f')
        ret = asccode - 'a' + 10;
    else if ('A' <= asccode && asccode <= 'F')
        ret = asccode - 'A' + 10;
    else
        ret = 0;
    
    return ret;
}

/**
  * @brief Normalized
  * @param[in] {dp_val} dp value
  * @return result
  * @retval None
  */
u16 color_val_lmt_get(u16 dp_val)
{
    u16 max = 255 * 100 / 100;
    u16 min = 255 * 1 / 100;
    
    return ((dp_val - 10) * (max - min) / (1000 - 10) + min);
}

/**
  * @brief hsv to rgb
  * @param[in] {h} tone
  * @param[in] {s} saturation
  * @param[in] {v} Lightness
  * @param[out] {color_r} red
  * @param[out] {color_g} green
  * @param[out] {color_b} blue
  * @retval None
  */
void hsv2rgb(float h, float s, float v, u8 *color_r, u8 *color_g, u8 *color_b)
{
    float h60, f;
    u32 h60f, hi;
  
    h60 = h / 60.0;
    h60f = h / 60;
  
    hi = ( signed int)h60f % 6;
    f = h60 - h60f;
  
    float p, q, t;
  
    p = v * (1 - s);
    q = v * (1 - f * s);
    t = v * (1 - (1 - f) * s);
  
    float r, g, b;
  
    r = g = b = 0;
    if (hi == 0) {
        r = v;          g = t;        b = p;
    } else if (hi == 1) {
        r = q;          g = v;        b = p;
    } else if (hi == 2) {
        r = p;          g = v;        b = t;
    } else if (hi == 3) {
        r = p;          g = q;        b = v;
    } else if (hi == 4) {
        r = t;          g = p;        b = v;
    } else if (hi == 5) {
        r = v;          g = p;        b = q;
    }
  
    DebugSerial.println(r);
    DebugSerial.println(g);
    DebugSerial.println(b);
    r = (r * (float)255);
    g = (g * (float)255);
    b = (b * (float)255);
  
    *color_r = r;
    *color_g = g;
    *color_b = b;
    
    // r *= 100;
    // g *= 100;
    // b *= 100;
  
    // *color_r = (r + 50) / 100;
    // *color_g = (g + 50) / 100;
    // *color_b = (b + 50) / 100;
}
