#include <iostream>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <queue>
#include <MQTTClient.h>
#include <time.h>

using namespace std;

FILE *fp;
// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static const int CHANNEL = 1;

#define bit(x) (1ul << x)

#define I_Am_SX1276 0x12

#define   SX1276_FIFO                       0x00
#define   SX1276_OP_MODE                    0x01
#define   SX1276_RESERVED1                  0x02
#define   SX1276_RESERVED2                  0x03
#define   SX1276_RESERVED3                  0x04
#define   SX1276_RESERVED4                  0x05
#define   SX1276_FRF_MSB                    0x06
#define   SX1276_FRF_MID                    0x07
#define   SX1276_FRF_LSB                    0x08
#define   SX1276_PA_CONFIG                  0x09
#define   SX1276_PA_RAMP                    0x0a
#define   SX1276_OCP                        0x0b
#define   SX1276_LNA                        0x0c
#define   SX1276_FIFO_ADDR_PTR              0x0d
#define   SX1276_FIFO_TX_BASE_ADDR          0x0e
#define   SX1276_FIFO_RX_BASE_ADDR          0x0f
#define   SX1276_FIFO_RX_CURRENT_ADDR       0x10
#define   SX1276_IRQ_FLAGS_MASK             0x11
#define   SX1276_IRQ_FLAGS                  0x12
#define   SX1276_RX_NB_BYTES                0x13
#define   SX1276_RX_HEADER_CNT_VALUE_MSB    0x14
#define   SX1276_RX_HEADER_CNT_VALUE_LSB    0x15
#define   SX1276_RX_PACKET_CNT_VALUE_MSB    0x16
#define   SX1276_RX_PACKET_CNT_VALUE_LSB    0x17
#define   SX1276_MODEM_STAT                 0x18
#define   SX1276_PKT_SNR_VALUE              0x19
#define   SX1276_PKT_RSSI_VALUE             0x1a
#define   SX1276_RSSI_VALUE                 0x1b
#define   SX1276_HOP_CHANNEL                0x1c
#define   SX1276_MODEM_CONFIG1              0x1d
#define   SX1276_MODEM_CONFIG2              0x1e
#define   SX1276_SYMB_TIMEOUT_LSB           0x1f
#define   SX1276_PREAMBLE_MSB               0x20
#define   SX1276_PREAMBLE_LSB               0x21
#define   SX1276_PAYLOAD_LENGTH             0x22
#define   SX1276_MAX_PAYLOAD_LENGTH         0x23
#define   SX1276_HOP_PERIOD                 0x24
#define   SX1276_FIFO_RX_BYTE_ADDR          0x25
#define   SX1276_MODEM_CONFIG3              0x26
#define   SX1276_DIO_MAPPING1               0x40
#define   SX1276_DIO_MAPPING2               0x41
#define   SX1276_VERSION                    0x42
#define   SX1276_TCXO                       0x4b
#define   SX1276_PA_DAC                     0x4d
#define   SX1276_FORMER_TEMP                0x5b
#define   SX1276_AGC_REF                    0x61
#define   SX1276_AGC_THRESH1                0x62
#define   SX1276_AGC_THRESH2                0x63
#define   SX1276_AGC_THRESH3                0x64

#define   SX1276_PA_DAC_DISABLE             0x04
#define   SX1276_PA_DAC_ENABLE              0x07

#define   SX1276_WRITE                      0x80

#define INT_PIN (4)

enum ModeValue { SLEEP, STDBY, FSTX, TX, FSRX, RXCONT, RXSINGLE, CAD };
enum AccessSel { LoRa, FSK };
enum ModemConfigRange { Bw125Cr45Sf128, Bw500Cr45Sf128, Bw31_25Cr48Sf512, Bw125Cr48Sf4096 };

enum BW {BW7_8kHz, BW10_4kHz, BW15_6kHz, BW20_8kHz, BW31_25kHz, BW41_7kHz, BW62_5kHz, BW125kHz, BW250kHz, BW500kHz};

enum CRC_Stance { CRC_Disable, CRC_Enable };

/*
   ModemConfig : aliased constant ModemConfigArrayT :=
     (((BW => BW125kHz, others => <>),
       (RxCrc => CRC_Enable, others => <>),
       (others => <>)), -- Bw125Cr45Sf128 (the chip default)
      ((BW => BW500kHz, others => <>),
       (RxCrc => CRC_Enable, others => <>),
       (others => <>)), --  Bw500Cr45Sf128
      ((BW => BW31_25kHz, Coding_Rate => Coding_Rate_4_8, others => <>),
       (SpreadFactor => SF512, RxCrc => CRC_Enable, others => <>),
       (others => <>)), --  Bw31_25Cr48Sf512
      ((BW => BW125kHz, Coding_Rate => Coding_Rate_4_8, others => <>),
       (SpreadFactor => SF4096, RxCrc => CRC_Enable, others => <>),
       (others => <>)));  --  Bw125Cr48Sf4096

*/
typedef struct OpMode_Register_t {
  uint8_t   Mode  	    : 3;
  uint8_t   LowFreqModeOn   : 1;
  uint8_t   Reserved        : 2;
  uint8_t   AccessSharedReg : 1;
  uint8_t   LongRangeMode   : 1;
} OpMode_Register;

ModeValue CurrentMode = SLEEP;

enum Coding_Rates { Coding_Rate_4_5=1, Coding_Rate_4_6, Coding_Rate_4_7, Coding_Rate_4_8 };

enum IrqFlags { CadDetected = 0, FhssChangeChannel, CadDone, TxDone, ValidHeader, PayloadCrcError, RxDone, RxTimeout };

typedef struct Config1_t {
  uint8_t   ImplicitHeaderModeOn : 1;
  uint8_t   CodingRate           : 3;
  uint8_t   Bw 	                 : 4;
} Config1;

typedef struct Config2_t {
  uint8_t   SymbTimeout        : 2;
  uint8_t   RxPayloadCrcOn     : 1;
  uint8_t   TxContinousMode    : 1;
  uint8_t   SpreadingFactor    : 4;
} Config2;

typedef struct Config3_t {
  uint8_t   Reserved             : 2;
  uint8_t   AgcAutoOn            : 1;
  uint8_t   LowDataRateOptimize  : 1;
  uint8_t   Unused               : 4;
} Config3;

typedef struct Pa_Config_t {
  uint8_t   OutputPower          : 4;
  uint8_t   MaxPower             : 3;
  uint8_t   PaSelect             : 1;
  uint8_t   Unused               : 4;
} Pa_Config;

void SetDIOMapping1(uint8_t map);

uint8_t ReadReg(uint8_t reg)
{
  uint8_t buffer[2];
  buffer[0] = reg;
  buffer[1] = 0;
  wiringPiSPIDataRW(CHANNEL, buffer, 2);
  return buffer[1];
}

void WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t buffer[2];
  buffer[0] = reg | SX1276_WRITE;
  buffer[1] = value;
  wiringPiSPIDataRW(CHANNEL, buffer, 2);
}

uint8_t Device_Id()
{
  return ReadReg(SX1276_VERSION);
}
void SetLoraMode()
{
  union {
    OpMode_Register r;
    uint8_t         b;
  } u;
  uint8_t buffer[2];

  u.r.Mode = SLEEP;
  u.r.LongRangeMode = 1;
  u.r.LowFreqModeOn = 0;
  u.r.AccessSharedReg = 0;

  WriteReg(SX1276_OP_MODE, u.b);
  fprintf(fp, "SetLoraMode %02x\n", ReadReg(SX1276_OP_MODE));
}
void SetupFifo()
{
  uint8_t buffer[2];
  WriteReg(SX1276_FIFO_TX_BASE_ADDR, 0);
  WriteReg(SX1276_FIFO_RX_BASE_ADDR, 0);
  fprintf(fp, "SetupFifo %x %x\n", ReadReg(SX1276_FIFO_TX_BASE_ADDR), ReadReg(SX1276_FIFO_RX_BASE_ADDR));
}
void SetIdleMode()
{
  union {
    OpMode_Register r;
    uint8_t         b;
  } u;
  uint8_t buffer[2];

  u.r.Mode = STDBY;
  u.r.LongRangeMode = 1;
  u.r.LowFreqModeOn = 0;
  u.r.AccessSharedReg = 0;

  WriteReg(SX1276_OP_MODE, u.b);
  fprintf(fp, "SetIdleMode %02x\n", ReadReg(SX1276_OP_MODE));
}
void SetRxMode()
{
  union {
    OpMode_Register r;
    uint8_t         b;
  } u;
  uint8_t buffer[2];

  u.r.Mode = RXCONT;
  u.r.LongRangeMode = 1;
  u.r.LowFreqModeOn = 0;
  u.r.AccessSharedReg = 0;

  WriteReg(SX1276_OP_MODE, u.b);
  fprintf(fp, "SetRxMode %02x\n", ReadReg(SX1276_OP_MODE));
  SetDIOMapping1(0);
}

uint8_t GetMode()
{
  return ReadReg(SX1276_OP_MODE);
}

void InitModem(uint8_t bw)
{
  union {
    Config1 r1;
    Config2 r2;
    Config3 r3;
    uint8_t  b;
  } u;
  u.b = 0;
  u.r1.Bw = BW125kHz;
  u.r1.CodingRate = Coding_Rate_4_5;
  WriteReg(SX1276_MODEM_CONFIG1, u.b);
  fprintf(fp, "c1 %02x\n", ReadReg(SX1276_MODEM_CONFIG1));

  u.b = 0;
  u.r2.SpreadingFactor = 7;
  u.r2.TxContinousMode = 0;
  u.r2.RxPayloadCrcOn = 1;
  u.r2.SymbTimeout = 0;
  WriteReg(SX1276_MODEM_CONFIG2, u.b);
  fprintf(fp, "c2 %02x\n", ReadReg(SX1276_MODEM_CONFIG2));

  u.b = 0;
  u.r3.LowDataRateOptimize = 0;
  u.r3.AgcAutoOn = 0;
  WriteReg(SX1276_MODEM_CONFIG3, u.b);
  fprintf(fp, "c3 %02x\n", ReadReg(SX1276_MODEM_CONFIG3));
  fprintf(fp, "InitModem done\n");
}
void SetPreambleLength(uint16_t Len)
{
  union {
    uint16_t Len;
    uint8_t  b[2];
  } u;
  u.Len = Len;

  WriteReg(SX1276_PREAMBLE_MSB, u.b[1]);
  fprintf(fp, "SetPreambleLenth MSB %02x\n", ReadReg(SX1276_PREAMBLE_MSB));

  WriteReg(SX1276_PREAMBLE_LSB, u.b[0]);
  fprintf(fp, "SetPreambleLenth LSB %02x\n", ReadReg(SX1276_PREAMBLE_LSB));
}
void SetFrequency(float f)
{
  union {
    uint64_t freq;
    uint8_t b[8];
  } u;
  uint8_t buffer[2];
  f *= 1E6;
  u.freq = (uint64_t)f;
  u.freq <<= 19;
  u.freq /= 32000000;

  WriteReg(SX1276_FRF_MSB, u.b[2]);
  fprintf(fp, "SetFreqency MSB %02x\n", ReadReg(SX1276_FRF_MSB));
  WriteReg(SX1276_FRF_MID, u.b[1]);
  fprintf(fp, "SetFreqency MID %02x\n", ReadReg(SX1276_FRF_MID));
  WriteReg(SX1276_FRF_LSB, u.b[0]);
  fprintf(fp, "SetFreqency LSB %02x\n", ReadReg(SX1276_FRF_LSB));
  fprintf(fp, "SetFreqency done\n");
}
void SetTxPower(uint8_t db, uint8_t rfo)
{
  uint8_t buffer[2];
  union {
    Pa_Config   c;
    uint8_t     b;
  } u;
  if(rfo) {
    fprintf(fp, "rfo set w/o handler\n");
    exit(1);
  } else { // PA boost
    if(db > 20) {
      WriteReg(SX1276_PA_DAC, SX1276_PA_DAC_ENABLE);
      fprintf(fp, "dac enable %02x\n", ReadReg(SX1276_PA_DAC));
      db -= 3;
    } else {
      WriteReg(SX1276_PA_DAC, SX1276_PA_DAC_DISABLE);
      fprintf(fp, "dac disable %02x\n", ReadReg(SX1276_PA_DAC));
    }
    // The RadioHead lib states 20db is a better choice than 17db in the datasheet.
    // PA := (PA_Select => True,
    // Output_Power => UInt4 (Local_Db - 5),
    // others => <>);
    u.c.OutputPower = db;
    u.c.MaxPower = 4;
    u.c.PaSelect = 1;
    WriteReg(SX1276_PA_CONFIG, u.b);
    fprintf(fp, "Paconfig %02x\n", ReadReg(SX1276_PA_CONFIG));
  }
}
  //      Write (This.Port, SX1276_DIO_MAPPING1, 16#00#); --  Interrupt on RxDone
void SetDIOMapping1(uint8_t map)
{
  WriteReg(SX1276_DIO_MAPPING1, map);
  fprintf(fp, "SetDIOMapping1 %02x\n", ReadReg(SX1276_DIO_MAPPING1));
}
void Init()
{
  if(Device_Id() != I_Am_SX1276) {
    fprintf(fp, "Device not found\n");
    exit(1);
  }
  fprintf(fp, "SX1276 detected\n");

  SetLoraMode();
  SetupFifo();
  SetIdleMode();
  InitModem(Bw125Cr45Sf128);
  SetPreambleLength (8);
  SetFrequency(915.0);
  SetTxPower (13, 0);
  SetDIOMapping1(0);
  SetRxMode();
}
typedef struct {
  uint8_t    len;
  uint8_t    msg[256];
} msg_t;
queue<msg_t *> msg_q;
sem_t rx_go;
sem_t mqtt_go;
int connlost_p = 1;
void int_handler()
{
   uint8_t irq;
   //   fprintf(fp, "int seen\n");
   irq = ReadReg(SX1276_IRQ_FLAGS);
   if(irq & bit(RxDone)) {
     //     fprintf(fp, "buf %02x\n", irq);
     sem_post(&rx_go);
   }
   WriteReg(SX1276_IRQ_FLAGS, irq);
}
void *rx_thread(void *arg)
{
  for(;;) {
    uint8_t pkt[128];
    uint8_t len;
    uint8_t rssi;
    int  snr;
    msg_t *message;
    time_t timer;
    char buffer[26];
    struct tm* tm_info;
    sem_wait(&rx_go);
    time(&timer);
    tm_info = localtime(&timer);
    strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
    fprintf(fp, "%s\n", buffer);
    //    fprintf(fp, "rx semaphore seen\n");
    rssi = ReadReg(SX1276_PKT_RSSI_VALUE);
    
    fprintf(fp, "RSSI %d\n", -157 + rssi);
    snr = (int8_t)ReadReg(SX1276_PKT_SNR_VALUE);
    fprintf(fp, "SNR %f\n", snr * 0.25);
    // Extract the pkt
    len = ReadReg(SX1276_RX_NB_BYTES);
    if(len) {
      WriteReg(SX1276_FIFO_ADDR_PTR, ReadReg(SX1276_FIFO_RX_CURRENT_ADDR));
      for(int i=0; i < len;i++) {
	pkt[i] = ReadReg(SX1276_FIFO);
      }
      message = (msg_t *)calloc(1, sizeof(msg_t));
      message->len = len;
      memcpy(message->msg, &pkt, len);
      msg_q.push(message);
      sem_post(&mqtt_go);
    }
  }
  return NULL;
}
const uint8_t g_crc8_table[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
    0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
    0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
    0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
    0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
    0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
    0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
    0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
    0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
    0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
    0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

void
update_crc8(uint8_t *crc, uint8_t m)
{
    uint8_t idx = ((*crc) ^ m) & 0xff;
    *crc = g_crc8_table[idx];
    *crc &= 0xFFU;
}

enum Door_States { G1_OPEN=1, G1_CLOSED, UNUSED1, FD_OPEN, FD_CLOSED, MBOX_CHANGE };
//#define CLIENTID    "paho_c_pub"
#define QOS         1
#define TIMEOUT     10000L

int delivered_p = 0;
void delivered(void *context, MQTTClient_deliveryToken dt) {
  //    fprintf(fp, "Message with token value %d delivery confirmed\n", dt);
  delivered_p = 1;
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    fprintf(fp, "Message arrived\n");
    fprintf(fp, "     topic: %s\n", topicName);
    fprintf(fp, "   message: %.*s\n", message->payloadlen, (char*)message->payload);
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void connlost(void *context, char *cause) {
  //    fprintf(fp, "\nConnection lost\n");
  // fprintf(fp, "     cause: %s\n", cause);
  connlost_p = 1;
}


void *mqtt_thread(void *arg)
{
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;
    const char *g1="g1/state";
    const char *fd="fd/state";
    const char *mbox="mbox/state";
    const char *ping="ping/ping";
    
  
  for(;;) {
    msg_t *message;
    uint8_t crc;
    uint8_t *p;
    const char *topic;
    const char *msg;

    if(connlost_p) {
      MQTTClient_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);

      conn_opts.keepAliveInterval = 20;
      conn_opts.cleansession = 1;
      conn_opts.username = USERNAME;
      conn_opts.password = PASSWORD;
    
      MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered);

      if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        fprintf(fp, "Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
      }
      connlost_p = 0;
    }
      
    sem_wait(&mqtt_go);
    //    fprintf(fp, "mqtt semaphore seen\n");
    message = msg_q.front();
    fprintf(fp, "len %d\n", message->len);
    crc = 0xff;
    p = message->msg;
    for(int i=0;i < message->len;i++) {
      fprintf(fp, "%02x ", p[i]);
      if(i == message->len-1) {
	//	fprintf(fp, "crc %02x\n", crc);
	if(crc == p[i]) {
	  // send mqtt message
	  // 1) was it for us?
	  if(p[0] != 0) { // 0 is us
	    break;
	  }
	  // Its for us
	  // What is it?
	  switch(p[2]) {
	  case G1_OPEN:
	    topic = g1;
	    msg = "open";
	    break;
	  case G1_CLOSED:
	    topic = g1;
	    msg = "closed";
	    break;
	  case FD_OPEN:
	    topic = fd;
	    msg = "open";
	    break;
	  case FD_CLOSED:
	    topic = fd;
	    msg = "closed";
	    break;
	  case MBOX_CHANGE:
	    topic = mbox;
	    msg = "door";
	    break;
	  case 0xFF:
	    topic = ping;
	    msg = "ping";
	    //	    fprintf(fp, "ping\n");
	    break;
	  default:
	    topic = 0;
	    fprintf(fp, "p[2] unknown %02x\n", p[2]);
	    break;
	  }
	  if(topic) {
	    MQTTClient_deliveryToken deliveredtoken;
	    MQTTClient_message pubmsg = MQTTClient_message_initializer;
	    pubmsg.payload = (void *)msg;
	    pubmsg.payloadlen = strlen(msg);
	    pubmsg.qos = QOS;
	    pubmsg.retained = 0;
	    MQTTClient_publishMessage(client, topic, &pubmsg, &deliveredtoken);
	    //	    fprintf(fp, "Publishing message: %s\n", msg);
	  }
	}
      } else {
	update_crc8(&crc, message->msg[i]);
      }
    }
    fprintf(fp, "\n");
    fflush(fp);
    msg_q.pop();
    free(message);
  }
  return NULL;
}
int main()
{
   int fd, result;
   unsigned char buffer[100];
   pthread_t rx_thread_ptr;
   pthread_t mqtt_thread_ptr;
   fp = fopen("/var/log/lora", "a+");
   if(!fp) {
     exit(1);
   }
   fseek(fp, 0L, SEEK_END);
   if(sem_init(&rx_go, 0, 0) != 0) {
     fprintf(fp, "Cannot init rx semaphore\n");
     exit(1);
   }
   if(sem_init(&mqtt_go, 0, 0) != 0) {
     fprintf(fp, "Cannot init mqtt semaphore\n");
     exit(1);
   }
   
   if(wiringPiSetup() < 0) {
     fprintf(fp, "Cannot setup wiring pi %s\n", strerror(errno));
     exit(1);
   }
   //   pinMode(4, INPUT);
   // Configure the interface.
   // CHANNEL insicates chip select,
   // 500000 indicates bus speed.
   fd = wiringPiSPISetup(CHANNEL, 10000000);

   if (wiringPiISR (INT_PIN, INT_EDGE_RISING, &int_handler) < 0) {
     fprintf(fp, "Unable to setup interrupt %s\n", strerror(errno));
     exit(1);
   }
   
   pthread_create(&rx_thread_ptr, NULL, *rx_thread, NULL);
   pthread_create(&mqtt_thread_ptr, NULL, *mqtt_thread, NULL);
   Init();
   GetMode();
   WriteReg(SX1276_IRQ_FLAGS, 0xff);

   fflush(fp);
   
   pthread_join(rx_thread_ptr, NULL);
   pthread_join(mqtt_thread_ptr, NULL);
   
   for(;;);
}
