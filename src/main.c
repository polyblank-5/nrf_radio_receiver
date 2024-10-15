#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
//#include "zephyr/drivers/gpio.h"
#include "hal/nrf_gpio.h"
//#include "nrf_gpio.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_radio.h>
#include <zephyr/irq.h>

/* Public API */

// S1 is used for compatibility with NRF24L0+. These three bits are used
// to store the PID and NO_ACK.
#define PACKET0_S1_SIZE                  (3UL)
// S0 is not used
#define PACKET0_S0_SIZE                  (0UL)
// The size of the packet length field is 6 bits
#define PACKET0_PAYLOAD_SIZE             (6UL)
// The size of the base address field is 4 bytes
#define PACKET1_BASE_ADDRESS_LENGTH      (4UL)
// Don't use any extra added length besides the length field when sending
#define PACKET1_STATIC_LENGTH            (0UL)
// Max payload allowed in a packet
#define PACKET1_PAYLOAD_SIZE             (63UL)

static uint64_t address = 0xE7E7E7E7E7ULL;

struct esbPacket_s {
    uint8_t length;
    uint8_t s1;
    char data[32];
} __attribute__((packed));

static uint32_t swap_bits(uint32_t inp)
{
  uint32_t i;
  uint32_t retval = 0;

  inp = (inp & 0x000000FFUL);

  for(i = 0; i < 8; i++)
  {
    retval |= ((inp >> i) & 0x01) << (7 - i);
  }

  return retval;
}

static uint32_t bytewise_bitswap(uint32_t inp)
{
  return (swap_bits(inp >> 24) << 24)
       | (swap_bits(inp >> 16) << 16)
       | (swap_bits(inp >> 8) << 8)
       | (swap_bits(inp));
}


#define RADIO_CHANNEL 80  // Should match the transmitter's channel
static void radio_isr(void *arg){
    
}

void radio_init(void) {
     NRF_RADIO->POWER = 1;


    NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);


        NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos);


    NRF_RADIO->FREQUENCY = 80;


    // Radio address config
    // We use local addresses 0 and 1
    //  * local address 0 is the unique address of the Crazyflie, used for 1-to-1 communication.
    //    This can be set dynamically and the current address is stored in EEPROM.
    //  * local address 1 is used for broadcasts
    //    This is currently 0xFFE7E7E7E7.
    NRF_RADIO->PREFIX0 = 0xC4C3FF00UL | (bytewise_bitswap(address >> 32) & 0xFF);  // Prefix byte of addresses 3 to 0
    NRF_RADIO->PREFIX1 = 0xC5C6C7C8UL;  // Prefix byte of addresses 7 to 4
    NRF_RADIO->BASE0   = bytewise_bitswap((uint32_t)address);  // Base address for prefix 0
    NRF_RADIO->BASE1   = 0xE7E7E7E7UL;  // Base address for prefix 1-7
    NRF_RADIO->TXADDRESS = 0x00UL;      // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = (1<<0) | (1<<1);    // Enable device address 0 and 1 to use which receiving

    // Packet configuration
    NRF_RADIO->PCNF0 = (PACKET0_S1_SIZE << RADIO_PCNF0_S1LEN_Pos) |
                        (PACKET0_S0_SIZE << RADIO_PCNF0_S0LEN_Pos) |
                        (PACKET0_PAYLOAD_SIZE << RADIO_PCNF0_LFLEN_Pos);

    // Packet configuration
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos)    |
                        (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos)           |
                        (PACKET1_BASE_ADDRESS_LENGTH << RADIO_PCNF1_BALEN_Pos)       |
                        (PACKET1_STATIC_LENGTH << RADIO_PCNF1_STATLEN_Pos)           |
                        (PACKET1_PAYLOAD_SIZE << RADIO_PCNF1_MAXLEN_Pos);

    // CRC Config
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
    NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
    NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1

    // Enable interrupt for end event
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;

    // Set all shorts so that RSSI is measured and only END is required interrupt
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
    NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RSSISTOP_Enabled;

    // Set RX buffer and start RX
    //NRF_RADIO->PACKETPTR = (uint32_t)&data;
    NRF_RADIO->TASKS_RXEN = 1U;
    IRQ_CONNECT(RADIO_IRQn, 3, radio_isr, NULL, 0);
    irq_enable(RADIO_IRQn);
}


void radio_receive(struct esbPacket_s *data, uint8_t *length) {
    NRF_RADIO->PACKETPTR = (uint32_t)&data;
    
    // Reset Radio for new reception
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;

    NRF_RADIO->TASKS_RXEN = 1; // Enable RX Mode
    while (NRF_RADIO->EVENTS_READY == 0); // Wait unit Radio is ramped up

    NRF_RADIO->TASKS_START = 1; // Start Radio

    while (NRF_RADIO->EVENTS_ADDRESS == 0);

    while (NRF_RADIO->EVENTS_END == 0); // Packet received 
    NRF_RADIO->EVENTS_END = 0;

    NRF_RADIO->TASKS_DISABLE = 1; // Disable  Radio
    while (NRF_RADIO->EVENTS_DISABLED == 0); // Wait until Radio has been disabled

    *length = NRF_RADIO->RXMATCH; // received address 
}

int main(void) {
    printk("Start");
    nrf_gpio_cfg_output(6); // LED on pin 20
    k_msleep(100);
    nrf_gpio_pin_toggle(6);
    radio_init();

    while (1) {
        struct esbPacket_s message; // Adjust the buffer size as needed
        uint8_t message_len = 0;

        // Receive a message
        radio_receive(&message, &message_len);

        // Toggle an LED to indicate a reception
        nrf_gpio_pin_toggle(6);

        // Process the received message (e.g., print it)
        if (message_len > 0) {
            //message[message_len] = '\0'; // Null-terminate the received data
            printk("Received: %s\n", message.data);
        }
    }
}
