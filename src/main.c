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
#include <nrfx_timer.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb);
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
    __NOP();
}
/*  TODO Config of Radio changes during runtime have to check that 
    Atleast the frequency changes during runtime probably address to and stuff 
*/
void radio_init(void) {
    nrf_timer_bit_width_set(NRF_TIMER0, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_frequency_set(NRF_TIMER0, NRF_TIMER_FREQ_1MHz);
    nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_CLEAR);
    nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_START);

    nrf_radio_power_set(NRF_RADIO, true);

    nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_0DBM);

    // Low level packet configuration
    nrf_radio_packet_conf_t radioConfig = {0,};
    radioConfig.lflen = 6;
    radioConfig.s0len = 0;
    radioConfig.s1len = 3;
    radioConfig.maxlen = 64; //32 in original
    radioConfig.statlen = 0;
    radioConfig.balen = 4;
    radioConfig.big_endian = true;
    radioConfig.whiteen = false;
    nrf_radio_packet_configure(NRF_RADIO, &radioConfig);

    // Configure channel and bitrate
    nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_NRF_2MBIT); // 1 byte Preamble len 0xAA or 0x55 
    nrf_radio_frequency_set(NRF_RADIO, 2480); //80 for fly 2447 in original //TODO have to check if 80 is correct maybe 2480 or someting

    // Configure Addresses
    nrf_radio_base0_set(NRF_RADIO, 0xe7e7e7e7); // 0xE7E7E7E7 for fly
    nrf_radio_prefix0_set(NRF_RADIO, 0xe7);
    nrf_radio_txaddress_set(NRF_RADIO, 0);
    nrf_radio_rxaddresses_set(NRF_RADIO, 0x01u);
    
    // Should be correct, doenst work with vanilla 
    /*uint64_t address = 0xE7E7E7E7E7ULL;
    nrf_radio_base0_set(NRF_RADIO,bytewise_bitswap((uint32_t)address));
    nrf_radio_base1_set(NRF_RADIO, 0xE7E7E7E7UL);
    nrf_radio_prefix0_set(NRF_RADIO,0xC4C3FF00UL | (bytewise_bitswap(address >> 32) & 0xFF));
    nrf_radio_prefix1_set(NRF_RADIO,0xC5C6C7C8UL);
    nrf_radio_txaddress_set(NRF_RADIO, 0);
    nrf_radio_rxaddresses_set(NRF_RADIO, (1<<0) | (1<<1));    
    */

    // Configure CRC
    nrf_radio_crc_configure(NRF_RADIO, 2, NRF_RADIO_CRC_ADDR_INCLUDE, 0x11021UL);
    nrf_radio_crcinit_set(NRF_RADIO, 0xfffful);
    // same as in the fly 

    // Acquire RSSI at radio address
    //nrf_radio_shorts_enable(NRF_RADIO, NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK | NRF_RADIO_SHORT_DISABLED_RSSISTOP_MASK);
    nrf_radio_shorts_enable(NRF_RADIO, RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_TXEN_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Enabled);
    
    // Enable disabled interrupt only, the rest is handled by shorts
    //nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK);
    nrf_radio_int_enable(NRF_RADIO,RADIO_INTENSET_END_Msk);
    IRQ_CONNECT(RADIO_IRQn, 2, radio_isr, NULL, 0);
    irq_enable(RADIO_IRQn);

    nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_RXEN);

}


void radio_receive(struct esbPacket_s *data, uint8_t *length) {
    NRF_RADIO->PACKETPTR = (uint32_t)&data;
    
    // Reset Radio for new reception
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;

    NRF_RADIO->TASKS_RXEN = 1; // Enable RX Mode
    while (NRF_RADIO->EVENTS_READY == 0); // Wait unit Radio is ramped up
    LOG_DBG("Radio Ramped up");
    NRF_RADIO->TASKS_START = 1; // Start Radio

    while (NRF_RADIO->EVENTS_ADDRESS == 0);
    LOG_DBG("Address matched");
    while (NRF_RADIO->EVENTS_END == 0); // Packet received 
    NRF_RADIO->EVENTS_END = 0;
    LOG_DBG("Packet  received");
    NRF_RADIO->TASKS_DISABLE = 1; // Disable  Radio
    while (NRF_RADIO->EVENTS_DISABLED == 0); // Wait until Radio has been disabled
    LOG_DBG("Radio Dissabled");
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
        printk("message received");
        // Toggle an LED to indicate a reception
        nrf_gpio_pin_toggle(6);

        // Process the received message (e.g., print it)
        if (message_len > 0) {
            //message[message_len] = '\0'; // Null-terminate the received data
            printk("Received: %s\n", message.data);
        }
    }
}
