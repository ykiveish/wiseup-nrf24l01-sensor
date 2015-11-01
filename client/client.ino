#include <SoftwareSerial.h>
#include <SPInterface.h>
#include <nRFModule.h>
#include <wise_rfcomm.h>
#include <wise_common.h>

typedef struct {
	struct client_flags {
		uint32_t server_address_set     : 1;
		uint32_t reserved				: 31;
	};

	client_flags flags;
} client_context_t;

const byte rxPin = 2;
const byte txPin = 3;

uint8_t rx[32];
uint8_t tx[32];
client_context_t local;

SoftwareSerial serialDebug (rxPin, txPin);
spi_interface 	spi	  = spi_interface();
NRF24L01    	nrf	  = NRF24L01(&spi);

uint8_t BROADCAST_ADDR[5]   = {0xFA, 0xFA, 0xFA, 0xFA, 0xFA};
uint8_t LOCAL_ADDR[5]       = {0xFF, 0xFE, 0xFD, 0xFC, 0x01};
uint8_t SERVER_ADDR[5] 	    = {0x00, 0x00, 0x00, 0x00, 0x00};

enum STATES {
	GET_SERVER_ADDRESS,
	SEND_LOCAL_ID,
	WORKING,
	IDLE,
	SLEEPING,
    SEND_TEMPERATURE_DATA
};
enum STATES state = WORKING;

uint8_t temperature_port = 2;
uint8_t temperature_data = 0;
long long check_temperature_value = 0;

void
hardware_layer_data_arrived_handler () {
	rfcomm_data * packet = (rfcomm_data *) rx;
	serialDebug.print ("#[client] Data Arrived ... ");

	// print_buffer (rx, 32);

	switch (packet->data_information.data_type) {
		case SERVER_ADDRESS_BROADCAST:
			if (local.flags.server_address_set == 0x0) {
                serialDebug.println ("#[client] SERVER_ADDRESS_BROADCAST");
				memcpy (SERVER_ADDR, &packet->data_frame, 5);
                nrf.setSourceAddress ((byte *) LOCAL_ADDR);
				nrf.setDestinationAddress ((byte *) SERVER_ADDR);
				local.flags.server_address_set = 0x1;
				state = SEND_LOCAL_ID;
			}
			break;
		case CLIENT_ID_RESPONSE:
			break;
		default:
			break;
	}
}

void setup () {
        serialDebug.begin(9600);
	nrf.init (10, 9); // default csn = 7, ce = 8 (10, 9 on imall board)
	nrf.setSourceAddress		((byte *) BROADCAST_ADDR);
	nrf.setDestinationAddress	((byte *) SERVER_ADDR);

	nrf.setPayload (16);
	nrf.setChannel (99);
	nrf.configure ();
	nrf.setSpeedRate (NRF_250KBPS);

	nrf.rx_buffer_ptr = rx;
	nrf.tx_buffer_ptr = tx;
	nrf.dataRecievedHandler = hardware_layer_data_arrived_handler;
    
	serialDebug.println("#[client] Initialized ...");
}

void loop () {
	switch (state) {
		case GET_SERVER_ADDRESS: {
            serialDebug.println("#[client] GET_SERVER_ADDRESS");
			memset (SERVER_ADDR, 0x00, 5);
			nrf.setSourceAddress		((byte *) BROADCAST_ADDR);
			nrf.setDestinationAddress	((byte *) SERVER_ADDR);
			local.flags.server_address_set = 0x0;
			break;
		}
		case WORKING: {
            // Serial.println("#[client] WORKING");
			nrf.pollListener ();
            
            if (abs (millis () - check_temperature_value > 5000)) {
				state = SEND_TEMPERATURE_DATA;
			}
			break;
		}
		case IDLE:
            serialDebug.println("#[client] IDLE");
			break;
		case SLEEPING:
            serialDebug.println("#[client] SLEEPING");
			break;
		case SEND_LOCAL_ID: {
            serialDebug.println ("#[client] SEND_LOCAL_ID");
            rfcomm_data * packet = (rfcomm_data *) tx;
			build_packet (tx, CLIENT_ID_REQUEST);
            memcpy (&packet->data_frame, LOCAL_ADDR, 5);
			nrf.send ();
            state = WORKING;
			break;
        }
        case SEND_TEMPERATURE_DATA: {
            if (local.flags.server_address_set == 0x1) {
                serialDebug.println ("#[client] SEND_TEMPERATURE_DATA");
                temperature_data = (uint8_t)check_temperature_value;
                rfcomm_data * packet = (rfcomm_data *) tx;
                build_packet (tx, CLIENT_SENSOR_DATA);
                memcpy (&packet->data_frame, LOCAL_ADDR, 5);                  // device id
                packet->data_frame.unframeneted.data[5] = temperature_port;   // port
                packet->data_frame.unframeneted.data[6] = 0x1;                // type
                packet->data_frame.unframeneted.data[7] = 0x1;                // data length
                packet->data_frame.unframeneted.data[8] = temperature_data;   // data
                nrf.send ();
                state = WORKING;
                check_temperature_value = millis ();
            }
            break;
        }
	}

	delay (100);
}
