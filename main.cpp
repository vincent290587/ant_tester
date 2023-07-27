
#include <cassert>
#include <fstream>
#include "dsi_serial_generic.hpp"
#include "ant_receive_ch.h"
#include "ue5_lib.h"

int fec_device_channel = -1;
USHORT power1, power2;

static void _pw1_callback(UCHAR *p_aucData) {

	UCHAR ucPageNum = p_aucData[0];

	//printf("Dev 0 Page: %d received\n", ucPageNum);

    printf("Receiving FEC page %u \n", ucPageNum);

	// Page specific data
	switch(ucPageNum) // Removing the toggle bit
	{
        case 25:
        {
            power1 = p_aucData[5] + ((p_aucData[6] & 0b00001111) << 8u);
            printf("FEC power %u received\n", power1);
        } break;

		default:

			break;

	}
}

static void _pw2_callback(UCHAR *p_aucData) {

	UCHAR ucPageNum = p_aucData[0];

	//printf("Dev 1 Page: %d received\n", ucPageNum);

	// Page specific data
	switch(ucPageNum) // Removing the toggle bit
	{
		case 16:
		{
			power2 = p_aucData[6] + p_aucData[7]*0xFF;
			printf("Dev 1 Page: power %u received\n", power2);
		}
			break;

		default:

			break;

	}
}

#define PW_PERIOD_COUNTS 8182u
#define PW_DEVICE_TYPE   0x0Bu

static void Loop(void);

BOOL bMyDone = FALSE;


DSI_THREAD_RETURN _io_task(void *pvParameter_)
{
	std::ofstream fParameters;

	fParameters.open("com.txt");

	fParameters << "power1 - power2" << std::endl;

	while (!bMyDone) {
		DSIThread_Sleep(500);

		fParameters << power1;
		fParameters << " - ";
		fParameters << power2 << std::endl;

		fParameters.flush();

	}

	fParameters.close();
	return NULL;
}

#define ANT_FEC_PAGE49_TARGET_POWER_LSB     (0.25f)

#define ANT_FEC_PAGE51_SLOPE_LSB       (1.f/100.f)
#define ANT_FEC_PAGE51_ROLL_RES_LSB    (5.f/100000.f)

#define USE_FEEL
#define USE_SLOPE

DSI_THREAD_RETURN _tx_task(void *pvParameter_)
{
    UCHAR aucTransmitBuffer[ANT_STANDARD_DATA_PAYLOAD_SIZE];

    while (!bMyDone) {
#if !defined( USE_SLOPE )

        DSIThread_Sleep(1000);

        float power = 50; // 50 watts
        uint16_t usPower = (uint16_t)(power / ANT_FEC_PAGE49_TARGET_POWER_LSB);

        aucTransmitBuffer[MESSAGE_BUFFER_DATA1_INDEX] = 49u; // page
        aucTransmitBuffer[MESSAGE_BUFFER_DATA7_INDEX] = usPower & 0xFF; // power part 2
        aucTransmitBuffer[MESSAGE_BUFFER_DATA8_INDEX] = (usPower & 0xFF00) >> 8; // power part 1

        ue5_lib__sendBytes(fec_device_channel, aucTransmitBuffer);

        printf("Transmitting on channel %d ... \n", fec_device_channel);

#else
        DSIThread_Sleep(1000);

        float slope = 1.f; // 1%
        uint16_t usSlope = (uint16_t)((slope / ANT_FEC_PAGE51_SLOPE_LSB) + 0x4e20);
        float roll_res = 0.005f; // 0.005
        uint8_t usroll_res = (uint16_t)(roll_res / ANT_FEC_PAGE51_ROLL_RES_LSB);

        aucTransmitBuffer[MESSAGE_BUFFER_DATA1_INDEX] = 51u; // page
        aucTransmitBuffer[MESSAGE_BUFFER_DATA6_INDEX] = usSlope & 0xFF; // slope part 1 (LSB is 0.5 W)
        aucTransmitBuffer[MESSAGE_BUFFER_DATA7_INDEX] = (usSlope & 0xFF00) >> 8; // slope part 2
        aucTransmitBuffer[MESSAGE_BUFFER_DATA8_INDEX] = usroll_res; // rolling res

        ue5_lib__sendBytes(fec_device_channel, aucTransmitBuffer);

        printf("Transmitting on channel %d ... \n", fec_device_channel);
#endif
#if defined( USE_FEEL )

        DSIThread_Sleep(1000);

        memset(aucTransmitBuffer, 0, ANT_STANDARD_DATA_PAYLOAD_SIZE);

        aucTransmitBuffer[MESSAGE_BUFFER_DATA1_INDEX] = 0xFC; // page

        aucTransmitBuffer[MESSAGE_BUFFER_DATA3_INDEX] = 0x00; // 0 = off 1= isokinetic
        aucTransmitBuffer[MESSAGE_BUFFER_DATA4_INDEX] = 0x00; // 84…168 (15…30km/h) / LSB = 0.05m/s

        aucTransmitBuffer[MESSAGE_BUFFER_DATA6_INDEX] = 0x03; // cobblestone
        aucTransmitBuffer[MESSAGE_BUFFER_DATA7_INDEX] = 0x32; // 50% intensity

        ue5_lib__sendBytes(fec_device_channel, aucTransmitBuffer);

        printf("Transmitting on channel %d ... \n", fec_device_channel);
#endif
    }

    return NULL;
}

#define FEC_DEVICE_TYPE            0x11u               ///< Device type reserved for ANT+ Bicycle Power.
#define FEC_MSG_PERIOD             8192u               ///< Message period, decimal 8182 (4 Hz).

//#define USE_DOUBLE_RECORDING

int main(int argc, char *argv[]) {

    /* Deactivate buffering so that messages appear in journald.
     * This could be done in the service file but the tools are
     * not present on the q8.
     */
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    ue5_lib__startupAntPlusLib();

    fec_device_channel = ue5_lib__addDeviceID(15568u,
                         FEC_DEVICE_TYPE,
                         FEC_MSG_PERIOD,
                         _pw1_callback);
    assert(fec_device_channel >= 0);

#ifdef USE_DOUBLE_RECORDING
    (void)ue5_lib__addDeviceID(0xEFAC,
                         PW_DEVICE_TYPE,
                         PW_PERIOD_COUNTS,
                         _pw2_callback);
#endif

    ue5_lib__startANT();

    DSI_THREAD_ID uiDSIThread;

#ifdef USE_DOUBLE_RECORDING
    uiDSIThread = DSIThread_CreateThread(_io_task, 0);
    assert(uiDSIThread);
#endif

    uiDSIThread = DSIThread_CreateThread(_tx_task, 0);
    assert(uiDSIThread);

    Loop();

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Loop
//
// Start the Test program.
//
////////////////////////////////////////////////////////////////////////////////
static void Loop(void)
{
	printf("Initialisation was successful!\n"); fflush(stdout);

	printf("\n");
	printf("U - Request USB Descriptor\n");
	printf("Q - Quit\n");
	printf("\n");
	fflush(stdout);

	while(!bMyDone)
	{
		UCHAR ucChar;
		char st[1024];
		fgets(st, sizeof(st), stdin);
		sscanf(st, "%c", &ucChar);

		switch(ucChar)
		{
			case 'Q':
			case 'q':
			{
				// Quit
				printf("Closing channels...\n");
				bMyDone = TRUE;
				break;
			}

			default:
			{
				break;
			}
		}

		DSIThread_Sleep(0);
	}

}
