# nRF 802.15.4 radio driver

This driver implements only the __non-beacon mode__ of operation.
It supports the following features:
 * [reception of unicast and broadcast frames (with filtering)](#receiving-frames-and-frames-filtering),
 * [automatic sending of ACK frames](#automatic-sending-ack-frames),
 * [setting a pending bit in the ACK frame according to pending data for the given destination](#setting-pending-bit-in-ack-frames),
 * [transmission of unicast and broadcast frames](#transmission-of-unicast-and-broadcast-frames),
 * [automatic CCA procedure before transmission](#automatic-cca-procedure-before-transmission),
 * [automatic receiving of ACK frames](#automatic-receiving-ack-frames),
 * [stand-alone CCA procedure](#stand-alone-cca-procedure),
 * [low power mode (sleep)](#low-power-mode),
 * [energy detection](#energy-detection),
 * [promiscuous mode](#promiscuous-mode),
 * [continuous carrier transmission](#continuous-carrier-transmission) (for radio testing),
 * [dynamic multiprotocol support](#multiprotocol).

## First steps

This section describes how to build the nRF 802.15.4 radio driver and how to implement a simple application using this driver.

### Source files

The driver supports multiple [radio arbiters](#multiprotocol). Depending on the selected arbiter, other files should be selected for the driver build. There are _core files_, which are used regardless of the selected arbiter, and _arbiter-dependent files_.

#### Core files
Source files:
 * nrf_drv_radio802154.c
 * nrf_drv_radio802154_ack_pending_bit.c
 * nrf_drv_radio802154_critical_section.c
 * nrf_drv_radio802154_debug.c
 * nrf_drv_radio802154_fsm.c
 * nrf_drv_radio802154_pib.c
 * nrf_drv_radio802154_rx_buffer.c

Header files:
 * nrf_drv_radio802154.h
 * nrf_drv_radio802154_config.h
 * nrf_drv_radio802154_const.h
 * nrf_drv_radio802154_ack_pending_bit.h
 * nrf_drv_radio802154_critical_section.h
 * nrf_drv_radio802154_debug.h
 * nrf_drv_radio802154_fsm.h
 * nrf_drv_radio802154_notification.h
 * nrf_drv_radio802154_pib.h
 * nrf_drv_radio802154_priority_drop.h
 * nrf_drv_radio802154_procedures_duration.h
 * nrf_drv_radio802154_request.h
 * nrf_drv_radio802154_rx_buffer.h

Hardware access layer files:
 * hal/nrf_clock.h
 * hal/nrf_egu.h
 * hal/nrf_gpio.h
 * hal/nrf_gpiote.h
 * hal/nrf_peripherals.h
 * hal/nrf_ppi.h
 * hal/nrf_radio.h

Radio arbiter abstraction layer headers:
 * raal/nrf_raal_api.h
 * raal/nrf_raal_config.h

#### Arbiter-dependent files

1. Single-PHY
 * nrf_drv_radio802154_notification_direct.c
 * nrf_drv_radio802154_priority_drop_direct.c
 * nrf_drv_radio802154_request_direct.c
 * raal/single_phy/single_phy.c

2. Simulator
 * nrf_drv_radio802154_notification_direct.c
 * nrf_drv_radio802154_priority_drop_direct.c
 * nrf_drv_radio802154_request_direct.c
 * raal/simulator/nrf_raal_simulator.c

3. SoftDevice
 * nrf_drv_radio802154_notification_swi.c
 * nrf_drv_radio802154_priority_drop_swi.c
 * nrf_drv_radio802154_request_swi.c
 * nrf_drv_radio802154_swi.c
 * nrf_drv_radio802154_swi.h
 * raal/softdevice/nrf_raal_softdevice.c
 * raal/softdevice/nrf_raal_softdevice.h

### Defines

A few features of the driver can be enabled using compile-time definitions. These definitions can be provided for the build through the -D compiler option.

 * RAAL_SIGNLE_PHY - This option shall be enabled when the Single-PHY arbiter is in use.
 * RAAL_SIMULATOR - This option shall be enabled when the simulated arbiter is in use.
 * RAAL_SOFTDEVICE - This option shall be enabled when the SoftDevice arbiter is in use.
 * DEBUG_LOG - This option provides an array containing events logged by the driver. [How to decode logs?](#logs)
 * DEBUG_ASSERT - This option disables all IRQ on an assert and enters indefinite loop. It is helpful with the DEBUG_LOG option, because it prevents updating logs from IRQ handlers after the assertion fails.
 * DEBUG_GPIO - This option connects RADIO events with GPIO output. It allows event timing measurements with a logic analyzer.

### Simple application

Transmitter:

```c
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_drv_radio802154.h"

#define MAX_MESSAGE_SIZE 16
#define CHANNEL          11

static volatile bool m_tx_in_progress;
static volatile bool m_tx_done;

int main(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    uint8_t message[MAX_MESSAGE_SIZE];

    for (uint32_t i = 0; i < sizeof(message) / sizeof(message[0]); i++)
    {
        message[i] = i;
    }

    message[0] = 0x41;                // Set MAC header: short addresses, no ACK
    message[1] = 0x98;                // Set MAC header

    m_tx_in_progress = false;
    m_tx_done        = false;

    nrf_drv_radio802154_init();
    nrf_drv_radio802154_channel_set(CHANNEL);
    nrf_drv_radio802154_receive();

    while (1)
    {
        if (m_tx_done)
        {
            m_tx_in_progress = false;
            m_tx_done        = false;
        }

        if (!m_tx_in_progress)
        {
            m_tx_in_progress = nrf_drv_radio802154_transmit(message, sizeof(message), true);
        }
    }

    return 0;
}

void nrf_drv_radio802154_transmitted(uint8_t * p_ack, uint8_t length, int8_t power, int8_t lqi)
{
    (void) length;
    (void) power;
    (void) lqi;

    m_tx_done = true;

    if (p_ack != NULL)
    {
        nrf_drv_radio802154_buffer_free(p_ack);
    }
}
```

Receiver:

```c
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_drv_radio802154.h"

#define MAX_MESSAGE_SIZE 127
#define CHANNEL          11

static uint8_t m_message[MAX_MESSAGE_SIZE];

static volatile uint32_t rx_counter;

int main(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
    uint8_t short_address[]    = {0x06, 0x07};
    uint8_t pan_id[]           = {0x04, 0x05};

    nrf_drv_radio802154_init();

    nrf_drv_radio802154_short_address_set(short_address);
    nrf_drv_radio802154_extended_address_set(extended_address);
    nrf_drv_radio802154_pan_id_set(pan_id);

    nrf_drv_radio802154_channel_set(CHANNEL);
    nrf_drv_radio802154_receive();

    while (1)
    {
        // Intentionally empty
    }

    return 0;
}

void nrf_drv_radio802154_received(uint8_t * p_data, uint8_t length, int8_t power, int8_t lqi)
{
    (void) power;
    (void) lqi;

    if (length > MAX_MESSAGE_SIZE)
    {
        goto exit;
    }

    memcpy(m_message, p_data, length);
    rx_counter++;

exit:
    nrf_drv_radio802154_buffer_free(p_data);

    return;
}
```

### Logs

The driver implements a debug logging feature. Event logs are stored in RAM memory. The logs can be retrieved from RAM by a debugger and decoded to a sequence diagram with the provided `decoder.html` utility.

#### Extracting driver logs using GDB
```
set print elements 0
set pagination off
set height unlimited

p/z nrf_drv_radio802154_debug_log_buffer
p nrf_drv_radio802154_debug_log_ptr
```

#### Creating a sequence diagram based on the extracted logs
Copy the output of nrf_drv_radio802154_debug_log_buffer to the GDB Log area in the decoder.html utility. Then, copy the nrf_drv_radio802154_debug_log_ptr value to the Index pointer field in decoder.html. Then, click Generate sequence diagram. The sequence diagram will be automatically generated.

## Implementation details

This section describes how the driver should be used and how it is internally implemented.

### FSM description

The main part of the driver is an FSM. From API perspective, it has six states. Most of these states contain sub-states in the implementation.

```
                      +---------+  cca()
                      |         | -------> +-----+
                      |         |          | CCA |
                      |         | <------- +-----+
                      |         | cca_done()
                      |         |
            receive() |         | transmit()
   +-------+ -------> |         | -------> +----------+
O->| Sleep |          | Receive |          | Transmit |
   +-------+ <------- |         | <------- +----------+
             sleep()  |         | receive() / transmitted() / transmit_failed()
                      |         |
                      |         | energy_detection()
                      |         | -----------------> +----------------- +
                      |         |                    | Energy detection |
                      |         | <----------------- +------------------+
                      |         | energy_detected()
                      |         |
                      |         | continuous_carrier()
                      |         | -------------------> +--------------------+
                      |         |                      | Continuous carrier |
                      |         | <------------------- +--------------------+
                      +---------+  receive()
```

#### Transitions

The driver is initialized in the Sleep state. The higher layer must call 
the receive() function to make the driver enter the Receive state and start 
radio operations.

In basic applications, the radio should be in the Receive state for most of the time. In this 
state, the radio receives incoming frames. Changing to any other state should be
performed from the Receive state.

When a frame is received in the Receive state, the driver notifies the higher layer 
by calling the received() function. This function is called after reception of a 
broadcast frame or after sending an ACK to a unicast frame.
In the promiscuous mode, the higher layer is notified about all of the received
frames, even if the frame was not destined to the receiving node.

To transmit a frame, the higher layer must call the transmit() function. If the 
channel is busy, the driver goes back to the Receive state and notifies the 
higher layer by calling the transmit_failed() function. If a broadcast frame was 
transmitted, the driver goes back to the Receive state and notifies the higher 
layer by calling the transmitted() function. If a unicast frame was transmitted 
and an ACK was received, the driver goes back to the Receive state and notifies 
the higher layer by calling the transmitted() function. If a unicast frame was 
transmitted and there was no expected ACK received, the higher layer shall call 
the receive() function after the ACK timeout to make the driver go back to the
Receive state.

To perform an Energy Detection procedure, the higher layer must call the
energy_detection() function. When the procedure is completed, the driver automatically goes 
back to the Receive state and notifies the higher layer with the
energy_detected() function.

To transmit carrier continuously, the higher layer must call the 
continuous_carrier() function. Note that the __Continuous carrier state should be used
only in test applications__. In Continuous carrier state, the driver transmits
carrier wave until the receive() function is called by the higher layer.

To perform a stand-alone CCA procedure, the higher layer must call the cca()
function. When the procedure is completed, the driver automatically transits back
to the Receive state and notifies the higher layer with the cca_done() function.

#### States

##### Sleep
In this state, the radio is in low power mode. It cannot transmit or receive any
frames.

##### Receive
In this state, the radio [receives 802.15.4 frames](#receiving-frames-and-frames-filtering). It filters out frames with invalid CRC, length, type, or destination address. 
If the driver receives a unicast frame destined to the receiving node, it can [automatically transmit an ACK frame](#automatic-sending-ack-frames). 

##### Transmit
In this state, the radio can perform the [CCA procedure](#automatic-cca-procedure-before-transmission). If the channel is free, the radio [transmits the requested frame](#transmission-of-unicast-and-broadcast-frames). If an ACK was requested in the transmitted frame, the driver [automatically receives the ACK frame](#automatic-receiving-ack-frames) in this state.

##### Energy detection
In this state, the radio performs the [Energy Detection procedure](#energy-detection). During this procedure the radio is busy and cannot change state to any other. The end of this procedure is notified to the higher layer by a energy_detected() function call.

### Features description

This section describes implementation details of the nRF 802.15.4 radio driver features.

#### Receiving frames and frames filtering

The driver is able to receive unicast and broadcast 802.15.4 frames on channels 11-26 from the channel page 0. The driver performs most of the received frame filtering procedure (IEEE 802.15.4-2006: 7.5.6.2):
 1. When PHR is received (FRAMESTART event), the driver checks if length of the frame is valid.
 2. When FCF is received (first BCMATCH event), the driver verifies the frame type. Version is not checked to support forward-compatibility.
 3. When destination address fields (PAN ID and address) are present and received (second BCMATCH event), the driver checks if the frame is destined to this node (broadcast or unicast). Source PAN ID of beacon frames is not verified, because the driver implements only non-beacon mode.
 4. When te full frame is received (END event), the driver verifies if the FCS field contains a valid value.
If all 4 checks pass, the driver passes the received frame to the MAC layer. 

Note that steps 2 and 3 may be bypassed in [promiscuous mode](#promiscuous-mode).

#### Automatic sending of ACK frames

The MAC layer may configure the driver to automatically send ACK frames (enabled by default). The automatically created ACK frame is compliant with IEEE 802.15.4-2006: 7.2.2.3. This frame is sent exactly 192 us (aTrunaroundTime) after a data frame is received. The ACK frame is sent only if the received frame passes all steps of [the filter](#receiving-frames-and-frames-filtering) (even in promiscuous mode) and if the ACK request bit is present in FCF of the received frame.

Automatic ACK procedure uses the 'Interframe spacing' feature of nRF RADIO. The driver sets TIFS register to 192 us (aTurnarondTime) and enables appropriate shorts (END->DISABLE, DISABLED->TXEN, READY->START). Moreover, after a START task is triggered in the 'receive' state, the driver sets PACKETPTR to ACK frame buffer. If any of the filter steps fails, the driver aborts automatic ACK by disabling all shorts and triggering a DISABLE task. If all filter steps pass, the driver sets the correct sequence number in the ACK frame and optionally performs [automatic pending bit procedure](#setting-pending-bit-in-ack-frames). The ACK frame is sent automatically by RADIO shorts.

If all three shorts (END->DISABLE, DISABLED->TXEN, READY->START) are enabled, it is possible that the RADIO peripheral transmits ACK frames continuously (i.e. when debugger or flash operation stops program code execution). To prevent this situation, the driver never enables all three shorts at once. When any frame is being received, two shorts are enabled: END->DISABLE and DISABLED->TXEN. When the received frame passes all filter steps, the driver disables END->DISABLE short and enables READY->START (it is performed in DISABLED event handler). With this approach, if program code execution is stopped during frame reception, the RADIO peripheral executes END->DISABLE and DISABLED->TXEN shorts and stays in TXIDLE state. This condition is detected by the driver. The frame is dropped and the RADIO is reset.
Note that the RADIO stays in TXIDLE state as long as program code execution is stopped (i.e. by the debugger). In this state, the RADIO transmits carrier continuously. 

#### Setting pending bit in ACK frames

The MAC layer may configure the driver to automatically set pending bit in automatically generated ACK frames (this feature is enabled by default). If this feature is disabled, pending bit in automatically generated ACK frames is always set (1). If this feature is enabled, the driver compares the source address of the data frame (the one that is being acknowledged) with array of entries containing addresses of nodes for which the MAC layer has pending data. If the driver matches the source address with an entry in the array, pending bit is set (1). If the array does not contain an address matching the source address, pending bit is cleared (0). If ACK frame is transmitted before the matching procedure is completed (i.e. due to too big array), pending bit is set (1).

This procedure is performed in the DISABLED event handler when the receiver is disabled in order to enable the transmitter to automatically transmit an ACK frame. Due to DISABLED->TXEN short, this procedure takes place during transmitter ramp up time.

#### Transmission of unicast and broadcast frames

The radio driver allows the MAC layer to transmit a frame containing any PSDU. The RADIO peripheral updates FCS field of every frame automatically. A transmission procedure may be preceded by a CCA procedure. The driver automatically receives an ACK frame if requested.

When the RADIO peripheral is in TXIDLE state, it transmits carrier wave. During frame transmission, it is a waste of energy and channel occupation.
To prevent the TXIDLE peripheral state, the driver uses two shorts during transmission:
1. READY -> START
2. END -> DISABLE
These shorts automatically start transmission of the frame when the transmitter is ready, and disable the transmitter when the frame has been transmitted.

#### Automatic CCA procedure before transmission

The MAC layer may request the driver to perform a CCA procedure prepending transmission. If CCA procedure is requested, the driver performs a CCA procedure. In case the channel is busy, the driver notifies the MAC layer and ends the transmission procedure. In case the channel is idle, the driver starts transmission immediately after the CCA procedure ends.

The driver may be configured to use CCAIDLE->TXEN short or to start the transmission in software. This configuration is selected compile-time by the RADIO_SHORT_CCAIDLE_TXEN definition.

#### Automatic receiving of ACK frames

If FCF of the frame requested for transmission has the ACK request bit cleared, the driver ends transmission procedure and notifies the MAC layer right after the RADIO peripheral ends transmission of the frame. If FCF of the frame has the ACK request bit set, the driver waits for ACK frame. Waiting may be interrupted by three events:
1. The driver receives the expected ACK frame. In this case, the driver resets the receiver, enters the 'receive' state, and notifies the MAC layer that the transmission succeeded.
2. The driver receives other frame than the expected ACK. In this case, the driver resets the receiver, enters the 'receive' state, and notifies the MAC layer that the transmission failed.
3. When ACK timer expires, the MAC layer must call the 'receive()' request to stop waiting for an ACK frame. In this case, the driver resets the receiver and enters the 'receive' state. It does not notify the MAC layer.

#### Stand-alone CCA procedure

The driver is able to perform a stand-alone CCA procedure. The MAC layer should request it when the driver is in the 'receive' state. The driver notifies the MAC layer about the result of the CCA procedure by the cca_done() call. After the CCA procedure ends, the driver enters the 'receive' state.

#### Low power mode

The MAC layer can request the driver to enter low power mode (sleep). In this mode, the RADIO peripheral cannot receive or transmit any frames, but power consumption is minimal.

#### Energy detection

The driver can perform an energy detection procedure for the time given by the MAC layer. This procedure returns maximal energy level detected during the procedure. The time given by the MAC layer is rounded up to multiplication of 128 us.

Note that energy detection procedure in multiprotocol configuration may take a little longer than the requested time. Energy detection procedure is interrupted by other protocols' radio activity, but total time of energy-detection periods is greater or equal to the time requested by the MAC layer.

#### Promiscuous mode

In the promiscuous mode the driver reports to the MAC layer the received frames that either pass all [filter](#receiving-frames-and-frames-filtering) steps or fail steps 2 or 3. If any step of the filter fails, the driver does not [automatically transmit ACK](#automatic-sending-ack-frames) frame in response to the received frame.

#### Continuous carrier transmission

The driver can send continuous carrier wave on a selected channel. This mode is intended for device testing and __should not__ be used in product application. Continuous carrier transmission forces CCA (ED mode) to report a busy channel on nearby devices. The MAC layer should request entering the 'receive' state by the driver to stop continuous carrier transmission.

Continuous carrier is transmitted when the RADIO peripheral is in the TXIDLE state.

#### Multiprotocol

The dynamic multiprotocol feature allows RADIO peripheral sharing between the 802.15.4 driver and other PHY protocol drivers.

Radio drivers request access to the peripheral from an arbiter. The arbiter grants or denies access. nRF 802.15.4 driver is arbiter-agnostic - it can cooperate with any arbiter through the Radio Arbiter Abstraction Layer (RAAL).

Currently there are three arbiter abstractions implemented:
1. Single-PHY - Used when only the 802.15.4 protocol is using the RADIO peripheral.
2. Simulator - Simulated arbiter that simulates concurrent access of Bluetooth Low Energy and 802.15.4 to the radio.
3. SoftDevice - Implementation of SoftDevice's Timeslot API client. It allows concurrent access to the radio by SoftDevice internal protocols (mostly Bluetooth Low Energy) and the  802.15.4 driver.

### Mutex and critical sections

State transitions in the FSM can be requested simultaneously by the higher layer and the IRQ handler. To prevent race conditions in the driver, there is a mutex. The mutex is unlocked only in the *Receive* state (*WaitingRxFrame* substate). If there is a requested state transition, the procedure shall lock the mutex before state is changed. If mutex cannot be locked, another procedure has locked it and is going to change the state. The mutex is unlocked when the driver enters the *Receive* state.

A race condition could also occur during handling of requests from the higher layer. Even if the receiver is stopped (TASK STOP), the END or DISABLED event can be raised for a few uS after triggering the task. To prevent interruption of the higher layer request handler by the IRQ handler, the higher layer request handlers are performed in critical sections. During the critical sections, IRQ from the RADIO peripheral are disabled and [RAAL](#multiprotocol) cannot trigger notifications. These IRQ are handled after the critical section ends.
