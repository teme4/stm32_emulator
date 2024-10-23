/************************************************************************************************************************
 * @file System and power-on reset.cpp
 * @author Artemiy Vasilchenko (teme4@mail.ru)
 * @brief
 * @version 1.0
 * @date 23.10.2024
 * @details Upon system and power-on reset, the first operation the application software should perform
 * is to provide all required clock signals to the USB peripheral and subsequently de-assert its
 * reset signal so to be able to access its registers. The whole initialization sequence is
 * hereafter described.
 * As a first step application software needs to activate register macrocell clock and de-assert
 * macrocell specific reset signal using related control bits provided by device clock
 * management logic.
 * After that, the analog part of the device related to the USB transceiver must be switched on
 * using the PDWN bit in CNTR register, which requires a special handling. This bit is intended
 * to switch on the internal voltage references that supply the port transceiver. This circuit has
 * a defined startup time (tSTARTUP specified in the datasheet) during which the behavior of the
 * USB transceiver is not defined. It is thus necessary to wait this time, after setting the PDWN
 * bit in the CNTR register, before removing the reset condition on the USB part (by clearing
 * the FRES bit in the CNTR register). Clearing the ISTR register then removes any spurious
 * pending interrupt before any other macrocell operation is enabled.
 * At system reset, the microcontroller must initialize all required registers and the packet
 * buffer description table, to make the USB peripheral able to properly generate interrupts and
 * data transfers. All registers not specific to any endpoint must be initialized according to the
 * needs of application software (choice of enabled interrupts, chosen address of packet
 * buffers, etc.). Then the process continues as for the USB reset case (see further
 * paragraph).
 * @copyright Quanttelecom Ⓒ
 *
 ***********************************************************************************************************************/
