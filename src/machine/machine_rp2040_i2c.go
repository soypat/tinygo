// +build rp2040

package machine

import (
	"device/rp"
	"errors"
	"strconv"
)

// I2CConfig is used to store config info for I2C.
type I2CConfig struct {
	Frequency uint32
}

type I2C struct {
	Bus           *rp.I2C0_Type
	restartOnNext bool
}

var (
	errInvalidI2CBaudrate = errors.New("invalid i2c baudrate")
	errInvalidTgtAddr     = errors.New("invalid target i2c address not in 0..0x80 or is reserved")
	errI2CTimeout         = errors.New("i2c timeout")
	errI2CGeneric         = errors.New("i2c error")
)

// Tx performs a write and then a read transfer placing the result in
// in r.
//
// Passing a nil value for w or r skips the transfer corresponding to write
// or read, respectively.
//
//  i2c.Tx(addr, nil, r)
// Performs only a read transfer.
//
//  i2c.Tx(addr, w, nil)
// Performs only a write transfer.
func (i2c *I2C) Tx(addr uint16, w, r []byte) error {
	if len(w) > 0 {
		if err := i2c.tx(uint8(addr), w, false, 0); nil != err {
			return err
		}
	}

	if len(r) > 0 {
		if err := i2c.rx(uint8(addr), r, false, 0); nil != err {
			return err
		}
	}

	return nil
}

func (i2c *I2C) Configure(config I2CConfig) error {
	return i2c.init(config)
}

// SetBaudrate sets the I2C frequency. It has the side effect of also
// enabling the I2C hardware if disabled beforehand.
//go:inline
func (i2c *I2C) SetBaudrate(br uint32) error {
	var freqin uint32 = 125 * MHz
	// Find smallest prescale value which puts o

	// TODO there are some subtleties to I2C timing which we are completely ignoring here
	period := (freqin + br/2) / br
	lcnt := period * 3 / 5 // oof this one hurts
	hcnt := period - lcnt
	// Check for out-of-range divisors:
	if hcnt > rp.I2C0_IC_FS_SCL_HCNT_IC_FS_SCL_HCNT_Msk || hcnt < 8 || lcnt > rp.I2C0_IC_FS_SCL_LCNT_IC_FS_SCL_LCNT_Msk || lcnt < 8 {
		return errInvalidI2CBaudrate
	}

	// Per I2C-bus specification a device in standard or fast mode must
	// internally provide a hold time of at least 300ns for the SDA signal to
	// bridge the undefined region of the falling edge of SCL. A smaller hold
	// time of 120ns is used for fast mode plus.

	// sda_tx_hold_count = freq_in [cycles/s] * 300ns * (1s / 1e9ns)
	// Reduce 300/1e9 to 3/1e7 to avoid numbers that don't fit in uint.
	// Add 1 to avoid division truncation.
	sdaTxHoldCnt := ((freqin * 3) / 10000000) + 1
	if br >= 1_000_000 {
		// sda_tx_hold_count = freq_in [cycles/s] * 120ns * (1s / 1e9ns)
		// Reduce 120/1e9 to 3/25e6 to avoid numbers that don't fit in uint.
		// Add 1 to avoid division truncation.
		sdaTxHoldCnt = ((freqin * 3) / 25000000) + 1
	}
	if sdaTxHoldCnt <= lcnt-2 {
		return errInvalidI2CBaudrate
	}
	i2c.disable()
	i2c.Bus.IC_ENABLE.Set(rp.I2C0_IC_ENABLE_ENABLE_DISABLED)
	// Always use "fast" mode (<= 400 kHz, works fine for standard mode too)
	i2c.Bus.IC_CON.ReplaceBits(rp.I2C0_IC_CON_SPEED_FAST, rp.I2C0_IC_CON_SPEED_Msk, rp.I2C0_IC_CON_SPEED_Pos)
	i2c.Bus.IC_FS_SCL_HCNT.Set(hcnt)
	i2c.Bus.IC_FS_SCL_LCNT.Set(lcnt)

	i2c.Bus.IC_FS_SPKLEN.Set(umax32(1, lcnt/16))

	i2c.Bus.IC_SDA_HOLD.ReplaceBits(sdaTxHoldCnt, rp.I2C0_IC_SDA_HOLD_IC_SDA_RX_HOLD_Msk, rp.I2C0_IC_SDA_HOLD_IC_SDA_RX_HOLD_Pos)
	i2c.enable()
	return nil
}

//go:inline
func (i2c *I2C) enable() {
	i2c.Bus.IC_ENABLE.Set(1)
	// i2c.Bus.IC_ENABLE.ReplaceBits(rp.I2C0_IC_ENABLE_ENABLE, rp.I2C0_IC_ENABLE_ENABLE_Msk, rp.I2C0_IC_ENABLE_ENABLE_Pos)
}

//go:inline
func (i2c *I2C) disable() {
	i2c.Bus.IC_ENABLE.Set(0)
	// i2c.Bus.IC_ENABLE.ReplaceBits(rp.I2C0_IC_ENABLE_ENABLE_DISABLED, rp.I2C0_IC_ENABLE_ENABLE_Msk, rp.I2C0_IC_ENABLE_ENABLE_Pos)
}

//go:inline
func (i2c *I2C) init(config I2CConfig) error {
	i2c.reset()
	i2c.disable()
	i2c.restartOnNext = false
	// Configure as a fast-mode master with RepStart support, 7-bit addresses
	i2c.Bus.IC_CON.Set(rp.I2C0_IC_CON_SPEED_FAST<<rp.I2C0_IC_CON_SPEED_Pos |
		rp.I2C0_IC_CON_MASTER_MODE | rp.I2C0_IC_CON_IC_SLAVE_DISABLE |
		rp.I2C0_IC_CON_IC_RESTART_EN | rp.I2C0_IC_CON_TX_EMPTY_CTRL)

	// Set FIFO watermarks to 1 to make things simpler. This is encoded by a register value of 0.
	i2c.Bus.IC_TX_TL.Set(0)
	i2c.Bus.IC_RX_TL.Set(0)

	i2c.Bus.

		// Always enable the DREQ signalling -- harmless if DMA isn't listening
		i2c.Bus.IC_DMA_CR.Set(rp.I2C0_IC_DMA_CR_TDMAE | rp.I2C0_IC_DMA_CR_RDMAE)
	return i2c.SetBaudrate(config.Frequency)
}

func (i2c *I2C) reset() {
	resetVal := i2c.deinit()
	rp.RESETS.RESET.ClearBits(resetVal)
	// Wait until reset is done.
	for !rp.RESETS.RESET_DONE.HasBits(resetVal) {
	}
}

// deinit sets reset bit for I2C. Must call reset to reenable I2C after deinit.
//go:inline
func (i2c *I2C) deinit() (resetVal uint32) {
	switch {
	case i2c.Bus == rp.I2C0:
		resetVal = rp.RESETS_RESET_I2C0
	case i2c.Bus == rp.I2C1:
		resetVal = rp.RESETS_RESET_I2C1
	}
	// Perform I2C reset.
	rp.RESETS.RESET.SetBits(resetVal)

	return resetVal
}

func (i2c *I2C) tx(addr uint8, tx []byte, nostop bool, timeout int64) (err error) {
	if addr >= 0x80 || isReservedI2CAddr(addr) {
		return errInvalidTgtAddr
	}
	tlen := len(tx)
	// Quick return if possible.
	if tlen == 0 {
		return nil
	}

	i2c.disable()
	i2c.Bus.IC_TAR.Set(uint32(addr))
	i2c.enable()
	// If no timeout was passed timeoutCheck is false.
	timeoutCheck := timeout != 0
	abort := false
	var abortReason uint32
	byteCtr := 0
	for ; byteCtr < tlen; byteCtr++ {
		first := byteCtr == 0
		last := byteCtr == tlen-1
		i2c.Bus.IC_DATA_CMD.Set(
			boolToBit(first && i2c.restartOnNext)<<rp.I2C0_IC_DATA_CMD_RESTART_Pos |
				boolToBit(last && !nostop)<<rp.I2C0_IC_DATA_CMD_STOP_Pos |
				uint32(tx[byteCtr]))
		// Wait until the transmission of the address/data from the internal
		// shift register has completed. For this to function correctly, the
		// TX_EMPTY_CTRL flag in IC_CON must be set. The TX_EMPTY_CTRL flag
		// was set in i2c_init.
		for i2c.Bus.IC_RAW_INTR_STAT.Get()&rp.I2C0_IC_RAW_INTR_STAT_TX_EMPTY != 0 {
			if timeoutCheck { //&& time.Since(deadline) > 0 {
				i2c.restartOnNext = nostop
				return errI2CTimeout // If there was a timeout, don't attempt to do anything else.
			}
		}

		abortReason = i2c.Bus.IC_TX_ABRT_SOURCE.Get()
		if abortReason != 0 {
			// Note clearing the abort flag also clears the reason, and
			// this instance of flag is clear-on-read! Note also the
			// IC_CLR_TX_ABRT register always reads as 0.
			i2c.Bus.IC_CLR_TX_ABRT.Get()
			abort = true
		}
		if abort || (last && !nostop) {
			// If the transaction was aborted or if it completed
			// successfully wait until the STOP condition has occured.

			// TODO Could there be an abort while waiting for the STOP
			// condition here? If so, additional code would be needed here
			// to take care of the abort.
			for i2c.Bus.IC_RAW_INTR_STAT.Get()&rp.I2C0_IC_RAW_INTR_STAT_STOP_DET != 0 {
				if timeoutCheck { //} && time.Since(deadline) > 0 {
					i2c.restartOnNext = nostop
					return errI2CTimeout
				}
			}
			i2c.Bus.IC_CLR_STOP_DET.Get()
		}
	}

	// From Pico SDK: A lot of things could have just happened due to the ingenious and
	// creative design of I2C. Try to figure things out.
	if abort {
		switch {
		case abortReason == 0 || abortReason&rp.I2C0_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK != 0:
			// No reported errors - seems to happen if there is nothing connected to the bus.
			// Address byte not acknowledged
			err = errI2CGeneric
		case abortReason&rp.I2C0_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK != 0:
			// Address acknowledged, some data not acknowledged
			fallthrough
		default:
			// panic("unknown i2c abortReason:" + strconv.Itoa(abortReason)
			err = makeI2CBuffError(byteCtr)
		}
	}

	// nostop means we are now at the end of a *message* but not the end of a *transfer*
	i2c.restartOnNext = nostop
	return err
}

func (i2c *I2C) rx(addr uint8, rx []byte, nostop bool, deadline int64) (err error) {
	if addr >= 0x80 || isReservedI2CAddr(addr) {
		return errInvalidTgtAddr
	}
	rlen := len(rx)
	// Quick return if possible.
	if rlen == 0 {
		return nil
	}
	i2c.disable()
	i2c.Bus.IC_TAR.Set(uint32(addr))
	i2c.enable()
	// If no timeout was passed timeoutCheck is false.
	timeoutCheck := deadline == 0 // !deadline.Equal(time.Time{})
	abort := false
	var abortReason uint32
	byteCtr := 0
	for ; byteCtr < rlen; byteCtr++ {
		first := byteCtr == 0
		last := byteCtr == rlen-1
		for i2c.writeAvailable() == 0 {
		}
		i2c.Bus.IC_DATA_CMD.Set(
			boolToBit(first && i2c.restartOnNext)<<rp.I2C0_IC_DATA_CMD_RESTART_Pos |
				boolToBit(last && !nostop)<<rp.I2C0_IC_DATA_CMD_STOP_Pos |
				rp.I2C0_IC_DATA_CMD_CMD)

		for i2c.readAvailable() == 0 && !abort {
			abortReason = i2c.Bus.IC_TX_ABRT_SOURCE.Get()
			if abortReason != 0 {
				abort = true
			}
			if timeoutCheck { //} && time.Since(deadline) > 0 {
				i2c.restartOnNext = nostop
				return errI2CTimeout // If there was a timeout, don't attempt to do anything else.
			}
		}
		if abort {
			break
		}
		rx[byteCtr] = uint8(i2c.Bus.IC_DATA_CMD.Get())
	}

	if abort {
		switch {
		case abortReason == 0 || abortReason&rp.I2C0_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK != 0:
			// No reported errors - seems to happen if there is nothing connected to the bus.
			// Address byte not acknowledged
			err = errI2CGeneric
		default:
			// undefined abort sequence
			err = makeI2CBuffError(byteCtr)
		}
	}

	i2c.restartOnNext = nostop
	return err
}

// writeAvailable determines non-blocking write space available
//go:inline
func (i2c *I2C) writeAvailable() uint32 {
	// const size_t IC_TX_BUFFER_DEPTH = 16; // This is the C implementation.
	// return IC_TX_BUFFER_DEPTH - i2c_get_hw(i2c)->txflr;
	return rp.I2C0_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_Pos - i2c.Bus.IC_RXFLR.Get()
}

// readAvailable determines number of bytes received
//go:inline
func (i2c *I2C) readAvailable() uint32 {
	return i2c.Bus.IC_RXFLR.Get()
}

type i2cBuffError int

func (b i2cBuffError) Error() string {
	return "i2c err after addr ack at data " + strconv.Itoa(int(b))
}

//go:inline
func makeI2CBuffError(idx int) error {
	return i2cBuffError(idx)
}

//go:inline
func boolToBit(a bool) uint32 {
	if a {
		return 1
	}
	return 0
}

//go:inline
func umax32(a, b uint32) uint32 {
	if a > b {
		return a
	}
	return b
}

//go:inline
func isReservedI2CAddr(addr uint8) bool {
	return (addr&0x78) == 0 || (addr&0x78) == 0x78
}
