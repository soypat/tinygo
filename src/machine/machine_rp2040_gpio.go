// +build rp2040

package machine

import (
	"device/rp"
	"runtime/volatile"
	"unsafe"
)

type io struct {
	status volatile.Register32
	ctrl   volatile.Register32
}

type irqCtrl struct {
	intE [4]volatile.Register32
	intS [4]volatile.Register32
	intF [4]volatile.Register32
}

type ioBank0Type struct {
	io                 [30]io
	intR               [4]volatile.Register32
	proc0IRQctrl       irqCtrl
	proc1IRQctrl       irqCtrl
	dormantWakeIRQctrl irqCtrl
}

var ioBank0 = (*ioBank0Type)(unsafe.Pointer(rp.IO_BANK0))

type padsBank0Type struct {
	voltageSelect volatile.Register32
	io            [30]volatile.Register32
}

var padsBank0 = (*padsBank0Type)(unsafe.Pointer(rp.PADS_BANK0))

// pinFunc represents a GPIO function.
//
// Each GPIO can have one function selected at a time.
// Likewise, each peripheral input (e.g. UART0 RX) should only be  selected
// on one GPIO at a time. If the same peripheral input is connected to multiple GPIOs,
// the peripheral sees the logical OR of these GPIO inputs.
type pinFunc uint8

// GPIO function selectors
const (
	fnJTAG pinFunc = 0
	fnSPI  pinFunc = 1
	fnUART pinFunc = 2
	fnI2C  pinFunc = 3
	fnPWM  pinFunc = 4
	fnSIO  pinFunc = 5
	fnPIO0 pinFunc = 6
	fnPIO1 pinFunc = 7
	fnGPCK pinFunc = 8
	fnUSB  pinFunc = 9
	fnNULL pinFunc = 0x1f

	fnXIP pinFunc = 0
)

const (
	PinOutput PinMode = iota
	PinInput
	PinInputPulldown
	PinInputPullup
	PinAnalog
	PinUART
)

// set drives the pin high
func (p Pin) set() {
	mask := uint32(1) << p
	rp.SIO.GPIO_OUT_SET.Set(mask)
}

// clr drives the pin low
func (p Pin) clr() {
	mask := uint32(1) << p
	rp.SIO.GPIO_OUT_CLR.Set(mask)
}

// xor toggles the pin
func (p Pin) xor() {
	mask := uint32(1) << p
	rp.SIO.GPIO_OUT_XOR.Set(mask)
}

// get returns the pin value
func (p Pin) get() bool {
	return rp.SIO.GPIO_IN.HasBits(uint32(1) << p)
}

func (p Pin) ioCtrl() *volatile.Register32 {
	return &ioBank0.io[p].ctrl
}

func (p Pin) padCtrl() *volatile.Register32 {
	return &padsBank0.io[p]
}

func (p Pin) pullup() {
	p.padCtrl().SetBits(rp.PADS_BANK0_GPIO0_PUE)
	p.padCtrl().ClearBits(rp.PADS_BANK0_GPIO0_PDE)
}

func (p Pin) pulldown() {
	p.padCtrl().SetBits(rp.PADS_BANK0_GPIO0_PDE)
	p.padCtrl().ClearBits(rp.PADS_BANK0_GPIO0_PUE)
}

func (p Pin) pulloff() {
	p.padCtrl().ClearBits(rp.PADS_BANK0_GPIO0_PDE)
	p.padCtrl().ClearBits(rp.PADS_BANK0_GPIO0_PUE)
}

// setFunc will set pin function to fn.
func (p Pin) setFunc(fn pinFunc) {
	// Set input enable, Clear output disable
	p.padCtrl().ReplaceBits(rp.PADS_BANK0_GPIO0_IE,
		rp.PADS_BANK0_GPIO0_IE_Msk|rp.PADS_BANK0_GPIO0_OD_Msk, 0)

	// Zero all fields apart from fsel; we want this IO to do what the peripheral tells it.
	// This doesn't affect e.g. pullup/pulldown, as these are in pad controls.
	p.ioCtrl().Set(uint32(fn) << rp.IO_BANK0_GPIO0_CTRL_FUNCSEL_Pos)
}

// init initializes the gpio pin
func (p Pin) init() {
	mask := uint32(1) << p
	rp.SIO.GPIO_OE_CLR.Set(mask)
	p.clr()
}

// Configure configures the gpio pin as per mode.
func (p Pin) Configure(config PinConfig) {
	p.init()
	mask := uint32(1) << p
	switch config.Mode {
	case PinOutput:
		p.setFunc(fnSIO)
		rp.SIO.GPIO_OE_SET.Set(mask)
	case PinInput:
		p.setFunc(fnSIO)
	case PinInputPulldown:
		p.setFunc(fnSIO)
		p.pulldown()
	case PinInputPullup:
		p.setFunc(fnSIO)
		p.pullup()
	case PinAnalog:
		p.setFunc(fnNULL)
		p.pulloff()
	case PinUART:
		p.setFunc(fnUART)
	}
}

// Set drives the pin high if value is true else drives it low.
func (p Pin) Set(value bool) {
	if value {
		p.set()
	} else {
		p.clr()
	}
}

// Get reads the pin value.
func (p Pin) Get() bool {
	return p.get()
}

// Interrupts. Type aliasing to allow
type IRQCallback func(pin Pin, events uint32)

// Interrupt callbacks limited to number of cores like pico-sdk
var irqCallbacks [2]IRQCallback

func (p Pin) IRQWithCallback(events uint32, enabled bool, callback IRQCallback) {
	p.setIRQEnabled(events, enabled)
	// Install IRQ handler
	irqCallbacks[GetCoreNumber()] = callback

}

func (p Pin) setIRQEnabled(events uint32, enabled bool) {
	// Separate mask/force/status per-core, so check which core called, and
	// set the relevant IRQ controls.
	var irqCtlBase *irqCtrl
	switch GetCoreNumber() {
	case 0:
		irqCtlBase = &ioBank0.proc0IRQctrl
	case 1:
		irqCtlBase = &ioBank0.proc1IRQctrl
	}
	p.ackIRQEnabled(events, irqCtlBase)
	enReg := irqCtlBase.intE[p/8]
	events <<= 4 * (p % 8)
	if enabled {
		enReg.SetBits(events)
	} else {
		enReg.ClearBits(events)
	}
}

func (p Pin) ackIRQEnabled(events uint32, base *irqCtrl) {
	ioBank0.intR[p/8].Set(events << 4 * (uint32(p) % 8))
}

func (p Pin) SetDormantIRQ(events uint32, enabled bool) {
	base := &ioBank0.dormantWakeIRQctrl
	p.ctrlSetIRQ(events, enabled, base)
}

func (p Pin) ctrlSetIRQ(events uint32, enabled bool, base *irqCtrl) {
	p.ackIRQEnabled(events, base)
	enReg := base.intE[p/8]
	events <<= 4 * (p % 8)
	if enabled {
		enReg.SetBits(events)
	} else {
		enReg.ClearBits(events)
	}
}
