//go:build rp2040
// +build rp2040

package machine

import (
	"device/arm"
	"device/rp"
	"runtime/interrupt"
	"runtime/volatile"
	"unsafe"
)

const (
	// Number of spin locks available
	_NUMSPINLOCKS = 32
	// Number of interrupt handlers available
	_NUMIRQ               = 32
	_PICO_SPINLOCK_ID_IRQ = 9
	_NUMBANK0_GPIOS       = 30
)

// C SDK exposed API for setting interrupt callback
func (p Pin) setIRQ(change PinChange, callback func(Pin)) {
	// TODO
	p.setInterrupt(change, true)
	pinCallbacks[CurrentCore()] = callback
	irqSetExclusiveHandler(rp.IRQ_IO_IRQ_BANK0, gpioirqHandler)
	irqSet(rp.IRQ_IO_IRQ_BANK0, true)
}

// Enable or disable a specific interrupt on the executing core.
// num is the interrupt number which must be in [0,31].
func irqSet(num uint32, enabled bool) {
	if num >= _NUMIRQ {
		return
	}
	irqSetMask(1<<num, enabled)
}

// pico-sdk: All interrupts handlers should be of this type, and follow normal ARM EABI register saving conventions
type irqHandler func()

func irqSetExclusiveHandler(num uint32, handler irqHandler) {
	if num >= _NUMIRQ {
		panic(ErrInvalidInputPin)
	}
	lock := spinLock{}
	lock.init(_PICO_SPINLOCK_ID_IRQ)
	is := lock.lock()
	current := getVtableHandler(num)
	if current != nil {
		panic("can't set handler due to non nil actual handler")
	}
	// current := getVtableHandler(num)
	// C SDK asserts either handler is unset or is same handler
	// hard_assert(current == __unhandled_user_irq || current == handler);
	setVtableHandler(num, handler)
	lock.unlock(is)
}

func setVtableHandler(num uint32, handler irqHandler) {
	// update vtable (vtable_handler may be same or updated depending on cases, but we do it anyway for compactness)
	getVtable()[16+num] = handler
}

func irqSetMask(mask uint32, enabled bool) {
	if enabled {
		// Clear pending before enable
		// (if IRQ is actually asserted, it will immediately re-pend)
		rp.PPB.NVIC_ICPR.Set(mask)
		rp.PPB.NVIC_ISER.Set(mask)
	} else {
		rp.PPB.NVIC_ICER.Set(mask)
	}
}

func gpioirqHandler() {
	var base *irqCtrl
	core := CurrentCore()
	switch core {
	case 0:
		base = &ioBank0.proc0IRQctrl
	case 1:
		base = &ioBank0.proc1IRQctrl
	}
	var gpio Pin
	for gpio = 0; gpio < _NUMBANK0_GPIOS; gpio++ {
		statreg := base.intS[gpio>>3]
		change := getIntChange(gpio, statreg.Get())
		if change != 0 {
			gpio.acknowledgeInterrupt(change)
			callback := pinCallbacks[core]
			if callback != nil {
				callback(gpio)
				return
			} else {
				panic("unset callback in handler")
			}
		}
	}
	panic("gpio INT status not found")
}

// Equivalent to get_vtable()[16+num] from pico-sdk.
func getVtableHandler(num uint32) irqHandler {
	if num >= _NUMIRQ {
		return nil
	}
	tablePtr := getVtable()
	return tablePtr[num+16]
}

func getVtable() *[16 + _NUMIRQ]irqHandler {
	// abuse of unsafe gets us there.
	return (*[16 + _NUMIRQ]irqHandler)(unsafe.Pointer(uintptr(arm.SCB.VTOR.Get())))
}

type spinLock struct {
	reg *volatile.Register32
}

// Initializes spinlock to a spin lock number.
func (lock *spinLock) init(lockID uint32) {
	const SIO_Offset, SIO_SPINLOCK0_OFFSET uintptr = 0xd0000000, 0x00000100
	if lockID >= _NUMSPINLOCKS {
		panic("spinlocknum >= 32")
	}
	lock.reg = (*volatile.Register32)(unsafe.Pointer(SIO_Offset + SIO_SPINLOCK0_OFFSET + uintptr(lockID)*4))
}

// Release a spin lock re-enabling interrupts
func (lock spinLock) unlock(is interrupt.State) {
	memFenceRelease()
	lock.reg.Set(0)
	interrupt.Restore(is)
}

// Acquire a spin lock safely disabling interrupts.
func (lock spinLock) lock() interrupt.State {
	is := interrupt.Disable()
	// Note we don't do a wfe or anything, because by convention these spin_locks are VERY SHORT LIVED and NEVER BLOCK and run
	// with INTERRUPTS disabled (to ensure that)... therefore nothing on our core could be blocking us, so we just need to wait on another core
	// anyway which should be finished soon
	for lock.reg.Get() != 0 {
	}
	memFenceAcquire()
	return is
}

// The DMB (data memory barrier) acts as a memory barrier, all memory accesses prior to this
// instruction will be observed before any explicit access after the instruction.
//go:inline
func memFenceAcquire() { arm.Asm("dmb") }

//go:inline
func memFenceRelease() { arm.Asm("dmb") }
